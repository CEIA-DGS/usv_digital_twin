/**
 * @file twin_interface.cpp
 * @brief Internal implementation of the Digital Twin Core and Snapshots.
 */

#include "dt_core/twin_interface.hpp"
#include <mutex>
#include <algorithm>
#include <cmath>
#include <memory>
#include "prediction/prediction.hpp"
#include "map/spatial_index.hpp"
#include "dt_core/types.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

namespace dt {

// -----------------------------------------------------------------------------
// Internal Immutable Snapshot Implementation
// -----------------------------------------------------------------------------
class ConcreteWorldStateSnapshot : public WorldStateSnapshot {
private:
    types::MapData _static_map;
    types::Pose _vehicle_pose;
    std::vector<types::Target> _targets;
    types::Trajectory _planned_trajectory;
    std::shared_ptr<SpatialIndex> _spatial_index;

public:
    ConcreteWorldStateSnapshot(const types::MapData& map,  
                               const types::Pose& pose, 
                               const std::vector<types::Target>& targets,
                               const types::Trajectory& planned_trajectory, 
                               std::shared_ptr<SpatialIndex> spatial_index)
        : _static_map(map), _vehicle_pose(pose), _targets(targets), 
          _planned_trajectory(planned_trajectory), _spatial_index(spatial_index) {
        
        if (_spatial_index) {
            _spatial_index->update_global_targets(_targets);
        }
    }

    float get_closest_static_obstacle_distance(const types::Point& pos) const override {
        if (!_spatial_index) return -1.0f;
        return _spatial_index->get_closest_static_obstacle_distance(pos);
    }

    types::Trajectory get_planned_trajectory() const override {
        return _planned_trajectory;
    }

    types::Pose get_vehicle_pose() const override {
        return _vehicle_pose;
    }

    std::vector<types::Target> get_all_targets() const override {
        return _targets;
    }

    bool is_inside_restricted_zone(const types::Point& pos) const override {
        if (!_spatial_index) return false;
        return _spatial_index->is_inside_restricted_zone(pos);
    }

    std::vector<types::Target> get_active_local_targets(const types::Point& center, float radius) const override {
        if (!_spatial_index) return std::vector<types::Target>();
        return _spatial_index->get_active_local_targets(center, radius);
    }

    types::Trajectory predict_trajectory_by_id(int32_t id, const std::vector<types::Target>& targets, double time_horizon, double time_step) const override {
        return prediction::predict_trajectory_by_id(id, targets, time_horizon, time_step);
    }

    types::TargetCollisionReport check_collisions_on_trajectory(const types::Trajectory& candidate_trajectory, const types::Entity& usv_state, double speed_profile, const std::vector<types::Target>& targets, double start_time) const override {
        return prediction::check_collisions_on_trajectory(candidate_trajectory, usv_state, speed_profile, targets, start_time);
    }

    double get_dynamic_risk_field(const types::Point& position, double timestamp, const types::Entity& usv_state, const std::vector<types::Target>& targets) const override {
        return prediction::get_dynamic_risk_field(position, timestamp, usv_state, targets);
    }

    types::Trajectory make_trajectory_between(const types::Point& origin, const types::Point& destination, const double step) const override {
        return prediction::make_trajectory_between(origin, destination, step);
    }
};

// -----------------------------------------------------------------------------
// Singleton Core Implementation
// -----------------------------------------------------------------------------
class DigitalTwinCoreImpl {
private:
    mutable std::mutex _mutex;
    types::MapData _current_map;
    types::Pose _current_pose;
    std::vector<types::Target> _current_targets;
    types::Trajectory _current_planned_trajectory;
    
    std::shared_ptr<const ConcreteWorldStateSnapshot> _latest_snapshot;
    std::shared_ptr<SpatialIndex> _spatial_engine;

    void refresh_snapshot_unlocked() {
        _latest_snapshot = std::make_shared<ConcreteWorldStateSnapshot>(
            _current_map, _current_pose, _current_targets, 
            _current_planned_trajectory, _spatial_engine);
    }

public:
    DigitalTwinCoreImpl() {
        _spatial_engine = std::make_shared<SpatialIndex>();
        
        // Loads Shapefiles to RAM once during initialization
        std::string dt_core_share_dir = ament_index_cpp::get_package_share_directory("dt_core");
        std::string dir = dt_core_share_dir + "/data/output/NavMesh_Shapefiles_BR501511";
        _spatial_engine->load_shapefiles(dir + "/2_Margem_Seguranca.shp", dir + "/4_Malha_NavMesh.shp");

        _latest_snapshot = std::make_shared<ConcreteWorldStateSnapshot>(
            _current_map, _current_pose, _current_targets, 
            _current_planned_trajectory, _spatial_engine);
    }

    void update_planned_trajectory(const types::Trajectory& traj) {
        std::lock_guard<std::mutex> lock(_mutex);
        _current_planned_trajectory = traj;
        refresh_snapshot_unlocked();
    }

    void update_static_map(const types::MapData& map) {
        std::lock_guard<std::mutex> lock(_mutex);
        _current_map = map;
        refresh_snapshot_unlocked();
    }

    void update_vehicle_pose(const types::Pose& pose) {
        std::lock_guard<std::mutex> lock(_mutex);
        _current_pose = pose;
        refresh_snapshot_unlocked();
    }

    void update_dynamic_targets(const std::vector<types::Target>& targets) {
        std::lock_guard<std::mutex> lock(_mutex);
        _current_targets = targets;
        refresh_snapshot_unlocked();
    }

    std::shared_ptr<const WorldStateSnapshot> get_latest_state() const {
        std::lock_guard<std::mutex> lock(_mutex);
        return _latest_snapshot;
    }
};

static DigitalTwinCoreImpl& get_core_impl() {
    static DigitalTwinCoreImpl instance;
    return instance;
}

// -----------------------------------------------------------------------------
// DigitalTwinCore Public Wrapper Methods
// -----------------------------------------------------------------------------

void DigitalTwinCore::update_planned_trajectory(const types::Trajectory& traj) {
    get_core_impl().update_planned_trajectory(traj);
}

void DigitalTwinCore::update_static_map(const types::MapData& map) {
    get_core_impl().update_static_map(map);
}

void DigitalTwinCore::update_vehicle_pose(const types::Pose& pose) {
    get_core_impl().update_vehicle_pose(pose);
}

void DigitalTwinCore::update_dynamic_targets(const std::vector<types::Target>& targets) {
    get_core_impl().update_dynamic_targets(targets);
}

std::shared_ptr<const WorldStateSnapshot> DigitalTwinCore::get_latest_state() const {
    return get_core_impl().get_latest_state();
}

} // namespace dt