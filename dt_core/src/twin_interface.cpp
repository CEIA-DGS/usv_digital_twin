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

/**
 * @brief Concrete implementation of the WorldStateSnapshot interface.
 * 
 * Represents an immutable snapshot of the world state at a specific point in time,
 * including static map data, vehicle pose, dynamic targets, and spatial engine references.
 */
class ConcreteWorldStateSnapshot : public WorldStateSnapshot {
private:
    types::MapData _static_map;                          ///< Snapshot of the static map data.
    types::Pose _vehicle_pose;                           ///< Snapshot of the vehicle's pose.
    std::vector<types::Target> _targets;                 ///< Snapshot of currently tracked dynamic targets.
    types::Trajectory _planned_trajectory;               ///< Snapshot of the currently planned trajectory.
    
    std::shared_ptr<SpatialIndex> _spatial_index;        ///< Shared pointer to the already loaded spatial engine.

public:
    /**
     * @brief Constructs a new ConcreteWorldStateSnapshot.
     * @param map Current static map data.
     * @param pose Current vehicle pose.
     * @param targets Current list of dynamic targets.
     * @param planned_trajectory Currently planned navigation trajectory.
     * @param spatial_index Shared pointer to the initialized spatial indexing engine.
     */
    ConcreteWorldStateSnapshot(const types::MapData& map, 
                               const types::Pose& pose, 
                               const std::vector<types::Target>& targets,
                               const types::Trajectory& planned_trajectory, 
                               std::shared_ptr<SpatialIndex> spatial_index)
        : _static_map(map), 
          _vehicle_pose(pose), 
          _targets(targets), 
          _planned_trajectory(planned_trajectory), 
          _spatial_index(spatial_index) {
        
        // Updates the spatial engine with the snapshot's targets for localized queries
        if (_spatial_index) {
            _spatial_index->update_global_targets(_targets);
        }
    }

    /**
     * @brief Calculates the distance to the nearest static obstacle.
     * @param pos The position to calculate the distance from.
     * @return Distance in meters, or -1.0f if the spatial index is unavailable.
     */
    float get_closest_static_obstacle_distance(const types::Point& pos) const override {
        if (!_spatial_index) return -1.0f;
        return _spatial_index->get_closest_static_obstacle_distance(pos);
    }

    /**
     * @brief Retrieves the planned trajectory at the time of the snapshot.
     * @return The planned trajectory structure.
     */
    types::Trajectory get_planned_trajectory() const override {
        return _planned_trajectory;
    }

    // --- Class C: Raw State Access (For the Graphical Interface) ---
    
    /**
     * @brief Gets the vehicle's raw pose.
     * @return The 2D pose (x, y, yaw).
     */
    types::Pose get_vehicle_pose() const override {
        return _vehicle_pose;
    }

    /**
     * @brief Gets the complete list of tracked targets.
     * @return Vector of dynamic targets.
     */
    std::vector<types::Target> get_all_targets() const override {
        return _targets;
    }

    /**
     * @brief Verifies if a given point lies within a restricted/non-navigable zone.
     * @param pos The coordinates to check.
     * @return true if restricted, false if navigable.
     */
    bool is_inside_restricted_zone(const types::Point& pos) const override {
        if (!_spatial_index) return false;
        return _spatial_index->is_inside_restricted_zone(pos);
    }

    /**
     * @brief Retrieves dynamic targets within a specific radius from a center point.
     * @param center The central point for the search.
     * @param radius The search radius in meters.
     * @return Vector of targets found within the radius.
     */
    std::vector<types::Target> get_active_local_targets(const types::Point& center, float radius) const override {
        if (!_spatial_index) return std::vector<types::Target>();
        return _spatial_index->get_active_local_targets(center, radius);
    }

    /**
     * @brief Predicts the trajectory of a specific target over a given time horizon.
     */
    types::Trajectory predict_trajectory_by_id(const int32_t id, const std::vector<types::Target>& targets, const double time_horizon, const double time_step) const override {
        return prediction::predict_trajectory_by_id(id, targets, time_horizon, time_step);
    }

    /**
     * @brief Evaluates potential collisions along a candidate trajectory against known targets.
     */
    types::TargetCollisionReport check_collisions_on_trajectory(const types::Trajectory& candidate_trajectory, const types::Entity& usv_state, double speed_profile, const std::vector<types::Target>& targets, const double start_time) const override {
        return prediction::check_collisions_on_trajectory(candidate_trajectory, usv_state, speed_profile, targets, start_time);
    }

    /**
     * @brief Calculates a dynamic risk field value for a given position and time based on target kinematics.
     */
    double get_dynamic_risk_field(const types::Point& position, const double timestamp, const types::Entity& usv_state, const std::vector<types::Target>& targets) const override {
        return prediction::get_dynamic_risk_field(position, timestamp, usv_state, targets);
    }
};

/**
 * @brief Singleton implementation core for the Digital Twin.
 * 
 * Manages the mutable global state of the environment and generates thread-safe
 * immutable snapshots for consumption by other modules.
 */
class DigitalTwinCoreImpl {
private:
    mutable std::mutex _mutex;                                           ///< Mutex for thread-safe state updates.
    types::MapData _current_map;                                         ///< Latest map data.
    types::Pose _current_pose;                                           ///< Latest vehicle pose.
    std::vector<types::Target> _current_targets;                         ///< Latest dynamic targets list.
    std::shared_ptr<const ConcreteWorldStateSnapshot> _latest_snapshot;  ///< Cached latest state snapshot.
    types::Trajectory _current_planned_trajectory;                       ///< Latest planned trajectory.
    
    std::shared_ptr<SpatialIndex> _spatial_engine;                       ///< Single instance of the Spatial Engine kept in memory.

public:
    /**
     * @brief Initializes the core implementation and loads base environmental data.
     */
    DigitalTwinCoreImpl() {
        _spatial_engine = std::make_shared<SpatialIndex>();
        
        // Loads the Shapefiles from disk to RAM only ONCE upon initialization
        std::string dt_core_share_dir = ament_index_cpp::get_package_share_directory("dt_core");
        std::string dir = dt_core_share_dir + "/data/output/NavMesh_Shapefiles_BR501511";
        _spatial_engine->load_shapefiles(dir + "/2_Margem_Seguranca.shp", dir + "/4_Malha_NavMesh.shp");

        // Passes the empty trajectory initially to construct the first valid snapshot
        _latest_snapshot = std::make_shared<ConcreteWorldStateSnapshot>(
            _current_map, 
            _current_pose, 
            _current_targets, 
            _current_planned_trajectory, 
            _spatial_engine);
    }

    /**
     * @brief Thread-safe update for the planned trajectory.
     */
    void update_planned_trajectory(const types::Trajectory& traj) {
        std::lock_guard<std::mutex> lock(_mutex);
        _current_planned_trajectory = traj;
        refresh_snapshot_unlocked();
    }

    /**
     * @brief Thread-safe update for the static map.
     */
    void update_static_map(const types::MapData& map) {
        std::lock_guard<std::mutex> lock(_mutex);
        _current_map = map;
        refresh_snapshot_unlocked();
    }

    /**
     * @brief Thread-safe update for the vehicle pose.
     */
    void update_vehicle_pose(const types::Pose& pose) {
        std::lock_guard<std::mutex> lock(_mutex);
        _current_pose = pose;
        refresh_snapshot_unlocked();
    }

    /**
     * @brief Thread-safe update for dynamic targets.
     */
    void update_dynamic_targets(const std::vector<types::Target>& targets) {
        std::lock_guard<std::mutex> lock(_mutex);
        _current_targets = targets;
        refresh_snapshot_unlocked();
    }

    /**
     * @brief Retrieves the latest generated thread-safe snapshot.
     */
    std::shared_ptr<const WorldStateSnapshot> get_latest_state() const {
        std::lock_guard<std::mutex> lock(_mutex);
        return _latest_snapshot;
    }

private:
    /**
     * @brief Internal helper to regenerate the cached snapshot (must be called with mutex locked).
     */
    void refresh_snapshot_unlocked() {
        _latest_snapshot = std::make_shared<ConcreteWorldStateSnapshot>(
            _current_map, 
            _current_pose, 
            _current_targets, 
            _current_planned_trajectory,
            _spatial_engine);
    }
};

/**
 * @brief Global accessor for the DigitalTwinCoreImpl singleton.
 * @return Reference to the single static instance of DigitalTwinCoreImpl.
 */
static DigitalTwinCoreImpl& get_core_impl() {
    static DigitalTwinCoreImpl instance;
    return instance;
}

// -----------------------------------------------------------------------------
// DigitalTwinCore Static Interface Implementation
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