#include "dt_core/twin_interface.hpp"
#include <mutex>
#include <algorithm>
#include <cmath>
#include <memory>
#include "prediction/prediction.hpp"
#include "map/IndiceEspacial.hpp"
#include "dt_core/types.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

namespace dt {

class ConcreteWorldStateSnapshot : public WorldStateSnapshot {
private:
    types::MapData _static_map;
    types::Pose _vehicle_pose;
    std::vector<types::Target> _targets;
    
    // Ponteiro compartilhado para o motor espacial já carregado
    std::shared_ptr<IndiceEspacial> _indice_espacial;

public:
    ConcreteWorldStateSnapshot(const types::MapData& map, 
                               const types::Pose& pose, 
                               const std::vector<types::Target>& targets,
                               std::shared_ptr<IndiceEspacial> indice_espacial)
        : _static_map(map), _vehicle_pose(pose), _targets(targets), _indice_espacial(indice_espacial) {
        
        // Atualiza a lista de alvos dinâmicos no motor espacial para este snapshot
        if (_indice_espacial) {
            _indice_espacial->update_global_targets(_targets);
        }
    }

    float get_closest_static_obstacle_distance(const types::Point& pos) const override {
        if (!_indice_espacial) return -1.0f;
        return _indice_espacial->get_closest_static_obstacle_distance(pos);
    }

    // --- Classe C: Acesso de Estado Bruto (Para a Interface Gráfica) ---
    types::Pose get_vehicle_pose() const override {
        return _vehicle_pose;
    }

    std::vector<types::Target> get_all_targets() const override {
        return _targets;
    }

    bool is_inside_restricted_zone(const types::Point& pos) const override {
        if (!_indice_espacial) return false;
        return _indice_espacial->is_inside_restricted_zone(pos);
    }

    std::vector<types::Target> get_active_local_targets(const types::Point& center, float radius) const override {
        if (!_indice_espacial) return std::vector<types::Target>();
        return _indice_espacial->get_active_local_targets(center, radius);
    }

    types::Trajectory predict_trajectory_by_id(const int32_t id, const std::vector<types::Target>& targets, const double time_horizon, const double time_step) const override {
        return prediction::predict_trajectory_by_id(id, targets, time_horizon, time_step);
    }

    types::TargetCollisionReport check_collisions_on_trajectory(const types::Trajectory& candidate_trajectory, const types::Entity& usv_state, double speed_profile, const std::vector<types::Target>& targets, const double start_time) const override {
        return prediction::check_collisions_on_trajectory(candidate_trajectory, usv_state, speed_profile, targets, start_time);
    }

    double get_dynamic_risk_field(const types::Point& position, const double timestamp, const types::Entity& usv_state, const std::vector<types::Target>& targets) const override {
        return prediction::get_dynamic_risk_field(position, timestamp, usv_state, targets);
    }
};


class DigitalTwinCoreImpl {
private:
    mutable std::mutex _mutex;
    types::MapData _current_map;
    types::Pose _current_pose;
    std::vector<types::Target> _current_targets;
    std::shared_ptr<const ConcreteWorldStateSnapshot> _latest_snapshot;
    
    // Instância única do Motor Espacial mantida em memória
    std::shared_ptr<IndiceEspacial> _motor_espacial;

public:
    DigitalTwinCoreImpl() {
        // 1. Instancia o motor espacial
        _motor_espacial = std::make_shared<IndiceEspacial>();
        
        // 2. Carrega os Shapefiles do disco para a RAM apenas UMA VEZ na inicialização
        std::string dt_core_share_dir = ament_index_cpp::get_package_share_directory("dt_core");
        std::string dir = dt_core_share_dir + "/data/output/NavMesh_Shapefiles_BR501511";
        _motor_espacial->carregarShapefiles(dir + "/2_Margem_Seguranca.shp", dir + "/4_Malha_NavMesh.shp");

        // 3. Inicializa com um snapshot seguro repassando o motor espacial
        _latest_snapshot = std::make_shared<ConcreteWorldStateSnapshot>(_current_map, _current_pose, _current_targets, _motor_espacial);
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

private:
    void refresh_snapshot_unlocked() {
        // A cada atualização (pose, alvo, mapa), gera um novo snapshot passando o motor já carregado
        _latest_snapshot = std::make_shared<ConcreteWorldStateSnapshot>(_current_map, _current_pose, _current_targets, _motor_espacial);
    }
};


static DigitalTwinCoreImpl& get_core_impl() {
    static DigitalTwinCoreImpl instance;
    return instance;
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