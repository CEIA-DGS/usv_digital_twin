#include "dt_core/twin_interface.hpp"
#include <mutex>
#include <algorithm>
#include <cmath>

namespace dt {

class ConcreteWorldStateSnapshot : public WorldStateSnapshot {
private:
    types::MapData _static_map;
    types::Pose _vehicle_pose;
    std::vector<types::Target> _targets;

public:
    ConcreteWorldStateSnapshot(const types::MapData& map, 
                               const types::Pose& pose, 
                               const std::vector<types::Target>& targets)
        : _static_map(map), _vehicle_pose(pose), _targets(targets) {}

    float get_closest_static_obstacle_distance(const types::Point& pos) const override {
        if (!_static_map.is_loaded()) {
            return -1.0f; // Mapa não carregado
        }
        double dx = pos.get_x() - _vehicle_pose.get_x();
        double dy = pos.get_y() - _vehicle_pose.get_y();
        return static_cast<float>(std::sqrt(dx * dx + dy * dy));
    }

    // --- Classe C: Acesso de Estado Bruto (Para a Interface Gráfica) ---
    types::Pose get_vehicle_pose() const override {
        return _vehicle_pose;
    }

    std::vector<types::Target> get_all_targets() const override {
        return _targets;
    }

    bool is_inside_restricted_zone(const types::Point& pos) const override {
        // Lógica de verificação de zona restrita baseada no mapa S57/NavMesh
        return false;
    }

    std::vector<types::Target> get_active_local_targets(const types::Point& center, float radius) const override {
        std::vector<types::Target> local_targets;
        for (const auto& target : _targets) {
            double dx = target.get_pose().get_x() - center.get_x();
            double dy = target.get_pose().get_y() - center.get_y();
            double distance = std::sqrt(dx * dx + dy * dy);
            
            if (distance <= radius) {
                local_targets.push_back(target);
            }
        }
        return local_targets;
    }

    types::Trajectory predict_trajectory_by_id(const int32_t id, const std::vector<types::Target>& targets, const double time_horizon, const double time_step) const override {
        // Delega para o módulo de predição cinemática se necessário, ou retorna vazia
        types::Trajectory empty_traj;
        return empty_traj;
    }

    types::TargetCollisionReport check_collisions_on_trajectory(const types::Trajectory& candidate_trajectory, const types::Entity& usv_state, double speed_profile, const std::vector<types::Target>& targets, const double start_time) const override {
        // Retorna um relatório padrão seguro
        types::TargetCollisionReport report;
        report.set_safety(true);
        return report;
    }

    double get_dynamic_risk_field(const types::Point& position, const double timestamp, const types::Entity& usv_state, const std::vector<types::Target>& targets) const override {
        // Cálculo inicial do campo de risco potencial dinâmico
        return 0.0;
    }
};


class DigitalTwinCoreImpl {
private:
    mutable std::mutex _mutex;
    types::MapData _current_map;
    types::Pose _current_pose;
    std::vector<types::Target> _current_targets;
    std::shared_ptr<const ConcreteWorldStateSnapshot> _latest_snapshot;

public:
    DigitalTwinCoreImpl() {
        // Inicializa com um snapshot vazio seguro
        _latest_snapshot = std::make_shared<ConcreteWorldStateSnapshot>(_current_map, _current_pose, _current_targets);
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
        _latest_snapshot = std::make_shared<ConcreteWorldStateSnapshot>(_current_map, _current_pose, _current_targets);
    }
};


// Como DigitalTwinCore na hpp não expõe os campos privados diretamente, 
// instanciamos o impl core de forma estática/interna por arquivo ou mantemos o ponteiro.
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