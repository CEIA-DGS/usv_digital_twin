#pragma once
#include "dt_core/types.hpp"
#include <vector>
#include <memory>

namespace dt {

// Essa classe representa uma "fotografia" do mundo em um instante de tempo.
// É ela que o algoritmo de navegação vai consultar velozmente.
class WorldStateSnapshot {
public:
    // --- Classe A: Representação do Ambiente ---
    virtual float get_closest_static_obstacle_distance(const types::Point& pos) const = 0;
    virtual bool is_inside_restricted_zone(const types::Point& pos) const = 0;
    virtual std::vector<types::Target> get_active_local_targets(const types::Point& center, float radius) const = 0;

    // --- Classe B: Suporte ao Planejamento ---
    virtual types::Trajectory predict_trajectory_by_id(const int32_t id, const std::vector<types::Target>& targets, const double time_horizon, const double time_step) const = 0;
    virtual types::TargetCollisionReport check_collisions_on_trajectory(const types::Trajectory& candidate_trajectory, const types::Entity& usv_state, double speed_profile, const std::vector<types::Target>& targets, const double start_time = 0.0) const = 0;
    virtual double get_dynamic_risk_field(const types::Point& position, const double timestamp, const types::Entity& usv_state, const std::vector<types::Target>& targets) const = 0;
    
    // --- Classe C: Acesso de Estado Bruto (Para a Interface Gráfica) ---
    virtual types::Pose get_vehicle_pose() const = 0;
    virtual std::vector<types::Target> get_all_targets() const = 0;

    virtual types::Trajectory get_planned_trajectory() const = 0;
};

// Interface Principal do Gêmeo Digital (Gerenciador do Buffer)
class DigitalTwinCore {
public:
    // Método chamado pelo ROS2 (Assíncrono) para atualizar os dados internamente
    void update_static_map(const types::MapData& map);
    void update_vehicle_pose(const types::Pose& pose);
    void update_dynamic_targets(const std::vector<types::Target>& targets);
    void update_planned_trajectory(const types::Trajectory& traj);

    // Método chamado pela NAVEGAÇÃO (Síncrono, Thread-Safe, Lock-free ou low-latency)
    // Retorna um ponteiro inteligente para o snapshot atualizado (Double Buffering)
    std::shared_ptr<const WorldStateSnapshot> get_latest_state() const;
};

} // namespace dt