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
    virtual types::Trajectory predict_obstacle_trajectory(uint32_t target_id, float time_horizon, float time_step) const = 0;
    virtual types::CollisionReport check_trajectory_collision(const types::Trajectory& candidate_path, float start_time, float speed_profile) const = 0;
    virtual float get_dynamic_risk_field(const types::Point& position, float timestamp) const = 0;
};

// Interface Principal do Gêmeo Digital (Gerenciador do Buffer)
class DigitalTwinCore {
public:
    // Método chamado pelo ROS2 (Assíncrono) para atualizar os dados internamente
    void update_static_map(const MapData& map);
    void update_vehicle_pose(const types::Pose& pose);
    void update_dynamic_targets(const std::vector<types::Target>& targets);

    // Método chamado pela NAVEGAÇÃO (Síncrono, Thread-Safe, Lock-free ou low-latency)
    // Retorna um ponteiro inteligente para o snapshot atualizado (Double Buffering)
    std::shared_ptr<const WorldStateSnapshot> get_latest_state() const;
};

} // namespace dt