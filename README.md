
---

## 1. Estrutura de Diretórios

```text
usv_digital_twin_ws/
├── dt_msgs/                      # Pacote 1: Apenas definições de mensagens ROS
│   ├── msg/
│   │   ├── TrackedGeoTarget.msg
│   │   └── TrackedGeoTargetArray.msg
│   └── CMakeLists.txt
│
├── dt_core/                      # Pacote 2: NÚCLEO PURO C++ (ZERO ROS2 AQUI)
│   ├── include/dt_core/
│   │   ├── types.hpp             # Tipos de dados agnósticos (Point, Pose, etc)
│   │   ├── map/                  # Equipe 1: Processamento BSB/KAP e R-Tree
│   │   ├── prediction/           # Equipe 2: Cinemática e predição de obstáculos
│   │   └── twin_interface.hpp    # A Interface unificada do Buffer (Double Buffering)
│   ├── src/
│   │   ├── map/
│   │   ├── prediction/
│   │   └── twin_interface.cpp
│   └── CMakeLists.txt
│
├── dt_ros/                       # Pacote 3: O ADAPTADOR ROS2 (Equipe 4)
│   ├── include/dt_ros/
│   │   └── dt_node.hpp           # Nó que faz Ingestão e atualiza o dt_core
│   ├── src/
│   │   └── dt_node.cpp           # Converte ROS msgs -> dt_core::types -> atualiza buffer
│   └── CMakeLists.txt
│
└── dt_viz/                       # Pacote 4: VISUALIZAÇÃO (Equipe 3)
    ├── include/dt_viz/
    ├── src/                      # Pode ser um plugin do RViz2 ou uma UI em Qt
    └── CMakeLists.txt

```

---

## 2. Estrutura 

```cpp
// dt_core/include/dt_core/twin_interface.hpp
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
    virtual std::vector<types::Pose> predict_obstacle_trajectory(uint32_t target_id, float time_horizon, float time_step) const = 0;
    virtual types::CollisionReport check_trajectory_collision(const std::vector<types::Pose>& candidate_path, float start_time, float speed_profile) const = 0;
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

```