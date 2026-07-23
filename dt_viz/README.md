# DT Viz - Interface Gráfica

Pacote responsável pela interface gráfica do módulo Digital Twin do projeto CEIA-DGS.

A aplicação foi desenvolvida em **C++**, utilizando **Qt 5** e **ROS 2 Humble**, com o objetivo de apresentar uma visualização bidimensional em vista superior dos principais elementos relacionados à navegação do USV.

A interface permite visualizar:

* Zona livre de navegação;
* Posição e orientação do USV;
* Embarcações monitoradas;
* Trajetória percorrida pelo USV;
* Derrota planejada;
* Waypoints ativos;
* Alertas visuais de risco de colisão.

---

## Objetivo

O pacote `dt_viz` tem como objetivo fornecer uma ferramenta de diagnóstico visual para os dados produzidos pelos demais módulos do sistema.

Nesta etapa, a interface recebe:

* A derrota planejada do USV;
* Os waypoints ativos da rota;
* Alertas de colisão associados a uma embarcação específica.

Quando um alerta de colisão é recebido, a interface identifica o alvo pelo MMSI e altera sua aparência visual.

---

## Tecnologias utilizadas

* Ubuntu 22.04;
* ROS 2 Humble;
* C++17;
* Qt 5 Widgets;
* `rclcpp`;
* `nav_msgs`;
* `dt_msgs`;
* `ament_cmake`;
* `colcon`.

---

## Estrutura do pacote

```text
dt_viz/
├── CMakeLists.txt
├── package.xml
├── README.md
├── include/
│   └── dt_viz/
│       ├── main_window.hpp
│       └── viz_node.hpp
└── src/
    ├── main.cpp
    ├── main_window.cpp
    └── viz_node.cpp
```

---

## Descrição dos arquivos

### `include/dt_viz/main_window.hpp`

Declara a classe principal da interface gráfica.

Esse arquivo contém:

* Estrutura dos waypoints;
* Componentes Qt;
* Objetos gráficos da cena;
* Representação do USV;
* Representação das embarcações;
* Representação da derrota planejada;
* Mapeamento dos alvos por MMSI;
* Métodos para atualização visual;
* Métodos para alertas de colisão.

Principais métodos públicos:

```cpp
void updatePlannedRoute(
  const std::vector<RoutePoint> & route);

void updateCollisionAlert(
  std::uint32_t mmsi,
  bool collision_imminent);
```

### `src/main_window.cpp`

Implementa a camada visual da aplicação.

É responsável por:

* Configurar a janela principal;
* Criar a cena gráfica;
* Desenhar a grade e os eixos;
* Desenhar a zona livre;
* Representar o USV;
* Representar as embarcações;
* Atualizar a trajetória;
* Desenhar a derrota planejada;
* Criar os marcadores de waypoints;
* Atualizar os rótulos;
* Destacar uma embarcação com risco de colisão.

### `include/dt_viz/viz_node.hpp`

Declara o nó ROS 2 que conecta os tópicos do sistema à interface gráfica.

O nó recebe:

```text
/planned_route
/collision_alert
```

### `src/viz_node.cpp`

Implementa os subscribers ROS 2.

O callback da rota recebe uma mensagem do tipo:

```text
nav_msgs/msg/Path
```

Cada pose da mensagem é convertida em um ponto da rota e enviada para a interface.

O callback de colisão recebe:

```text
dt_msgs/msg/CollisionAlert
```

O MMSI e o estado do alerta são encaminhados à janela principal.

### `src/main.cpp`

Inicializa:

* ROS 2;
* Aplicação Qt;
* Janela principal;
* Nó de visualização;
* Processamento periódico dos callbacks ROS 2.

O ROS 2 é processado utilizando:

```cpp
rclcpp::spin_some(node);
```

Esse processamento é chamado por um `QTimer`, mantendo as atualizações da interface na thread principal do Qt.

---

## Dependências

O pacote depende de:

```text
rclcpp
nav_msgs
dt_msgs
Qt 5 Widgets
```

O pacote `dt_msgs` deve estar disponível na mesma workspace e precisa gerar a mensagem:

```text
dt_msgs/msg/CollisionAlert
```

---

## Tópicos utilizados

### Derrota planejada

Nome:

```text
/planned_route
```

Tipo:

```text
nav_msgs/msg/Path
```

A mensagem contém uma sequência ordenada de poses. Cada posição representa um waypoint da rota ativa.

Fluxo:

```text
Módulo de Navegação
        ↓
nav_msgs/msg/Path
        ↓
/planned_route
        ↓
VizNode
        ↓
MainWindow
        ↓
Linha da derrota e waypoints
```

### Alerta de colisão

Nome:

```text
/collision_alert
```

Tipo:

```text
dt_msgs/msg/CollisionAlert
```

Fluxo:

```text
Módulo Preditivo
        ↓
dt_msgs/msg/CollisionAlert
        ↓
/collision_alert
        ↓
VizNode
        ↓
Busca da embarcação pelo MMSI
        ↓
Alteração visual do alvo
```

---

## Funcionamento da derrota planejada

Ao receber uma mensagem em `/planned_route`, a interface:

1. Lê a lista de poses;
2. Converte cada pose em um waypoint;
3. Remove a rota anterior;
4. Desenha uma linha ligando os pontos;
5. Cria marcadores para cada waypoint;
6. Adiciona rótulos como `WP1`, `WP2`, `WP3`.

A representação utiliza:

```cpp
QGraphicsPathItem
```

para a linha da rota e:

```cpp
QGraphicsEllipseItem
```

para os marcadores dos waypoints.

---

## Funcionamento do alerta de colisão

Ao receber um alerta, a interface utiliza o campo MMSI para localizar a embarcação correta.

Quando:

```text
collision_imminent: true
```

a interface:

* Muda a cor do alvo;
* Aumenta seu tamanho;
* Reforça o contorno;
* Altera o texto exibido;
* Informa visualmente o risco de colisão.

Quando:

```text
collision_imminent: false
```

a aparência normal do alvo é restaurada.

---

## Requisitos

O ambiente recomendado é:

```text
Ubuntu 22.04
ROS 2 Humble
Qt 5
C++17
```

Instale as dependências necessárias:

```bash
sudo apt update

sudo apt install -y \
  build-essential \
  cmake \
  python3-colcon-common-extensions \
  qtbase5-dev \
  qtbase5-dev-tools
```

---

## Organização na workspace

A estrutura esperada é:

```text
usv_digital_twin_ws/
└── src/
    └── usv_digital_twin/
        ├── dt_msgs/
        └── dt_viz/
```

O pacote `dt_msgs` deve estar presente porque `dt_viz` utiliza a mensagem `CollisionAlert`.

---

## Instalação das dependências

Na raiz da workspace:

```bash
cd ~/usv_digital_twin_ws
```

Carregue o ROS 2:

```bash
source /opt/ros/humble/setup.bash
```

Instale as dependências:

```bash
rosdep install \
  --from-paths src \
  --ignore-src \
  -r \
  -y
```

---

## Compilação

Como o `dt_viz` depende de `dt_msgs`, compile os pacotes respeitando a ordem de dependências:

```bash
cd ~/usv_digital_twin_ws

source /opt/ros/humble/setup.bash

colcon build \
  --packages-up-to dt_viz \
  --symlink-install
```

Depois carregue a workspace:

```bash
source install/setup.bash
```

Também é possível compilar separadamente:

```bash
colcon build --packages-select dt_msgs
source install/setup.bash

colcon build \
  --packages-select dt_viz \
  --symlink-install
source install/setup.bash
```

---

## Execução

Execute a interface:

```bash
ros2 run dt_viz dt_visualizer_node
```

A janela deverá ser aberta com a visualização gráfica.

---

## Teste da derrota planejada

Em outro terminal, carregue o ambiente:

```bash
source /opt/ros/humble/setup.bash
source ~/usv_digital_twin_ws/install/setup.bash
```

Publique uma rota:

```bash
ros2 topic pub --once \
/planned_route \
nav_msgs/msg/Path \
"{
  header: {
    frame_id: map
  },
  poses: [
    {
      header: {frame_id: map},
      pose: {
        position: {x: 0.0, y: 0.0, z: 0.0},
        orientation: {w: 1.0}
      }
    },
    {
      header: {frame_id: map},
      pose: {
        position: {x: 80.0, y: 30.0, z: 0.0},
        orientation: {w: 1.0}
      }
    },
    {
      header: {frame_id: map},
      pose: {
        position: {x: 170.0, y: 90.0, z: 0.0},
        orientation: {w: 1.0}
      }
    },
    {
      header: {frame_id: map},
      pose: {
        position: {x: 270.0, y: 140.0, z: 0.0},
        orientation: {w: 1.0}
      }
    }
  ]
}"
```

A interface deverá exibir a linha da derrota e os waypoints.

---

## Teste do alerta de colisão

Nesta versão, os alvos simulados utilizam MMSIs fictícios:

```text
710000001
710000002
710000003
```

Para ativar o alerta na segunda embarcação:

```bash
ros2 topic pub --once \
/collision_alert \
dt_msgs/msg/CollisionAlert \
"{
  mmsi: 710000002,
  collision_imminent: true,
  time_to_collision: 12.5,
  closest_approach_distance: 8.0
}"
```

Para remover o alerta:

```bash
ros2 topic pub --once \
/collision_alert \
dt_msgs/msg/CollisionAlert \
"{
  mmsi: 710000002,
  collision_imminent: false,
  time_to_collision: -1.0,
  closest_approach_distance: -1.0
}"
```

---

## Inspeção dos tópicos

Liste os tópicos disponíveis:

```bash
ros2 topic list
```

Verifique o tipo da rota:

```bash
ros2 topic type /planned_route
```

Saída esperada:

```text
nav_msgs/msg/Path
```

Verifique o tipo do alerta:

```bash
ros2 topic type /collision_alert
```

Saída esperada:

```text
dt_msgs/msg/CollisionAlert
```

Observe as mensagens:

```bash
ros2 topic echo /planned_route
```

```bash
ros2 topic echo /collision_alert
```

---

## Imagem da interface

Adicione uma captura de tela em:

```text
docs/interface-dt-viz.png
```

Depois, inclua no README:

```markdown
![Interface gráfica do DT Viz](docs/interface-dt-viz.png)
```

Resultado esperado:

![Interface gráfica do DT Viz](docs/interface-dt-viz.png)

---

## Estado atual

O pacote está preparado para:

* Receber a derrota planejada;
* Desenhar os waypoints ativos;
* Receber alertas de colisão;
* Localizar embarcações pelo MMSI;
* Alterar visualmente somente o alvo em risco.

As posições das embarcações ainda podem ser simuladas na interface, dependendo do estágio de integração com os outros módulos.

---

## Próximas etapas

* Receber `dt_msgs/msg/AisReport`;
* Criar e atualizar alvos diretamente a partir dos dados AIS;
* Remover os alvos simulados;
* Exibir tempo estimado até a colisão;
* Exibir distância mínima estimada;
* Diferenciar níveis de risco;
* Integrar o sistema de logging;
* Definir QoS em conjunto com os outros módulos;
* Adicionar testes automatizados para os callbacks.

---

## Pontos de alinhamento

Antes da integração definitiva com os demais módulos, devem ser confirmados:

* Nome oficial do tópico da rota;
* Nome oficial do tópico de colisão;
* Sistema de coordenadas utilizado;
* `frame_id`;
* Unidade das coordenadas;
* Convenção do heading;
* Identificação dos alvos pelo MMSI;
* Frequência das mensagens;
* Política de QoS.

A implementação atual considera:

```text
Rota:
  tópico: /planned_route
  tipo: nav_msgs/msg/Path

Alerta:
  tópico: /collision_alert
  tipo: dt_msgs/msg/CollisionAlert

Sistema de coordenadas:
  frame_id: map
  unidade: metros

Identificador:
  MMSI
```

---

## Critérios de conclusão

A implementação pode ser considerada concluída quando:

* O pacote `dt_viz` compilar;
* A interface abrir corretamente;
* `/planned_route` for recebido;
* A linha da derrota for exibida;
* Os waypoints forem identificados;
* `/collision_alert` for recebido;
* O alvo for localizado pelo MMSI;
* O alvo em risco mudar visualmente;
* O alerta puder ser removido;
* Os demais elementos da interface continuarem funcionando.
