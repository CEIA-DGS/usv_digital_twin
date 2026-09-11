# Digital Twin - CEIA-DGS

O **Digital Twin** é um módulo de simulação, predição e visualização em tempo real desenvolvido para Veículos de Superfície Não Tripulados (USVs). 

Este sistema é capaz de processar cartas náuticas oficiais (S-57), gerar malhas de navegação (NavMesh) em tempo de execução, calcular dinamicamente o risco de colisão e exibir diagnósticos operacionais em uma interface gráfica dedicada.

---

## Arquitetura do Sistema

O projeto foi dividido em quatro pacotes para garantir a separação de responsabilidades (SoC) e a independência do motor matemático em relação ao middleware de comunicação:

*   **`dt_core`**: Motor central em C++ puro (independente do ROS). Responsável por todo o processamento geométrico (GDAL/Boost), R-Trees, estruturas de dados matemáticas (Types) e os algoritmos de predição de risco de colisão.
*   **`dt_msgs`**: Definição das mensagens customizadas (IDL) para tráfego de dados de navegação e alertas no barramento do ROS 2.
*   **`dt_ros`**: Nó adaptador (Padrão Adapter). Atua como uma ponte, inscrevendo-se nos tópicos de sensores do USV e traduzindo os dados brutos para injetá-los no `dt_core`.
*   **`dt_viz`**: Interface Gráfica de Diagnóstico desenvolvida em Qt5. Renderiza a NavMesh, a telemetria do USV, os alvos dinâmicos e os alertas visuais de colisão, rodando em uma thread assíncrona.

---

## 📂 Estrutura de Diretórios

```text
usv_digital_twin/
└── src/
    ├── dt_core/                # Módulo central em C++ (Lógica, Mapeamento e Predição independentes do ROS)
    │   ├── CMakeLists.txt
    │   ├── package.xml
    │   ├── config.json
    │   ├── include/
    │   │   ├── dt_core/
    │   │   ├── map/
    │   │   └── prediction/
    │   └── src/
    │       ├── twin_interface.cpp
    │       ├── map/
    │       └── prediction/
    │
    ├── dt_msgs/                  # Definição das mensagens customizadas para o barramento ROS 2
    │   ├── CMakeLists.txt
    │   ├── package.xml
    │   └── msg/
    │
    ├── dt_ros/                   # Nó Adaptador (Recebe tópicos ROS e injeta no Core)
    │   ├── CMakeLists.txt
    │   ├── package.xml
    │   ├── include/
    │   │   └── dt_ros/
    │   └── src/
    │
    └── dt_viz/                   # Interface Gráfica de Diagnóstico (Qt5)
        ├── CMakeLists.txt
        ├── package.xml
        ├── README.md
        ├── include/
        │   └── dt_viz/
        └── src/
```

## ⚙️ Pré-Requisitos e Dependências

O sistema foi homologado para Ubuntu 22.04 rodando ROS 2 Humble.

Certifique-se de instalar as dependências de sistema (compiladores, bibliotecas geográficas e interface gráfica) antes de realizar o build:

``` bash
sudo apt update
sudo apt install -y \
  build-essential \
  cmake \
  python3-colcon-common-extensions \
  libgdal-dev \
  libglfw3-dev \
  libgl1-mesa-dev \
  libboost-all-dev \
  qtbase5-dev \
  qtbase5-dev-tools
```

Aviso para usuários de WSL (Windows Subsystem for Linux):
Para evitar erros de permissão ou bugs no gerador de mensagens do ROS (rosidl), certifique-se de que o seu workspace esteja localizado no sistema de arquivos nativo do Linux (ex: ~/usv_digital_twin_ws) e não na partição montada do Windows (/mnt/c/...).

Crie a pasta do Workspace e uma pasta ```src``` dentro dele:
```bash
mkdir $nome_do_workspace$
cd $nome_do_workspace$
mkdir src
cd src
```


## 📚 Clone do Repositório
Clone o repositório dentro da pasta ```src``` que acabamos de criar usando o comando abaixo. Dê preferência a clonar o repositório num diretório cujo caminho não possua espaços no nome, isso pode causar problemas de compilação.
``` bash
git clone https://github.com/CEIA-DGS/usv_digital_twin.git
```

## 🚀 Compilação (Build)

Para a compilação ser executada adequadamente o workspace deve estar organizado da seguinte forma:
``` text
$nome_do_workspace$/
└── src/
    └── usv_digital_twin/       # clone do repositório
```

Devido à hierarquia de dependências (dt_viz depende de dt_ros, que depende de dt_msgs e dt_core), utilize o colcon na raiz do seu workspace para compilar na ordem correta:

``` bash
cd ~/$nome_do_workspace$
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
```

Caso faça alterações no código, para re-compilar as alterações rode:

``` bash
colcon build --cmake-clean-cache
```


## 🖥️ Execução do código compilado

``` bash
source ~/$nome_do_workspace$/install/setup.bash
ros2 run dt_viz dt_visualizer_node
```

Isso deve abrir o módulo do gêmeo digital
![alt text](<README images/image.png>)


## Conexão com Unity
Para conectar o visualizador com o Unity clone o repositório a seguir dentro da pasta ```src```:
```bash
cd ~/$nome_do_workspace$/src

git clone https://github.com/Unity-Technologies/ROS-TCP-Endpoint.git
```

Uma pasta ```ROS-TCP-Endpoint``` deve ter sido criada dentro de ```src```, entre nela:
```bash
cd ~/$nome_do_workspace$/src/ROS-TCP-Endpoint
```

e dê checkout na branch ```main-ros2```
```bash
git checkout main-ros2
```

Por fim, recompile:

```bash
cd ~/$nome_do_workspace$
colcon build --symlink-install
```

Para se conectar ao Unity, rode:
```bash
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0
```

## No Unity
Faça login no unity hub com as seguintes credenciais:
```
email: darkfox769@gmail.com
senha: rt34Ubr@87
```

Após realizar o login, clone o repositório ```SimuladorVSNT```:

![alt text](<README images/image-1.png>)

E preencha da seguinte forma:
![alt text](<README images/image-2.png>)

Após abrir o simulador, na pasta ```Scenes``` abra ```SampleScene```, deve ver algo parecido, o barco não necessariamente aparecerá centralizado:
![alt text](<README images/image-3.png>)

No menu ```Robotics```, acesse o submenu ```Generate ROS Messages```:
![alt text](<README images/image-4.png>)

Em ```ROS Message path``` localize a pasta ```dt_msgs``` dentro do workspace criado, deve estar em algo do tipo ```/$nome_do_workspace$/src/usv_digital_twin/dt_msgs``` (1). Assim que selecionar o camino, deve aparecer o submenu ```dt_msgs``` (2). Clique nele e depois clique em Build 4 msgs (3).
![alt text](<README images/image-6.png>)

Agora precisamos descobrir o IP do computador, no wsl basta rodar o seguinte comando no terminal:

```bash
ifconfig
```

No wsl o IP que funciona é o destacado em amarelo na imagem:
![alt text](<README images/image-5.png>)

Com este IP, preencha no Unity no menu ```Robotics``` ```ROS Settigns```:
![alt text](<README images/image-7.png>)

![alt text](<README images/image-8.png>)

## Integralizando
Dê o play na simulação:

![alt text](<README images/image-9.png>)

Após a compilação do próprio Unity, a simulação deve aparecer:
![alt text](<README images/image-10.png>)

Em um terminal abra o workspace e rode a conexão TCP:
```bash
cd ~/$nome_do_workspace$
source ~/$nome_do_workspace$/install/setup.bash
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0
```

Não se assuste caso algumas mensagens em vermelho apareçam, no fim a prova de que a conexão foi bem sucedida é um indicador no próprio unity, que outrora estava em vermelho e agora deve estar em azul.
![alt text](<README images/image-11.png>)

![alt text](<README images/image-13.png>)

Por fim, basta rodar o visualizador do Digital Twin em outro terminal:
```bash
cd ~/$nome_do_workspace$
source ~/$nome_do_workspace$/install/setup.bash
ros2 run dt_viz dt_visualizer_node
```

![alt text](<README images/image-14.png>)

---
## 📡 API ROS2 (Tópicos e Mensagens)

Para integrar módulos de navegação e controle ao Gêmeo Digital, utilize a seguinte especificação de tópicos:

### Subscribers (Tópicos Lidos pelo Sistema)

| Tópico | Tipo de Mensagem | Finalidade |
| :--- | :--- | :--- |
| ```/sensors/gps/fix``` | ```sensor_msgs/NavSatFix``` | Atualiza a posição (Lat/Lon) global do USV. |
| ```/sensors/imu/data``` | ```sensor_msgs/Imu``` | Atualiza a orientação (Quaternions convertidos para Euler). |
| ```/sensors/ais/report``` | ```dt_msgs/AisReport``` | Recebe a lista de alvos dinâmicos no raio de alcance. |
| ```/planned_route``` | ```nav_msgs/Path``` | Recebe a rota (waypoints) planejada pelo módulo de navegação. |


### Publishers (Tópicos Emitidos pelo Sistema)

| Tópico | Tipo de Mensagem | Finalidade |
| :--- | :--- | :--- |
| ```/collision_alert``` | ```dt_msgs/CollisionAlert``` | Emite alertas consolidados com tempo, distância (CPA) e índice de risco calculados para cada alvo. |


