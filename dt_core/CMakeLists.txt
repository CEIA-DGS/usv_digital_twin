cmake_minimum_required(VERSION 3.14)
project(NavMesh_Generator)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# BUSCA DE PACOTES EXTERNOS
find_package(GDAL REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(glfw3 REQUIRED)
find_package(OpenGL REQUIRED)
find_package(Boost REQUIRED)

# DEPENDÊNCIAS VIA FETCHCONTENT (poly2tri)
include(FetchContent)
FetchContent_Declare(
    poly2tri
    GIT_REPOSITORY https://github.com/jhasse/poly2tri.git
    GIT_TAG        master
)
FetchContent_MakeAvailable(poly2tri)

# DIRETÓRIOS DE INCLUSÃO DE CABEÇALHOS
include_directories(include ${GDAL_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

# ARQUIVOS FONTE E CRIAÇÃO DO EXECUTÁVEL
file(GLOB SOURCES "src/*.cpp")
add_executable(navmesh_gen ${SOURCES})

# LINKAGEM DE BIBLIOTECAS
target_link_libraries(navmesh_gen PRIVATE 
    ${GDAL_LIBRARIES} 
    glfw 
    OpenGL::GL
    nlohmann_json::nlohmann_json
    poly2tri
)

# EMPACOTAMENTO DA GDAL
# Copia automaticamente os arquivos de dados S-57/GDAL para o diretorio de build,
# permitindo que o GdalInicializador encontre os arquivos de dados sem caminhos absolutos.
set(GDAL_SHARE_DIR "C:/msys64/ucrt64/share/gdal")

if(EXISTS "${GDAL_SHARE_DIR}")
    message(STATUS "[CMake] Empacotando arquivos de dados da GDAL para versao local...")
    file(COPY "${GDAL_SHARE_DIR}" DESTINATION "${CMAKE_BINARY_DIR}/gdal_data")
else()
    message(WARNING "[CMake] Pasta GDAL_SHARE_DIR nao encontrada em '${GDAL_SHARE_DIR}'. O deploy local pode exigir variáveis de ambiente configuradas no sistema.")
endif()