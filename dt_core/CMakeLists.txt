cmake_minimum_required(VERSION 3.14)
project(dt_core)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# ==============================================================================
# 1. DEPENDÊNCIAS DO SISTEMA
# Requer: sudo apt install libgdal-dev libglfw3-dev libgl1-mesa-dev libboost-all-dev
# ==============================================================================
find_package(ament_cmake REQUIRED)
find_package(GDAL REQUIRED)
find_package(glfw3 REQUIRED)
find_package(OpenGL REQUIRED)
find_package(Boost REQUIRED)
find_package(ament_index_cpp REQUIRED)

# ==============================================================================
# 2. DEPENDÊNCIAS FETCHCONTENT
# ==============================================================================
include(FetchContent)

FetchContent_Declare(
    poly2tri
    GIT_REPOSITORY https://github.com/jhasse/poly2tri.git
    GIT_TAG        master
)
FetchContent_MakeAvailable(poly2tri)

FetchContent_Declare(
    nlohmann_json
    GIT_REPOSITORY https://github.com/nlohmann/json.git
    GIT_TAG        v3.11.3
)
FetchContent_MakeAvailable(nlohmann_json)

# ==============================================================================
# 3. CONSTRUÇÃO DA BIBLIOTECA CORE
# ==============================================================================
file(GLOB_RECURSE CORE_SOURCES "src/*.cpp")
list(REMOVE_ITEM CORE_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/src/map/main.cpp")

add_library(${PROJECT_NAME} SHARED ${CORE_SOURCES})

target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${GDAL_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME} 
    PUBLIC
        ${GDAL_LIBRARIES}
        ament_index_cpp::ament_index_cpp
    PRIVATE
        glfw
        OpenGL::GL
        nlohmann_json::nlohmann_json
        poly2tri
)


# ==============================================================================
# 4. CONSTRUÇÃO DO EXECUTÁVEL STANDALONE
# ==============================================================================
add_executable(navmesh_gen src/map/main.cpp)
target_link_libraries(navmesh_gen PRIVATE ${PROJECT_NAME})

# ==============================================================================
# 5. REGRAS DE INSTALAÇÃO
# ==============================================================================
include(GNUInstallDirs)

install(TARGETS ${PROJECT_NAME} navmesh_gen
    EXPORT ${PROJECT_NAME}Targets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(DIRECTORY include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
    FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)

install(DIRECTORY data
    DESTINATION share/${PROJECT_NAME}
)

install(EXPORT ${PROJECT_NAME}Targets
    FILE ${PROJECT_NAME}Targets.cmake
    NAMESPACE ${PROJECT_NAME}::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
)

include(CMakePackageConfigHelpers)
write_basic_package_version_file(
    ${PROJECT_NAME}ConfigVersion.cmake
    VERSION 1.0.0
    COMPATIBILITY SameMajorVersion
)
file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
    "include(\"\${CMAKE_CURRENT_LIST_DIR}/${PROJECT_NAME}Targets.cmake\")\n"
)
install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
)

ament_package()