#include "../../include/map/config_manager.hpp"
#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

MapConfiguration ConfigManager::load_configuration(const std::string& file_path) {
    MapConfiguration config;
    std::ifstream file(file_path);
    
    // Validates the initialization and opening of the file reading stream
    if (!file.is_open()) {
        std::cerr << "Error: Could not open the configuration file: " << file_path << std::endl;
        exit(1);
    }

    // Executes the parsing of the input stream into the library's JSON object
    json j;
    file >> j;

    // Direct mapping of primitive numerical parameters
    // Note: JSON keys (e.g., "margem_seguranca_metros") were kept in the original Portuguese for compatibility with the physical file.
    config.safety_margin = j["margem_seguranca_metros"];
    config.simplification_tolerance = j["tolerancia_simplificacao"];
    
    // Extracts the identifying strings of the navigable feature classes
    for (const auto& item : j["classes_navegaveis"]) {
        config.navigable_classes.push_back(item);
    }
    
    // Extracts the identifying strings of the restriction and obstacle classes
    for (const auto& item : j["classes_colisao"]) {
        config.collision_classes.push_back(item);
    }

    std::cout << "[Config] File loaded successfully." << std::endl;
    return config;
}