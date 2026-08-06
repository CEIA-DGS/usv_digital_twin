#pragma once
#include <string>
#include <vector>

/**
 * @brief Stores operational parameters and class definitions extracted from the configuration file.
 */
struct MapConfiguration {
    double safety_margin;           
    double simplification_tolerance;   
    std::vector<std::string> navigable_classes; 
    std::vector<std::string> collision_classes;    
};

/**
 * @brief Manages the reading, parsing, and validation of the system configuration file (JSON).
 */
class ConfigManager {
public:
    /**
     * @brief Loads parameters from an external JSON file and populates the configuration structure.
     * @param file_path Physical path of the JSON configuration file.
     * @return MapConfiguration object populated with the read values.
     */
    static MapConfiguration load_configuration(const std::string& file_path);
};