/**
 * @file test_config_manager.cpp
 * @brief Unit tests for the ConfigManager class.
 * 
 * Validates the parsing of JSON configuration files, checking proper 
 * parameter extraction and system crash protection for missing files.
 */

#include <gtest/gtest.h>
#include <fstream>
#include <string>
#include <cstdio>
#include "map/config_manager.hpp"

/**
 * @class ConfigManagerTest
 * @brief Test suite fixture for the ConfigManager class.
 * 
 * Manages file I/O operations, creating a dummy JSON file before tests
 * and deleting it automatically afterward to ensure an isolated environment.
 */
class ConfigManagerTest : public ::testing::Test {
protected:
    std::string valid_file_path = "test_valid_config.json";     ///< Path to the dummy valid JSON file
    std::string invalid_file_path = "test_missing_config.json"; ///< Path to a non-existent JSON file

    /**
     * @brief Setup phase executed before each test runs.
     * 
     * Creates a temporary JSON file with valid syntax and parameters 
     * mapping to navigable areas and collision limits.
     */
    void SetUp() override {
        std::ofstream temp_file(valid_file_path);
        temp_file << R"({
            "margem_seguranca_metros": 15.5,
            "tolerancia_simplificacao": 0.5,
            "classes_navegaveis": ["DEPARE", "DRGARE"],
            "classes_colisao": ["LNDARE", "BOYISD"]
        })";
        temp_file.close();
    }

    /**
     * @brief Teardown phase executed after each test finishes.
     * 
     * Cleans up the temporary files from the disk to prevent side effects
     * on the operational system or other test suites.
     */
    void TearDown() override {
        std::remove(valid_file_path.c_str());
    }
};

/**
 * @brief Validates the successful parsing of a valid JSON configuration file.
 * 
 * Expects all numerical values and string arrays to map perfectly to the 
 * MapConfiguration struct. Uses EXPECT_EQ which performs deep character 
 * comparison for std::string types.
 */
TEST_F(ConfigManagerTest, LoadValidConfiguration) {
    MapConfiguration config = ConfigManager::load_configuration(valid_file_path);

    // Verify numerical values using DOUBLE_EQ for floating-point precision safety
    EXPECT_DOUBLE_EQ(config.safety_margin, 15.5);
    EXPECT_DOUBLE_EQ(config.simplification_tolerance, 0.5);

    // Verify navigable classes string vector
    ASSERT_EQ(config.navigable_classes.size(), 2);
    EXPECT_EQ(config.navigable_classes[0], "DEPARE");
    EXPECT_EQ(config.navigable_classes[1], "DRGARE");

    // Verify collision classes string vector
    ASSERT_EQ(config.collision_classes.size(), 2);
    EXPECT_EQ(config.collision_classes[0], "LNDARE");
    EXPECT_EQ(config.collision_classes[1], "BOYISD");
}

/**
 * @brief Validates the system's crash protection when the configuration file is missing.
 * 
 * Utilizes a Death Test to ensure the function aborts execution (via exit(1))
 * and prints the correct error pattern to the standard error output.
 */
TEST_F(ConfigManagerTest, HandlesMissingFileWithExit) {
    EXPECT_DEATH(ConfigManager::load_configuration(invalid_file_path), ".*Error:.*");
}