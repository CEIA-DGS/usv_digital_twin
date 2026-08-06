#pragma once

#include <mutex>
#include <filesystem>
#include <iostream>
#include "gdal_priv.h"
#include "cpl_conv.h"

/**
 * @brief Static initialization manager for the GDAL library.
 */
class GdalInitializer {
public:
    // Prevents accidental instantiation of the class
    GdalInitializer() = delete;

    /**
     * @brief Executes the global registration of GDAL drivers and configures environment variables.
     */
    static void initialize() {
        static std::once_flag init_flag;
        
        std::call_once(init_flag, []() {
            // Checks if the OS or framework has already natively configured the paths
            if (std::getenv("GDAL_DATA") == nullptr) {
                
                // Looks for the folder that the CMake script copies next to the executable
                std::filesystem::path portable_path = std::filesystem::current_path() / "gdal_data" / "gdal";
                
                if (std::filesystem::exists(portable_path)) {
                    CPLSetConfigOption("GDAL_DATA", portable_path.string().c_str());
                    CPLSetConfigOption("S57_CSV", portable_path.string().c_str());
                }
            }
            
            // Registers all spatial drivers and formats (S-57, Shapefile, etc.)
            GDALAllRegister();
        });
    }
};