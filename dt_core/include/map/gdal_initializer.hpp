#pragma once

#include <mutex>
#include <filesystem>
#include <iostream>
#include "gdal_priv.h"
#include "cpl_conv.h"

/**
 * @brief Gerenciador de inicialização estática da biblioteca GDAL.
 */
class GdalInitializer {
public:
    // Evita a instanciação acidental da classe
    GdalInitializer() = delete;

    /**
     * @brief Executa o registro global dos drivers da GDAL e configura variáveis de ambiente.
     */
    static void initialize() {
        static std::once_flag init_flag;
        
        std::call_once(init_flag, []() {
            // Verifica se o SO ou o framework já configurou os caminhos nativamente
            if (std::getenv("GDAL_DATA") == nullptr) {
                
                // Procura a pasta que o script do CMake copia para o lado do executável
                std::filesystem::path portable_path = std::filesystem::current_path() / "gdal_data" / "gdal";
                
                if (std::filesystem::exists(portable_path)) {
                    CPLSetConfigOption("GDAL_DATA", portable_path.string().c_str());
                    CPLSetConfigOption("S57_CSV", portable_path.string().c_str());
                }
            }
            
            // Registra todos os drivers espaciais e formatos (S-57, Shapefile, etc.)
            GDALAllRegister();
        });
    }
};