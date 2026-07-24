#pragma once

#include <mutex>
#include <filesystem>
#include <iostream>
#include "gdal_priv.h"
#include "cpl_conv.h"

/**
 * @brief Gerenciador de inicialização estática da biblioteca GDAL.
 */
class GdalInicializador {
public:
    // Evita a instanciação acidental da classe
    GdalInicializador() = delete;

    /**
     * @brief Executa o registro global dos drivers da GDAL e configura variáveis de ambiente.
     */
    static void inicializar(){
        static std::once_flag flag_inicializacao;
        
        std::call_once(flag_inicializacao, [](){
            // Verifica se o SO ou o framework já configurou os caminhos nativamente
            if (std::getenv("GDAL_DATA") == nullptr) {
                
                // Procura a pasta que o script do CMake copia para o lado do executável
                std::filesystem::path caminho_portatil = std::filesystem::current_path() / "gdal_data" / "gdal";
                
                if (std::filesystem::exists(caminho_portatil)) {
                    CPLSetConfigOption("GDAL_DATA", caminho_portatil.string().c_str());
                    CPLSetConfigOption("S57_CSV", caminho_portatil.string().c_str());
                }
            }
            
            // Registra todos os drivers espaciais e formatos (S-57, Shapefile, etc.)
            GDALAllRegister();
        });
    }
};