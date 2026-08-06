#pragma once
#include <string>
#include <vector>

/**
 * @brief Armazena parâmetros operacionais e definições de classes extraídas do arquivo de configuração.
 */
struct MapConfiguration {
    double safety_margin;           
    double simplification_tolerance;   
    std::vector<std::string> navigable_classes; 
    std::vector<std::string> collision_classes;    
};

/**
 * @brief Gerencia a leitura, parsing e validação do arquivo de configuração do sistema (JSON).
 */
class ConfigManager {
public:
    /**
     * @brief Carrega os parâmetros de um arquivo JSON externo e popula a estrutura de configuração.
     * @param file_path Caminho físico do arquivo JSON de configuração.
     * @return Objeto MapConfiguration preenchido com os valores lidos.
     */
    static MapConfiguration load_configuration(const std::string& file_path);
};