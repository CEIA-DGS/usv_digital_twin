#include "../../include/map/config_manager.hpp"
#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

/**
 * @brief Carrega e decodifica os parâmetros operacionais e as classes de features geoespaciais a partir de um arquivo JSON.
 * @param file_path Caminho físico do arquivo de configuração do sistema.
 * @return Estrutura MapConfiguration preenchida com as variáveis de segurança e filtros de camadas.
 */
MapConfiguration ConfigManager::load_configuration(const std::string& file_path) {
    MapConfiguration config;
    std::ifstream file(file_path);
    
    // Valida a inicialização e abertura do fluxo de leitura do arquivo
    if (!file.is_open()) {
        std::cerr << "Erro: Nao foi possivel abrir o arquivo de configuracao: " << file_path << std::endl;
        exit(1);
    }

    // Executa o parsing do fluxo de entrada para o objeto JSON da biblioteca
    json j;
    file >> j;

    // Mapeamento direto dos parâmetros primitivos numéricos
    // Nota: As chaves do JSON ("margem_seguranca_metros") foram mantidas no original para compatibilidade com o arquivo físico.
    config.safety_margin = j["margem_seguranca_metros"];
    config.simplification_tolerance = j["tolerancia_simplificacao"];
    
    // Extrai as strings identificadoras das classes de feições navegáveis
    for (const auto& item : j["classes_navegaveis"]) {
        config.navigable_classes.push_back(item);
    }
    
    // Extrai as strings identificadoras das classes de restrição e obstáculos
    for (const auto& item : j["classes_colisao"]) {
        config.collision_classes.push_back(item);
    }

    std::cout << "[Config] Arquivo carregado com sucesso." << std::endl;
    return config;
}