#include "../include/GerenciadorConfig.h"
#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

/**
 * @brief Carrega e decodifica os parâmetros operacionais e as classes de features geoespaciais a partir de um arquivo JSON.
 * @param caminhoArquivo Caminho físico do arquivo de configuração do sistema.
 * @return Estrutura ConfiguracaoMapa preenchida com as variáveis de segurança e filtros de camadas.
 */

ConfiguracaoMapa GerenciadorConfig::carregarConfiguracao(const std::string& caminhoArquivo) {
    ConfiguracaoMapa config;
    std::ifstream arquivo(caminhoArquivo);
    
    // Valida a inicialização e abertura do fluxo de leitura do arquivo
    if (!arquivo.is_open()) {
        std::cerr << "Erro: Nao foi possivel abrir o arquivo de configuracao: " << caminhoArquivo << std::endl;
        exit(1);
    }

    // Executa o parsing do fluxo de entrada para o objeto JSON da biblioteca
    json j;
    arquivo >> j;

    // Mapeamento direto dos parâmetros primitivos numéricos
    config.margemSeguranca = j["margem_seguranca_metros"];
    config.toleranciaSimplificacao = j["tolerancia_simplificacao"];
    
    // Extrai as strings identificadoras das classes de feições navegáveis
    for (const auto& item : j["classes_navegaveis"]) {
        config.classesNavegaveis.push_back(item);
    }
    
    // Extrai as strings identificadoras das classes de restrição e obstáculos
    for (const auto& item : j["classes_colisao"]) {
        config.classesColisao.push_back(item);
    }

    std::cout << "[Config] Arquivo carregado com sucesso." << std::endl;
    return config;
}