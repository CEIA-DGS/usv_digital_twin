#pragma once
#include <string>
#include <vector>

/**
 * @brief Armazena parâmetros operacionais e definições de classes extraídas do arquivo de configuração.
 */
struct ConfiguracaoMapa {
    double margemSeguranca;           
    double toleranciaSimplificacao;   
    std::vector<std::string> classesNavegaveis; 
    std::vector<std::string> classesColisao;    
};

/**
 * @brief Gerencia a leitura, parsing e validação do arquivo de configuração do sistema (JSON).
 */
class GerenciadorConfig {
public:
    /**
     * @brief Carrega os parâmetros de um arquivo JSON externo e popula a estrutura de configuração.
     * @param caminhoArquivo Caminho físico do arquivo JSON de configuração.
     * @return Objeto ConfiguracaoMapa preenchido com os valores lidos.
     */
    static ConfiguracaoMapa carregarConfiguracao(const std::string& caminhoArquivo);
};