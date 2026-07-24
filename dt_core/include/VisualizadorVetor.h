#pragma once
#include <string>

/**
 * @brief Responsável pela renderização gráfica do NavMesh e das camadas espaciais processadas.
 * Utiliza o contexto OpenGL/GLFW para visualização interativa dos dados vetoriais.
 */

class VisualizadorVetor {
public:
    /**
     * @brief Inicia o loop de renderização e exibe as camadas geográficas contidas no diretório alvo.
     * @param pastaShapefiles Caminho para o diretório contendo os arquivos .shp processados.
     * @param nomeCarta Identificador da carta, utilizado para rotulagem da janela de exibição.
     */
    static void exibir(const std::string& pastaShapefiles, const std::string& nomeCarta);
};