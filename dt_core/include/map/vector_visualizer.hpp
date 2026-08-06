#pragma once
#include <string>

/**
 * @brief Responsável pela renderização gráfica do NavMesh e das camadas espaciais processadas.
 * Utiliza o contexto OpenGL/GLFW para visualização interativa dos dados vetoriais.
 */
class VectorVisualizer {
public:
    /**
     * @brief Inicia o loop de renderização e exibe as camadas geográficas contidas no diretório alvo.
     * @param shapefiles_folder Caminho para o diretório contendo os arquivos .shp processados.
     * @param chart_name Identificador da carta, utilizado para rotulagem da janela de exibição.
     */
    static void display(const std::string& shapefiles_folder, const std::string& chart_name);
};