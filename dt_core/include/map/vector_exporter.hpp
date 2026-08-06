#pragma once

#include <string>
#include "mesh_generator.hpp"
#include "gdal_priv.h"
#include "ogrsf_frmts.h"

/**
 * @brief Utilitário responsável por exportar as estruturas geradas da NavMesh para arquivos vetoriais (Shapefiles).
 * 
 * Atua como uma classe estática pura, encapsulando as operações de I/O em disco 
 * e conversão geométrica utilizando a biblioteca GDAL/OGR.
 */
class VectorExporter {
public:
    // Evita a instanciação acidental da classe
    VectorExporter() = delete;

    /**
     * @brief Exporta as geometrias (terra original, margem de segurança e malha de triângulos) para arquivos .shp.
     * @param nav_mesh Referência constante para a estrutura contendo os dados processados da malha de navegação.
     * @param output_dir Caminho absoluto ou relativo do diretório onde os Shapefiles serão gravados.
     * @param epsg_utm Código EPSG da projeção UTM para garantir o georreferenciamento correto dos arquivos gerados.
     */
    static void export_shapefile(const NavigationMesh& nav_mesh, const std::string& output_dir, int epsg_utm);

private:
    /**
     * @brief Função auxiliar para anexar de forma segura uma geometria genérica a uma camada OGR (Layer).
     * @param layer Ponteiro para a camada de destino (OGR Layer) previamente criada no dataset.
     * @param geom Ponteiro para a geometria base (Polígono, Linha, etc.) que será encapsulada em uma feature.
     */
    static void insert_geometry(OGRLayer* layer, OGRGeometry* geom);
};