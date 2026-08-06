#pragma once
#include <string>
#include <vector>
#include "gdal_priv.h"
#include "ogrsf_frmts.h"
#include "ogr_spatialref.h"
#include "config_manager.hpp"

/**
 * @brief Contêiner para armazenamento das geometrias processadas extraídas das cartas S-57.
 * Encapsula a lógica de gerenciamento de memória das estruturas OGRGeometry.
 */
struct ProcessedGeometries {
    std::vector<OGRGeometry*> navigable_area;
    std::vector<OGRGeometry*> obstacles;
    int dynamic_utm_epsg;

    /**
     * @brief Executa a desalocação forçada de cada geometria OGR e limpa os vetores.
     */
    void free_memory() {
        for (auto geom : navigable_area) OGRGeometryFactory::destroyGeometry(geom);
        for (auto geom : obstacles) OGRGeometryFactory::destroyGeometry(geom);
        navigable_area.clear();
        obstacles.clear();
    }
};

/**
 * @brief Processador especializado na ingestão, extração e tratamento de dados geoespaciais S-57.
 */
class S57Processor {
public:
    /**
     * @brief Abre uma carta náutica S-57, filtra camadas via configuração e reprojeta coordenadas para EPSG:3857.
     * @param s57_path Caminho físico da carta (.000).
     * @param config Configuração contendo as classes de camadas a serem extraídas.
     * @return Objeto ProcessedGeometries contendo os dados reprojetados prontos para uso.
     */
    static ProcessedGeometries process_chart(const std::string& s57_path, const MapConfiguration& config);
};