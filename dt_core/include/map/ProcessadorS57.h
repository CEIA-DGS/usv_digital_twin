#pragma once
#include <string>
#include <vector>
#include "gdal_priv.h"
#include "ogrsf_frmts.h"
#include "ogr_spatialref.h"
#include "GerenciadorConfig.h"

/**
 * @brief Contêiner para armazenamento das geometrias processadas extraídas das cartas S-57.
 * Encapsula a lógica de gerenciamento de memória das estruturas OGRGeometry.
 */

struct GeometriasProcessadas {
    std::vector<OGRGeometry*> areaNavegavel;
    std::vector<OGRGeometry*> obstaculos;
    int epsg_utm;

    /**
     * @brief Executa a desalocação forçada de cada geometria OGR e limpa os vetores.
     */
    void liberarMemoria() {
        for (auto geom : areaNavegavel) OGRGeometryFactory::destroyGeometry(geom);
        for (auto geom : obstaculos) OGRGeometryFactory::destroyGeometry(geom);
        areaNavegavel.clear();
        obstaculos.clear();
    }
};

/**
 * @brief Processador especializado na ingestão, extração e tratamento de dados geoespaciais S-57.
 */

class ProcessadorS57 {
public:
    /**
     * @brief Abre uma carta náutica S-57, filtra camadas via configuração e reprojeta coordenadas para EPSG:3857.
     * @param caminhoS57 Caminho físico da carta (.000).
     * @param config Configuração contendo as classes de camadas a serem extraídas.
     * @return Objeto GeometriasProcessadas contendo os dados reprojetados prontos para uso.
     */
    static GeometriasProcessadas processarCarta(const std::string& caminhoS57, const ConfiguracaoMapa& config);
};