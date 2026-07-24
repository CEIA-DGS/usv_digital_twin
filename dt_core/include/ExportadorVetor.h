#pragma once

#include <string>
#include "GeradorMalha.h"
#include "gdal_priv.h"
#include "ogrsf_frmts.h"

/**
 * @brief Utilitário responsável por exportar as estruturas geradas da NavMesh para arquivos vetoriais (Shapefiles).
 * 
 * Atua como uma classe estática pura, encapsulando as operações de I/O em disco 
 * e conversão geométrica utilizando a biblioteca GDAL/OGR.
 */
class ExportadorVetor {
public:
    // Evita a instanciação acidental da classe
    ExportadorVetor() = delete;

    /**
     * @brief Exporta as geometrias (terra original, margem de segurança e malha de triângulos) para arquivos .shp.
     * @param navMesh Referência constante para a estrutura contendo os dados processados da malha de navegação.
     * @param diretorioSaida Caminho absoluto ou relativo do diretório onde os Shapefiles serão gravados.
     * @param epsg_utm Código EPSG da projeção UTM para garantir o georreferenciamento correto dos arquivos gerados.
     */
    static void exportarShapefile(const MalhaNavegacao& navMesh, const std::string& diretorioSaida, int epsg_utm);

private:
    /**
     * @brief Função auxiliar para anexar de forma segura uma geometria genérica a uma camada OGR (Layer).
     * @param layer Ponteiro para a camada de destino (OGR Layer) previamente criada no dataset.
     * @param geom Ponteiro para a geometria base (Polígono, Linha, etc.) que será encapsulada em uma feature.
     */
    static void inserirGeometria(OGRLayer* layer, OGRGeometry* geom);
};