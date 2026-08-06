#pragma once
#include <vector>
#include "gdal_priv.h"
#include "s57_processor.hpp"

/**
 * @brief Primitivas geométricas para representação da malha de navegação.
 */
struct Point2D { double x, y; };
struct Triangle { Point2D p1, p2, p3; };

/**
 * @brief Estrutura de dados contendo a malha triangular processada e geometrias auxiliares.
 */
struct NavigationMesh {
    std::vector<Triangle> triangles;
    OGRGeometry* original_land = nullptr;
    OGRGeometry* safety_margin = nullptr;
    OGRGeometry* safe_navigable_perimeter = nullptr;

    /**
     * @brief Desaloca a memória ocupada pelas geometrias OGR mantidas em cache.
     */
    void free_memory() {
        if (original_land) OGRGeometryFactory::destroyGeometry(original_land);
        if (safety_margin) OGRGeometryFactory::destroyGeometry(safety_margin);
        if (safe_navigable_perimeter) OGRGeometryFactory::destroyGeometry(safe_navigable_perimeter);
    }
};

/**
 * @brief Motor geométrico para processamento e geração da malha de navegação.
 * 
 * Atua como uma classe utilitária estática. Executa operações booleanas, 
 * filtragem espacial e a Triangulação de Delaunay Restrita (CDT).
 */
class MeshGenerator {
public:
    // Evita a instanciação acidental da classe
    MeshGenerator() = delete;

    /**
     * @brief Pipeline completo de geração da malha a partir de dados S-57 processados.
     * @param geometries Estrutura contendo os polígonos base de água e terra.
     * @param margin_meters Distância da margem de segurança a ser expandida (Buffer).
     * @param simplification_tolerance Erro máximo permitido (em metros) na redução de vértices (Douglas-Peucker).
     * @return NavigationMesh contendo os triângulos gerados e polígonos auxiliares.
     */
    static NavigationMesh generate(const ProcessedGeometries& geometries, double margin_meters, double simplification_tolerance);

private:
    /**
     * @brief Consolida um conjunto de geometrias fragmentadas em um único objeto unificado.
     * @param geometry_list Vetor de ponteiros para as geometrias a serem unidas.
     * @return Ponteiro OGRGeometry para a geometria fundida resultante.
     */
    static OGRGeometry* union_geometries(const std::vector<OGRGeometry*>& geometry_list);
    
    /**
     * @brief Executa a triangulação CDT (Constrained Delaunay) em polígonos com furos.
     * @param polygon Ponteiro do polígono base a ser triangulado.
     * @param target_list Vetor de saída onde as estruturas de Triângulos geradas serão anexadas.
     * @param fail_counter Referência para contagem de falhas do motor geométrico poly2tri.
     */
    static void triangulate_polygon(OGRPolygon* polygon, std::vector<Triangle>& target_list, int& fail_counter);
    
    /**
     * @brief Filtra features poligonais baseadas em um limiar mínimo de área.
     * @param geom Ponteiro para a geometria de entrada original.
     * @param min_area Área mínima exigida em metros quadrados para que a feature seja mantida.
     * @return Ponteiro OGRGeometry para a nova geometria filtrada.
     */
    static OGRGeometry* filter_by_area(OGRGeometry* geom, double min_area);
};