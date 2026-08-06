#include "../../include/map/mesh_generator.hpp"
#include <iostream>
#include <cmath>
#include <poly2tri/poly2tri.h>

/**
 * @brief Extrai e limpa os vértices de um anel linear para compatibilidade com a biblioteca poly2tri.
 * Remove pontos duplicados dentro de uma tolerância espacial e elimina a auto-inclusão do ponto de fechamento.
 * @param ring Ponteiro para o anel linear original da GDAL/OGR.
 * @return Vetor de ponteiros de pontos alocados dinamicamente para o poly2tri.
 */
std::vector<p2t::Point*> extract_clean_contour(OGRLinearRing* ring) {
    std::vector<p2t::Point*> contour;
    if (!ring || ring->getNumPoints() < 4) return contour;
    
    double epsilon = 0.1; // Tolerância para eliminação de vértices redundantes/colineares
    for (int i = 0; i < ring->getNumPoints() - 1; i++) {
        double x = ring->getX(i);
        double y = ring->getY(i);
        if (!contour.empty()) {
            if (std::abs(x - contour.back()->x) < epsilon && std::abs(y - contour.back()->y) < epsilon) continue;
        }
        contour.push_back(new p2t::Point(x, y));
    }
    
    // Garante que o contorno não replique o ponto inicial no final
    if (contour.size() >= 3) {
        if (std::abs(contour.front()->x - contour.back()->x) < epsilon && 
            std::abs(contour.front()->y - contour.back()->y) < epsilon) {
            delete contour.back();
            contour.pop_back();
        }
    }
    return contour;
}

/**
 * @brief Normaliza qualquer primitiva geométrica para o tipo poligonal.
 * Aplica uma operação de buffer inflatório mínimo para converter linhas e pontos em polígonos válidos.
 */
static OGRGeometry* ensure_polygon(OGRGeometry* geom) {
    if (!geom) return nullptr;
    OGRwkbGeometryType type = wkbFlatten(geom->getGeometryType());
    
    if (type == wkbPolygon || type == wkbMultiPolygon) {
        return geom->clone();
    } else {
        return geom->Buffer(1.0); // Buffer estático para blindagem contra erros de tipo de feição
    }
}

/**
 * @brief Filtra e descarta componentes poligonais cuja área seja inferior ao limiar especificado.
 * Utilizado para eliminação de slivers e artefatos geométricos gerados por operações booleanas.
 */
OGRGeometry* MeshGenerator::filter_by_area(OGRGeometry* geom, double min_area) {
    if (!geom) return nullptr;
    OGRwkbGeometryType type = wkbFlatten(geom->getGeometryType());

    if (type == wkbPolygon) {
        OGRPolygon* poly = geom->toPolygon();
        if (poly->get_Area() >= min_area) return poly->clone();
        return nullptr;
    } 
    else if (type == wkbMultiPolygon || type == wkbGeometryCollection) {
        OGRGeometryCollection* col = (OGRGeometryCollection*)geom;
        OGRMultiPolygon* new_multi = (OGRMultiPolygon*)OGRGeometryFactory::createGeometry(wkbMultiPolygon);
        for (auto&& sub : *col) {
            if (wkbFlatten(sub->getGeometryType()) == wkbPolygon) {
                OGRPolygon* poly = sub->toPolygon();
                if (poly->get_Area() >= min_area) {
                    new_multi->addGeometry(poly->clone());
                }
            }
        }
        if (new_multi->getNumGeometries() > 0) return new_multi;
        OGRGeometryFactory::destroyGeometry(new_multi);
        return nullptr;
    }
    return geom->clone();
}

/**
 * @brief Pipeline principal para a geração do NavMesh vetorial e cálculo de restrições espaciais.
 * Executa normalização, expansão de buffers protetores, diferenças booleanas e triangulação restrita.
 */
NavigationMesh MeshGenerator::generate(const ProcessedGeometries& geometries, double margin_meters, double simplification_tolerance) {
    NavigationMesh nav_mesh;

    std::cout << "[MeshGenerator] Processando e convertendo obstaculos em poligonos..." << std::endl;
    std::vector<OGRGeometry*> polygon_obstacles;
    for (auto* geom : geometries.obstacles) {
        OGRGeometry* valid_poly = ensure_polygon(geom);
        if (valid_poly) {
            polygon_obstacles.push_back(valid_poly);
        }
    }

    std::cout << "[MeshGenerator] Unindo poligonos de agua e terra..." << std::endl;
    OGRGeometry* mega_ocean = union_geometries(geometries.navigable_area);
    OGRGeometry* original_land = union_geometries(polygon_obstacles);

    for (auto* p : polygon_obstacles) OGRGeometryFactory::destroyGeometry(p);

    nav_mesh.original_land = original_land ? original_land->clone() : nullptr;

    std::cout << "[MeshGenerator] Calculando margens de seguranca fisicas (Buffer Duplo)..." << std::endl;
    OGRGeometry* real_expanded_land = nullptr;
    OGRGeometry* navmesh_expanded_land = nullptr;

    if (original_land && margin_meters > 0.0) {
        real_expanded_land = original_land->Buffer(margin_meters);
        nav_mesh.safety_margin = real_expanded_land->Difference(original_land);
        navmesh_expanded_land = original_land->Buffer(margin_meters + simplification_tolerance);
    } else if (original_land) {
        real_expanded_land = original_land->clone();
        navmesh_expanded_land = original_land->clone();
    }

    std::cout << "[MeshGenerator] Realizando Diferenca Booleana (Agua - Obstaculos)..." << std::endl;
    OGRGeometry* raw_safe_area = nullptr;
    if (mega_ocean && navmesh_expanded_land) {
        raw_safe_area = mega_ocean->Difference(navmesh_expanded_land);
    } else if (mega_ocean) {
        raw_safe_area = mega_ocean->clone();
    }

    std::cout << "[MeshGenerator] Filtrando micro-ilhas/pocas (Slivers < 25m2)..." << std::endl;
    OGRGeometry* filtered_area = filter_by_area(raw_safe_area, 25.0);

    std::cout << "[MeshGenerator] Simplificando contornos do NavMesh..." << std::endl;
    if (filtered_area && simplification_tolerance > 0.0) {
        // Reduz a densidade de vértices preservando a topologia.
        // A invasão máxima será = simplification_tolerance, parando exatamente na Margem Real.
        nav_mesh.safe_navigable_perimeter = filtered_area->SimplifyPreserveTopology(simplification_tolerance);
    } else if (filtered_area) {
        nav_mesh.safe_navigable_perimeter = filtered_area->clone();
    }

    // Liberação de memória das geometrias intermediárias do pipeline
    if (mega_ocean) OGRGeometryFactory::destroyGeometry(mega_ocean);
    if (original_land) OGRGeometryFactory::destroyGeometry(original_land);
    if (real_expanded_land) OGRGeometryFactory::destroyGeometry(real_expanded_land);
    if (navmesh_expanded_land) OGRGeometryFactory::destroyGeometry(navmesh_expanded_land);
    if (raw_safe_area) OGRGeometryFactory::destroyGeometry(raw_safe_area);
    if (filtered_area) OGRGeometryFactory::destroyGeometry(filtered_area);

    if (!nav_mesh.safe_navigable_perimeter) return nav_mesh;

    std::cout << "[MeshGenerator] Iniciando Triangulacao (CDT)..." << std::endl;
    int fail_counter = 0;

    // Função interna lambda para processamento de geometrias simples e complexas (MultiPolygons)
    auto process_geometry = [&](OGRGeometry* geom) {
        OGRwkbGeometryType type = wkbFlatten(geom->getGeometryType());
        if (type == wkbPolygon) {
            triangulate_polygon(geom->toPolygon(), nav_mesh.triangles, fail_counter);
        } 
        else if (type == wkbMultiPolygon || type == wkbGeometryCollection) {
            OGRGeometryCollection* collection = (OGRGeometryCollection*)geom;
            for (auto&& sub_geom : *collection) {
                if (wkbFlatten(sub_geom->getGeometryType()) == wkbPolygon) {
                    triangulate_polygon(sub_geom->toPolygon(), nav_mesh.triangles, fail_counter);
                }
            }
        }
    };

    process_geometry(nav_mesh.safe_navigable_perimeter);

    if (fail_counter > 0) {
        std::cout << "[MeshGenerator] AVISO: " << fail_counter << " sub-poligonos problematicos ignorados." << std::endl;
    }
    std::cout << "[MeshGenerator] Malha concluida! Triangulos gerados: " << nav_mesh.triangles.size() << std::endl;
    return nav_mesh;
}

/**
 * @brief Consolida um vetor de primitivas geométricas em um único objeto unificado via operações de união.
 */
OGRGeometry* MeshGenerator::union_geometries(const std::vector<OGRGeometry*>& geometry_list) {
    if (geometry_list.empty()) return nullptr;
    OGRGeometry* geometric_union = geometry_list[0]->clone();
    for (size_t i = 1; i < geometry_list.size(); i++) {
        OGRGeometry* temp = geometric_union->Union(geometry_list[i]);
        if (temp) {
            OGRGeometryFactory::destroyGeometry(geometric_union);
            geometric_union = temp;
        }
    }
    return geometric_union;
}

/**
 * @brief Executa a Triangulação de Delaunay Restrita (CDT) em um polígono individual contendo furos internos.
 * @param polygon Ponteiro do polígono Alvo da GDAL.
 * @param target_list Vetor de saída onde as estruturas de Triângulos geradas serão anexadas.
 * @param fail_counter Referência para incremento em caso de exceções no motor geométrico da biblioteca poly2tri.
 */
void MeshGenerator::triangulate_polygon(OGRPolygon* polygon, std::vector<Triangle>& target_list, int& fail_counter) {
    if (!polygon) return;
    
    // Força correção de fechamento topológico e auto-interseções com buffer zero
    OGRPolygon* clean_poly = (OGRPolygon*)polygon->Buffer(0.0);
    if (!clean_poly) clean_poly = polygon;
    
    OGRLinearRing* ext_ring = clean_poly->getExteriorRing();
    std::vector<p2t::Point*> ext_contour = extract_clean_contour(ext_ring);

    if (ext_contour.size() < 3) {
        for (auto* p : ext_contour) delete p;
        if (clean_poly != polygon) OGRGeometryFactory::destroyGeometry(clean_poly);
        return;
    }

    p2t::CDT* cdt = nullptr;
    std::vector<std::vector<p2t::Point*>> holes_list;

    try {
        // Inicializa o motor estrutural CDT passando a fronteira externa
        cdt = new p2t::CDT(ext_contour);
        
        // Injeta os anéis internos (ilhas/restrições) como furos geométricos no motor
        int num_holes = clean_poly->getNumInteriorRings();
        for (int b = 0; b < num_holes; b++) {
            std::vector<p2t::Point*> p2t_hole = extract_clean_contour(clean_poly->getInteriorRing(b));
            if (p2t_hole.size() >= 3) {
                cdt->AddHole(p2t_hole);
                holes_list.push_back(p2t_hole);
            } else {
                for (auto* p : p2t_hole) delete p;
            }
        }
        
        // Processa as restrições e gera a malha triangular
        cdt->Triangulate();
        
        // Mapeia os triângulos gerados pelo poly2tri para a estrutura interna nativa do sistema
        std::vector<p2t::Triangle*> tris = cdt->GetTriangles();
        for (auto* t : tris) {
            target_list.push_back({{t->GetPoint(0)->x, t->GetPoint(0)->y}, {t->GetPoint(1)->x, t->GetPoint(1)->y}, {t->GetPoint(2)->x, t->GetPoint(2)->y}});
        }
    } catch (...) { 
        fail_counter++; 
    }

    // Liberação de memória local do escopo de triangulação
    if (cdt) delete cdt;
    for (auto* p : ext_contour) delete p;
    for (auto& hole : holes_list) { for (auto* p : hole) delete p; }
    if (clean_poly != polygon) OGRGeometryFactory::destroyGeometry(clean_poly);
}