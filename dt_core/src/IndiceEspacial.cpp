#include "IndiceEspacial.hpp"
#include <iostream>
#include <filesystem>
#include "gdal_priv.h"
#include "ogrsf_frmts.h"
#include "GdalInicializador.hpp"

/**
 * @brief Função auxiliar interna para injetar vértices de um anel linear na R-Tree de distâncias.
 * @param anel Ponteiro para o anel linear (OGRLinearRing) contendo os vértices da geometria.
 * @param rtree Referência para a R-Tree de segmentos onde as caixas delimitadoras serão inseridas.
 */
void injetarAnelNaRTree(OGRLinearRing* anel, bgi::rtree<ValorRTreeSegmento, bgi::quadratic<16>>& rtree) {
    if (!anel) return;
    for (int i = 0; i < anel->getNumPoints() - 1; i++) {
        BgPonto p1(anel->getX(i), anel->getY(i));
        BgPonto p2(anel->getX(i+1), anel->getY(i+1));
        BgSegmento segmento(p1, p2);
        
        BgCaixaReal caixa;
        bg::envelope(segmento, caixa); 
        rtree.insert(std::make_pair(caixa, segmento));
    }
}

// CARREGAMENTO DOS DADOS (MARGEM E NAVMESH)

bool IndiceEspacial::carregarShapefiles(const std::string& shpMargem, const std::string& shpMalha) {
    rtree_margem_.clear();
    rtree_malha_.clear();
    triangulos_malha_.clear();

    // CARREGA MARGEM DE SEGURANÇA
    GDALDataset* ds_margem = (GDALDataset*)GDALOpenEx(shpMargem.c_str(), GDAL_OF_VECTOR, nullptr, nullptr, nullptr);
    if (ds_margem) {
        OGRLayer* layer_margem = ds_margem->GetLayer(0);
        if (layer_margem) {
            OGRFeature* feicao;
            layer_margem->ResetReading();
            while ((feicao = layer_margem->GetNextFeature()) != nullptr) {
                OGRGeometry* geom = feicao->GetGeometryRef();
                if (geom) {
                    OGRwkbGeometryType tipo = wkbFlatten(geom->getGeometryType());
                    if (tipo == wkbPolygon) {
                        OGRPolygon* poly = (OGRPolygon*)geom;
                        injetarAnelNaRTree(poly->getExteriorRing(), rtree_margem_);
                    } else if (tipo == wkbMultiPolygon) {
                        OGRMultiPolygon* mpoly = (OGRMultiPolygon*)geom;
                        for (int i = 0; i < mpoly->getNumGeometries(); i++) {
                            OGRGeometry* subGeom = mpoly->getGeometryRef(i);
                            if (subGeom && wkbFlatten(subGeom->getGeometryType()) == wkbPolygon) {
                                injetarAnelNaRTree(((OGRPolygon*)subGeom)->getExteriorRing(), rtree_margem_);
                            }
                        }
                    }
                }
                OGRFeature::DestroyFeature(feicao);
            }
        }
        GDALClose(ds_margem);
    } else {
        std::cerr << "[IndiceEspacial] Erro ao abrir Margem: " << shpMargem << "\n";
    }

    // CARREGA MALHA DE NAVEGAÇÃO
    GDALDataset* ds_malha = (GDALDataset*)GDALOpenEx(shpMalha.c_str(), GDAL_OF_VECTOR, nullptr, nullptr, nullptr);
    if (ds_malha) {
        OGRLayer* layer_malha = ds_malha->GetLayer(0);
        if (layer_malha) {
            OGRFeature* feicao;
            layer_malha->ResetReading();
            while ((feicao = layer_malha->GetNextFeature()) != nullptr) {
                OGRGeometry* geom = feicao->GetGeometryRef();
                if (geom && wkbFlatten(geom->getGeometryType()) == wkbPolygon) {
                    OGRPolygon* poly = (OGRPolygon*)geom;
                    OGRLinearRing* anel_exterior = poly->getExteriorRing();
                    
                    if (anel_exterior) {
                        BgPoligono triangulo_boost;
                        for (int i = 0; i < anel_exterior->getNumPoints(); i++) {
                            bg::append(triangulo_boost.outer(), BgPonto(anel_exterior->getX(i), anel_exterior->getY(i)));
                        }
                        bg::correct(triangulo_boost);
                        
                        size_t indice = triangulos_malha_.size();
                        triangulos_malha_.push_back(triangulo_boost);

                        BgCaixaReal caixa_triangulo;
                        bg::envelope(triangulo_boost, caixa_triangulo);
                        rtree_malha_.insert(std::make_pair(caixa_triangulo, indice));
                    }
                }
                OGRFeature::DestroyFeature(feicao);
            }
        }
        GDALClose(ds_malha);
    } else {
        std::cerr << "[IndiceEspacial] Erro ao abrir Malha: " << shpMalha << "\n";
    }

    std::cout << "[IndiceEspacial] R-Trees Prontas! Segmentos da Margem: " << rtree_margem_.size() 
              << " | Triangulos da NavMesh: " << rtree_malha_.size() << "\n";
    return true;
}

// FUNÇÕES DE CÁLCULO MATEMÁTICO RÁPIDO

double IndiceEspacial::calcularDistanciaMargem(const types::Point& posicao) const {
    if (rtree_margem_.empty()) return -1.0; 

    BgPonto ponto_busca(posicao.get_x(), posicao.get_y());
    std::vector<ValorRTreeSegmento> resultados;
    
    // Consulta as 50 caixas mais próximas para garantir a correta
    rtree_margem_.query(bgi::nearest(ponto_busca, 50), std::back_inserter(resultados));
    
    if (resultados.empty()) return -1.0;

    // Realiza a medição exata ponto-a-segmento para os 50 candidatos
    double distancia_minima_exata = std::numeric_limits<double>::max();
    for (const auto& res : resultados) {
        double dist = bg::distance(ponto_busca, res.second);
        if (dist < distancia_minima_exata) {
            distancia_minima_exata = dist;
        }
    }
    
    return distancia_minima_exata;
}

bool IndiceEspacial::isNavegavel(const types::Point& posicao) const {
    if (rtree_malha_.empty()) return false;

    BgPonto ponto_busca(posicao.get_x(), posicao.get_y());
    std::vector<ValorRTreePoligono> candidatos;
    
    // Consulta a R-Tree de triângulos interceptados geometricamente
    rtree_malha_.query(bgi::intersects(ponto_busca), std::back_inserter(candidatos));
    
    for (const auto& candidato : candidatos) {
        size_t indice = candidato.second; 
        if (bg::within(ponto_busca, triangulos_malha_[indice])) {
            return true; // Ponto está estritamente dentro de um triângulo navegável seguro
        }
    }
    return false; // Fora da malha de navegação
}

// DEBUG VISUAL DAS R-TREES E LINHAS DE DISTÂNCIA PARA OS PONTOS NAVEGÁVEIS

void IndiceEspacial::exportarDebugRTree(const std::vector<types::Point>& pontos_teste, const std::string& pastaSaida, int epsg_utm) const {
    GDALDriver* driver = GetGDALDriverManager()->GetDriverByName("ESRI Shapefile");
    OGRSpatialReference srs; 
    srs.importFromEPSG(epsg_utm);

    // R-TREE DA MARGEM DE SEGURANÇA
    if (!rtree_margem_.empty()) {
        std::string pathMargem = pastaSaida + "/98_RTree_Caixas_Margem.shp";
        if (std::filesystem::exists(pathMargem)) driver->Delete(pathMargem.c_str());
        GDALDataset* dsMargem = driver->Create(pathMargem.c_str(), 0, 0, 0, GDT_Unknown, nullptr);
        OGRLayer* layerMargem = dsMargem->CreateLayer("caixas_margem", &srs, wkbPolygon, nullptr);

        for (auto it = rtree_margem_.begin(); it != rtree_margem_.end(); ++it) {
            BgCaixaReal caixa = it->first;
            double min_x = caixa.min_corner().get<0>(), min_y = caixa.min_corner().get<1>();
            double max_x = caixa.max_corner().get<0>(), max_y = caixa.max_corner().get<1>();

            OGRPolygon poly; OGRLinearRing ring;
            ring.addPoint(min_x, min_y); ring.addPoint(max_x, min_y);
            ring.addPoint(max_x, max_y); ring.addPoint(min_x, max_y); ring.addPoint(min_x, min_y);
            poly.addRing(&ring);

            OGRFeature* feat = OGRFeature::CreateFeature(layerMargem->GetLayerDefn());
            feat->SetGeometry(&poly);
            if (layerMargem->CreateFeature(feat) != OGRERR_NONE) {}
            OGRFeature::DestroyFeature(feat);
        }
        layerMargem->SyncToDisk();
        GDALClose(dsMargem);
    }

    // R-TREE DA MALHA DE NAVEGAÇÃO
    if (!rtree_malha_.empty()) {
        std::string pathMalha = pastaSaida + "/97_RTree_Caixas_Malha.shp";
        if (std::filesystem::exists(pathMalha)) driver->Delete(pathMalha.c_str());
        GDALDataset* dsMalha = driver->Create(pathMalha.c_str(), 0, 0, 0, GDT_Unknown, nullptr);
        OGRLayer* layerMalha = dsMalha->CreateLayer("caixas_malha", &srs, wkbPolygon, nullptr);

        for (auto it = rtree_malha_.begin(); it != rtree_malha_.end(); ++it) {
            BgCaixaReal caixa = it->first;
            double min_x = caixa.min_corner().get<0>(), min_y = caixa.min_corner().get<1>();
            double max_x = caixa.max_corner().get<0>(), max_y = caixa.max_corner().get<1>();

            OGRPolygon poly; OGRLinearRing ring;
            ring.addPoint(min_x, min_y); ring.addPoint(max_x, min_y);
            ring.addPoint(max_x, max_y); ring.addPoint(min_x, max_y); ring.addPoint(min_x, min_y);
            poly.addRing(&ring);

            OGRFeature* feat = OGRFeature::CreateFeature(layerMalha->GetLayerDefn());
            feat->SetGeometry(&poly);
            if (layerMalha->CreateFeature(feat) != OGRERR_NONE) {}
            OGRFeature::DestroyFeature(feat);
        }
        layerMalha->SyncToDisk();
        GDALClose(dsMalha);
    }

    // LINHAS DE DISTÂNCIA EXATA ATÉ A MARGEM (Apenas para pontos na área navegável)
    if (!rtree_margem_.empty() && !pontos_teste.empty()) {
        std::string pathLinha = pastaSaida + "/96_RTree_Distancia.shp";
        if (std::filesystem::exists(pathLinha)) driver->Delete(pathLinha.c_str());
        GDALDataset* dsLinha = driver->Create(pathLinha.c_str(), 0, 0, 0, GDT_Unknown, nullptr);
        OGRLayer* layerLinha = dsLinha->CreateLayer("linhas", &srs, wkbLineString, nullptr);
        
        OGRFieldDefn fieldTipo("Tipo", OFTString);
        if (layerLinha->CreateField(&fieldTipo) != OGRERR_NONE) {}
        OGRFieldDefn fieldDist("Distancia", OFTReal);
        if (layerLinha->CreateField(&fieldDist) != OGRERR_NONE) {}

        for (const auto& pt : pontos_teste) {
            BgPonto ponto_busca(pt.get_x(), pt.get_y());

            // Valida se o ponto está na área navegável antes de desenhar a linha de distância
            bool navegavel = false;
            std::vector<ValorRTreePoligono> candidatos;
            rtree_malha_.query(bgi::intersects(ponto_busca), std::back_inserter(candidatos));
            for (const auto& candidato : candidatos) {
                if (bg::within(ponto_busca, triangulos_malha_[candidato.second])) {
                    navegavel = true;
                    break;
                }
            }

            if (!navegavel) continue; // Pula pontos fora da área navegável

            std::vector<ValorRTreeSegmento> resultados;
            rtree_margem_.query(bgi::nearest(ponto_busca, 50), std::back_inserter(resultados));
            
            if (!resultados.empty()) {
                BgSegmento seg_proximo;
                double minDist = std::numeric_limits<double>::max();
                
                for (const auto& res : resultados) {
                    double d = bg::distance(ponto_busca, res.second);
                    if (d < minDist) {
                        minDist = d;
                        seg_proximo = res.second;
                    }
                }

                double px = ponto_busca.get<0>(), py = ponto_busca.get<1>();
                double ax = seg_proximo.first.get<0>(), ay = seg_proximo.first.get<1>();
                double bx = seg_proximo.second.get<0>(), by = seg_proximo.second.get<1>();

                double ab_x = bx - ax, ab_y = by - ay;
                double ap_x = px - ax, ap_y = py - ay;
                double dot_ab_ab = ab_x * ab_x + ab_y * ab_y;
                double t = (dot_ab_ab > 0.0) ? (ap_x * ab_x + ap_y * ab_y) / dot_ab_ab : 0.0;
                
                double proj_x = (t <= 0.0) ? ax : (t >= 1.0) ? bx : ax + t * ab_x;
                double proj_y = (t <= 0.0) ? ay : (t >= 1.0) ? by : ay + t * ab_y;

                OGRLineString linha_distancia;
                linha_distancia.addPoint(px, py); 
                linha_distancia.addPoint(proj_x, proj_y);
                
                OGRFeature* featDist = OGRFeature::CreateFeature(layerLinha->GetLayerDefn());
                featDist->SetField("Tipo", "Distancia_Ate_Margem");
                featDist->SetField("Distancia", minDist);
                featDist->SetGeometry(&linha_distancia);
                if (layerLinha->CreateFeature(featDist) != OGRERR_NONE) {}
                OGRFeature::DestroyFeature(featDist);
            }
        }
        layerLinha->SyncToDisk();
        GDALClose(dsLinha);
    }
    std::cout << "[DEBUG] Shapefiles das R-Trees e Linhas de Distancia gravados (Arquivos 96, 97 e 98).\n";
}

// MÉTODOS DE INTERFACE

float IndiceEspacial::get_closest_static_obstacle_distance(const types::Point& position) const {
    return static_cast<float>(calcularDistanciaMargem(position));
}

bool IndiceEspacial::is_inside_restricted_zone(const types::Point& position) const {
    return !isNavegavel(position);
}

void IndiceEspacial::update_global_targets(const std::vector<types::Target>& targets) {
    alvos_globais_cache_ = targets;
}

std::vector<types::Target> IndiceEspacial::get_active_local_targets(const types::Point& center, float radius) const {
    std::vector<types::Target> locais;
    
    for (const auto& alvo : alvos_globais_cache_) {
        double dx = alvo.get_pose().get_x() - center.get_x();
        double dy = alvo.get_pose().get_y() - center.get_y();
        double dist = std::sqrt(dx * dx + dy * dy);
        
        if (dist <= radius) {
            locais.push_back(alvo);
        }
    }
    return locais;
}