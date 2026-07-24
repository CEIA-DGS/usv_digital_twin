#include "../include/GeradorMalha.h"
#include <iostream>
#include <cmath>
#include <poly2tri/poly2tri.h>

/**
 * @brief Extrai e limpa os vértices de um anel linear para compatibilidade com a biblioteca poly2tri.
 * Remove pontos duplicados dentro de uma tolerância espacial e elimina a auto-inclusão do ponto de fechamento.
 * @param anel Ponteiro para o anel linear original da GDAL/OGR.
 * @return Vetor de ponteiros de pontos alocados dinamicamente para o poly2tri.
 */

std::vector<p2t::Point*> extrairContornoLimpo(OGRLinearRing* anel) {
    std::vector<p2t::Point*> contorno;
    if (!anel || anel->getNumPoints() < 4) return contorno;
    
    double epsilon = 0.1; // Tolerância para eliminação de vértices redundantes/colineares
    for (int i = 0; i < anel->getNumPoints() - 1; i++) {
        double x = anel->getX(i);
        double y = anel->getY(i);
        if (!contorno.empty()) {
            if (std::abs(x - contorno.back()->x) < epsilon && std::abs(y - contorno.back()->y) < epsilon) continue;
        }
        contorno.push_back(new p2t::Point(x, y));
    }
    
    // Garante que o contorno não replique o ponto inicial no final
    if (contorno.size() >= 3) {
        if (std::abs(contorno.front()->x - contorno.back()->x) < epsilon && 
            std::abs(contorno.front()->y - contorno.back()->y) < epsilon) {
            delete contorno.back();
            contorno.pop_back();
        }
    }
    return contorno;
}

/**
 * @brief Normaliza qualquer primitiva geométrica para o tipo poligonal.
 * Aplica uma operação de buffer inflatório mínimo para converter linhas e pontos em polígonos válidos.
 */
static OGRGeometry* garantirPoligono(OGRGeometry* geom) {
    if (!geom) return nullptr;
    OGRwkbGeometryType tipo = wkbFlatten(geom->getGeometryType());
    
    if (tipo == wkbPolygon || tipo == wkbMultiPolygon) {
        return geom->clone();
    } else {
        return geom->Buffer(1.0); // Buffer estático para blindagem contra erros de tipo de feição
    }
}

/**
 * @brief Filtra e descarta componentes poligonais cuja área seja inferior ao limiar especificado.
 * Utilizado para eliminação de slivers e artefatos geométricos gerados por operações booleanas.
 */

OGRGeometry* GeradorMalha::filtrarPorArea(OGRGeometry* geom, double areaMinima) {
    if (!geom) return nullptr;
    OGRwkbGeometryType tipo = wkbFlatten(geom->getGeometryType());

    if (tipo == wkbPolygon) {
        OGRPolygon* poly = geom->toPolygon();
        if (poly->get_Area() >= areaMinima) return poly->clone();
        return nullptr;
    } 
    else if (tipo == wkbMultiPolygon || tipo == wkbGeometryCollection) {
        OGRGeometryCollection* col = (OGRGeometryCollection*)geom;
        OGRMultiPolygon* multiNovo = (OGRMultiPolygon*)OGRGeometryFactory::createGeometry(wkbMultiPolygon);
        for (auto&& sub : *col) {
            if (wkbFlatten(sub->getGeometryType()) == wkbPolygon) {
                OGRPolygon* poly = sub->toPolygon();
                if (poly->get_Area() >= areaMinima) {
                    multiNovo->addGeometry(poly->clone());
                }
            }
        }
        if (multiNovo->getNumGeometries() > 0) return multiNovo;
        OGRGeometryFactory::destroyGeometry(multiNovo);
        return nullptr;
    }
    return geom->clone();
}

/**
 * @brief Pipeline principal para a geração do NavMesh vetorial e cálculo de restrições espaciais.
 * Executa normalização, expansão de buffers protetores, diferenças booleanas e triangulação restrita.
 */
MalhaNavegacao GeradorMalha::gerar(const GeometriasProcessadas& geometrias, double margemMetros, double toleranciaSimplificacao) {
    MalhaNavegacao navMesh;

    std::cout << "[GeradorMalha] Processando e convertendo obstaculos em poligonos..." << std::endl;
    std::vector<OGRGeometry*> obstaculosPoligonais;
    for (auto* geom : geometrias.obstaculos) {
        OGRGeometry* polyValido = garantirPoligono(geom);
        if (polyValido) {
            obstaculosPoligonais.push_back(polyValido);
        }
    }

    std::cout << "[GeradorMalha] Unindo poligonos de agua e terra..." << std::endl;
    OGRGeometry* megaOceano = unirGeometrias(geometrias.areaNavegavel);
    OGRGeometry* terraOriginal = unirGeometrias(obstaculosPoligonais);

    for (auto* p : obstaculosPoligonais) OGRGeometryFactory::destroyGeometry(p);

    navMesh.terraOriginal = terraOriginal ? terraOriginal->clone() : nullptr;

    std::cout << "[GeradorMalha] Calculando margens de seguranca fisicas (Buffer Duplo)..." << std::endl;
    OGRGeometry* terraExpandidaReal = nullptr;
    OGRGeometry* terraExpandidaNavMesh = nullptr;

    if (terraOriginal && margemMetros > 0.0) {
        terraExpandidaReal = terraOriginal->Buffer(margemMetros);
        navMesh.margemSeguranca = terraExpandidaReal->Difference(terraOriginal);
        terraExpandidaNavMesh = terraOriginal->Buffer(margemMetros + toleranciaSimplificacao);
    } else if (terraOriginal) {
        terraExpandidaReal = terraOriginal->clone();
        terraExpandidaNavMesh = terraOriginal->clone();
    }

    std::cout << "[GeradorMalha] Realizando Diferenca Booleana (Agua - Obstaculos)..." << std::endl;
    OGRGeometry* areaSeguraBruta = nullptr;
    if (megaOceano && terraExpandidaNavMesh) {
        areaSeguraBruta = megaOceano->Difference(terraExpandidaNavMesh);
    } else if (megaOceano) {
        areaSeguraBruta = megaOceano->clone();
    }

    std::cout << "[GeradorMalha] Filtrando micro-ilhas/pocas (Slivers < 25m2)..." << std::endl;
    OGRGeometry* areaFiltrada = filtrarPorArea(areaSeguraBruta, 25.0);

    std::cout << "[GeradorMalha] Simplificando contornos do NavMesh..." << std::endl;
    if (areaFiltrada && toleranciaSimplificacao > 0.0) {
        // Reduz a densidade de vértices preservando a topologia.
        // A invasão máxima será = toleranciaSimplificacao, parando exatamente na Margem Real.
        navMesh.perimetroNavegavelSeguro = areaFiltrada->SimplifyPreserveTopology(toleranciaSimplificacao);
    } else if (areaFiltrada) {
        navMesh.perimetroNavegavelSeguro = areaFiltrada->clone();
    }

    // Liberação de memória das geometrias intermediárias do pipeline
    if (megaOceano) OGRGeometryFactory::destroyGeometry(megaOceano);
    if (terraOriginal) OGRGeometryFactory::destroyGeometry(terraOriginal);
    if (terraExpandidaReal) OGRGeometryFactory::destroyGeometry(terraExpandidaReal);
    if (terraExpandidaNavMesh) OGRGeometryFactory::destroyGeometry(terraExpandidaNavMesh);
    if (areaSeguraBruta) OGRGeometryFactory::destroyGeometry(areaSeguraBruta);
    if (areaFiltrada) OGRGeometryFactory::destroyGeometry(areaFiltrada);

    if (!navMesh.perimetroNavegavelSeguro) return navMesh;

    std::cout << "[GeradorMalha] Iniciando Triangulacao (CDT)..." << std::endl;
    int contadorFalhas = 0;

    // Função interna lambda para processamento de geometrias simples e complexas (MultiPolygons)
    auto processarGeometria = [&](OGRGeometry* geom) {
        OGRwkbGeometryType tipo = wkbFlatten(geom->getGeometryType());
        if (tipo == wkbPolygon) {
            triangularPoligono(geom->toPolygon(), navMesh.triangulos, contadorFalhas);
        } 
        else if (tipo == wkbMultiPolygon || tipo == wkbGeometryCollection) {
            OGRGeometryCollection* colecao = (OGRGeometryCollection*)geom;
            for (auto&& subGeom : *colecao) {
                if (wkbFlatten(subGeom->getGeometryType()) == wkbPolygon) {
                    triangularPoligono(subGeom->toPolygon(), navMesh.triangulos, contadorFalhas);
                }
            }
        }
    };

    processarGeometria(navMesh.perimetroNavegavelSeguro);

    if (contadorFalhas > 0) {
        std::cout << "[GeradorMalha] AVISO: " << contadorFalhas << " sub-poligonos problematicos ignorados." << std::endl;
    }
    std::cout << "[GeradorMalha] Malha concluida! Triangulos gerados: " << navMesh.triangulos.size() << std::endl;
    return navMesh;
}

/**
 * @brief Consolida um vetor de primitivas geométricas em um único objeto unificado via operações de união.
 */

OGRGeometry* GeradorMalha::unirGeometrias(const std::vector<OGRGeometry*>& listaGeometrias) {
    if (listaGeometrias.empty()) return nullptr;
    OGRGeometry* uniao = listaGeometrias[0]->clone();
    for (size_t i = 1; i < listaGeometrias.size(); i++) {
        OGRGeometry* temp = uniao->Union(listaGeometrias[i]);
        if (temp) {
            OGRGeometryFactory::destroyGeometry(uniao);
            uniao = temp;
        }
    }
    return uniao;
}

/**
 * @brief Executa a Triangulação de Delaunay Restrita (CDT) em um polígono individual contendo furos internos.
 * @param poligono Ponteiro do polígono Alvo da GDAL.
 * @param listaDestino Vetor de saída onde as estruturas de Triângulos geradas serão anexadas.
 * @param contadorFalhas Referência para incremento em caso de exceções no motor geométrico da biblioteca poly2tri.
 */
void GeradorMalha::triangularPoligono(OGRPolygon* poligono, std::vector<Triangulo>& listaDestino, int& contadorFalhas) {
    if (!poligono) return;
    
    // Força correção de fechamento topológico e auto-interseções com buffer zero
    OGRPolygon* polyLimpo = (OGRPolygon*)poligono->Buffer(0.0);
    if (!polyLimpo) polyLimpo = poligono;
    
    OGRLinearRing* anelExterno = polyLimpo->getExteriorRing();
    std::vector<p2t::Point*> contornoExterno = extrairContornoLimpo(anelExterno);

    if (contornoExterno.size() < 3) {
        for (auto* p : contornoExterno) delete p;
        if (polyLimpo != poligono) OGRGeometryFactory::destroyGeometry(polyLimpo);
        return;
    }

    p2t::CDT* cdt = nullptr;
    std::vector<std::vector<p2t::Point*>> listaBuracos;

    try {
        // Inicializa o motor estrutural CDT passando a fronteira externa
        cdt = new p2t::CDT(contornoExterno);
        
        // Injeta os anéis internos (ilhas/restrições) como furos geométricos no motor
        int numBuracos = polyLimpo->getNumInteriorRings();
        for (int b = 0; b < numBuracos; b++) {
            std::vector<p2t::Point*> buracoP2T = extrairContornoLimpo(polyLimpo->getInteriorRing(b));
            if (buracoP2T.size() >= 3) {
                cdt->AddHole(buracoP2T);
                listaBuracos.push_back(buracoP2T);
            } else {
                for (auto* p : buracoP2T) delete p;
            }
        }
        
        // Processa as restrições e gera a malha triangular
        cdt->Triangulate();
        
        // Mapeia os triângulos gerados pelo poly2tri para a estrutura interna nativa do sistema
        std::vector<p2t::Triangle*> tris = cdt->GetTriangles();
        for (auto* t : tris) {
            listaDestino.push_back({{t->GetPoint(0)->x, t->GetPoint(0)->y}, {t->GetPoint(1)->x, t->GetPoint(1)->y}, {t->GetPoint(2)->x, t->GetPoint(2)->y}});
        }
    } catch (...) { 
        contadorFalhas++; 
    }

    // Liberação de memória local do escopo de triangulação
    if (cdt) delete cdt;
    for (auto* p : contornoExterno) delete p;
    for (auto& buraco : listaBuracos) { for (auto* p : buraco) delete p; }
    if (polyLimpo != poligono) OGRGeometryFactory::destroyGeometry(polyLimpo);
}