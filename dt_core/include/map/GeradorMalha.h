#pragma once
#include <vector>
#include "gdal_priv.h"
#include "ProcessadorS57.h"

/**
 * @brief Primitivas geométricas para representação da malha de navegação.
 */
struct Ponto2D { double x, y; };
struct Triangulo { Ponto2D p1, p2, p3; };

/**
 * @brief Estrutura de dados contendo a malha triangular processada e geometrias auxiliares.
 */
struct MalhaNavegacao {
    std::vector<Triangulo> triangulos;
    OGRGeometry* terraOriginal = nullptr;
    OGRGeometry* margemSeguranca = nullptr;
    OGRGeometry* perimetroNavegavelSeguro = nullptr;

    /**
     * @brief Desaloca a memória ocupada pelas geometrias OGR mantidas em cache.
     */
    void liberarMemoria() {
        if (terraOriginal) OGRGeometryFactory::destroyGeometry(terraOriginal);
        if (margemSeguranca) OGRGeometryFactory::destroyGeometry(margemSeguranca);
        if (perimetroNavegavelSeguro) OGRGeometryFactory::destroyGeometry(perimetroNavegavelSeguro);
    }
};

/**
 * @brief Motor geométrico para processamento e geração da malha de navegação.
 * 
 * Atua como uma classe utilitária estática. Executa operações booleanas, 
 * filtragem espacial e a Triangulação de Delaunay Restrita (CDT).
 */
class GeradorMalha {
public:
    // Evita a instanciação acidental da classe
    GeradorMalha() = delete;

    /**
     * @brief Pipeline completo de geração da malha a partir de dados S-57 processados.
     * @param geometrias Estrutura contendo os polígonos base de água e terra.
     * @param margemMetros Distância da margem de segurança a ser expandida (Buffer).
     * @param toleranciaSimplificacao Erro máximo permitido (em metros) na redução de vértices (Douglas-Peucker).
     * @return MalhaNavegacao contendo os triângulos gerados e polígonos auxiliares.
     */
    static MalhaNavegacao gerar(const GeometriasProcessadas& geometrias, double margemMetros, double toleranciaSimplificacao);

private:
    /**
     * @brief Consolida um conjunto de geometrias fragmentadas em um único objeto unificado.
     * @param listaGeometrias Vetor de ponteiros para as geometrias a serem unidas.
     * @return Ponteiro OGRGeometry para a geometria fundida resultante.
     */
    static OGRGeometry* unirGeometrias(const std::vector<OGRGeometry*>& listaGeometrias);
    
    /**
     * @brief Executa a triangulação CDT (Constrained Delaunay) em polígonos com furos.
     * @param poligono Ponteiro do polígono base a ser triangulado.
     * @param listaDestino Vetor de saída onde as estruturas de Triângulos geradas serão anexadas.
     * @param contadorFalhas Referência para contagem de falhas do motor geométrico poly2tri.
     */
    static void triangularPoligono(OGRPolygon* poligono, std::vector<Triangulo>& listaDestino, int& contadorFalhas);
    
    /**
     * @brief Filtra features poligonais baseadas em um limiar mínimo de área.
     * @param geom Ponteiro para a geometria de entrada original.
     * @param areaMinima Área mínima exigida em metros quadrados para que a feature seja mantida.
     * @return Ponteiro OGRGeometry para a nova geometria filtrada.
     */
    static OGRGeometry* filtrarPorArea(OGRGeometry* geom, double areaMinima);
};