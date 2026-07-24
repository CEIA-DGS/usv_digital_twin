#include "../include/ProcessadorS57.h"
#include <iostream>
#include <cmath>
#include "GdalInicializador.hpp"

/**
 * @brief Executa o parsing analítico de cartas náuticas S-57, extraindo e reprojetando features cartográficas para UTM dinâmico.
 * @param caminhoS57 Caminho físico do arquivo binário S-57 (.000).
 * @param config Estrutura contendo os vetores de strings com as classes de mapeamento desejadas.
 * @return Estrutura GeometriasProcessadas contendo os ponteiros das geometrias reprojetadas para UTM métrico real.
 */
GeometriasProcessadas ProcessadorS57::processarCarta(const std::string& caminhoS57, const ConfiguracaoMapa& config) {
    GeometriasProcessadas resultado;

    // Instancia o dataset vetorial S-57 via abstração nativa da GDAL
    GDALDataset* dataset = (GDALDataset*)GDALOpenEx(caminhoS57.c_str(), GDAL_OF_VECTOR, nullptr, nullptr, nullptr);
    if (!dataset) {
        std::cerr << "Erro: Nao foi possivel abrir a carta S-57: " << caminhoS57 << std::endl;
        exit(1);
    }

    // Descobre o envelope geográfico global da carta para calcular o Fuso UTM dinâmico
    OGREnvelope envelope_global;
    bool envelope_valido = false;
    for (int i = 0; i < dataset->GetLayerCount(); ++i) {
        OGRLayer* layer = dataset->GetLayer(i);
        if (layer) {
            OGREnvelope env_layer;
            if (layer->GetExtent(&env_layer, TRUE) == OGRERR_NONE) {
                if (!envelope_valido) {
                    envelope_global = env_layer;
                    envelope_valido = true;
                } else {
                    envelope_global.Merge(env_layer);
                }
            }
        }
    }

    // Validação estrita: Se o envelope falhar, aborta imediatamente com exceção.
    if (!envelope_valido) {
        throw std::runtime_error("[ProcessadorS57] Erro crítico: Envelope global da carta S-57 inválido ou vazio. Impossível determinar o sistema de coordenadas UTM.");
    }

    double centro_lon = (envelope_global.MinX + envelope_global.MaxX) / 2.0;
    double centro_lat = (envelope_global.MinY + envelope_global.MaxY) / 2.0;

    // Cálculo automático do Fuso UTM e código EPSG baseado na posição da carta
    int fuso = static_cast<int>(std::floor((centro_lon + 180.0) / 6.0)) + 1;
    int epsg_utm = (centro_lat >= 0) ? (32600 + fuso) : (32700 + fuso); // 326xx para Norte, 327xx para Sul

    // Armazena o EPSG calculado na estrutura de retorno
    resultado.epsg_utm = epsg_utm;

    std::cout << "\n[Processador] Centro geografico da carta: Lon=" << centro_lon << ", Lat=" << centro_lat << std::endl;
    std::cout << "[Processador] Fuso UTM detectado: " << fuso << " | Sistema de Destino EPSG:" << epsg_utm << " (Metros Reais)\n";

    // Define o Sistema de Referência Espacial de origem (WGS84 Geográfico)
    OGRSpatialReference sistOrigem;
    sistOrigem.importFromEPSG(4326);
    sistOrigem.SetAxisMappingStrategy(OAMS_TRADITIONAL_GIS_ORDER); // Força ordem Longitude/Latitude padrão GIS

    // Define o Sistema de Referência Espacial de destino (UTM Métrico Dinâmico)
    OGRSpatialReference sistDestino;
    sistDestino.importFromEPSG(epsg_utm);

    // Inicializa o motor matemático de transformação e reprojeção de coordenadas
    OGRCoordinateTransformation* transformador = OGRCreateCoordinateTransformation(&sistOrigem, &sistDestino);
    if (!transformador) {
        std::cerr << "Erro: Falha ao criar o transformador de coordenadas UTM." << std::endl;
        GDALClose(dataset);
        exit(1);
    }

    std::cout << "[Processador] Analisando e extraindo camadas do config.json..." << std::endl;
    
    // Função lambda para processamento em lote, reprojeção espacial e diagnóstico de camadas OGR
    auto extrairCamadasComDiagnostico = [&](const std::vector<std::string>& classes, std::vector<OGRGeometry*>& destino, const std::string& tipoCamada) {
        for (const auto& nomeClasse : classes) {
            OGRLayer* camada = dataset->GetLayerByName(nomeClasse.c_str());
            if (!camada) {
                std::cout << "  [AVISO] Classe de " << tipoCamada << " ('" << nomeClasse << "') NAO existe nesta carta." << std::endl;
                continue;
            }
            
            int totalFeicoes = 0;
            OGRFeature* feicao;
            camada->ResetReading();
            
            // Loop de iteração sobre as features físicas contidas no layer S-57
            while ((feicao = camada->GetNextFeature()) != nullptr) {
                OGRGeometry* geomOriginal = feicao->GetGeometryRef();
                if (geomOriginal != nullptr) {
                    // Clona a geometria para desvincular seu ciclo de vida do OGRFeature
                    OGRGeometry* clonada = geomOriginal->clone();
                    
                    // Executa a transposição matemática de coordenadas in-place para UTM métrico
                    clonada->transform(transformador);
                    destino.push_back(clonada);
                    totalFeicoes++;
                }
                // Liberação da feature consumida para evitar vazamento de memória
                OGRFeature::DestroyFeature(feicao);
            }
            std::cout << "  [OK] Classe " << tipoCamada << " ('" << nomeClasse << "') extraida com sucesso. Feicoes: " << totalFeicoes << std::endl;
        }
    };

    // Execução do pipeline de extração para a submalha hidrogáfica navegável
    std::cout << "--- Extraindo Areas Navegaveis ---" << std::endl;
    extrairCamadasComDiagnostico(config.classesNavegaveis, resultado.areaNavegavel, "Navegavel");

    // Execução do pipeline de extração para a submalha de colisão e contornos de terra firme
    std::cout << "--- Extraindo Obstaculos / Terra ---" << std::endl;
    extrairCamadasComDiagnostico(config.classesColisao, resultado.obstaculos, "Colisao/Terra"); // Corrigido de obstaculosTerra para obstaculos

    // Desalocação do contexto de transformação espacial e fechamento seguro do arquivo
    OGRCoordinateTransformation::DestroyCT(transformador);
    GDALClose(dataset);
    
    return resultado;
}