#include "../../include/map/s57_processor.hpp"
#include <iostream>
#include <cmath>
#include "../../include/map/gdal_initializer.hpp"

/**
 * @brief Executa o parsing analítico de cartas náuticas S-57, extraindo e reprojetando features cartográficas para UTM dinâmico.
 * @param s57_path Caminho físico do arquivo binário S-57 (.000).
 * @param config Estrutura contendo os vetores de strings com as classes de mapeamento desejadas.
 * @return Estrutura ProcessedGeometries contendo os ponteiros das geometrias reprojetadas para UTM métrico real.
 */
ProcessedGeometries S57Processor::process_chart(const std::string& s57_path, const MapConfiguration& config) {
    ProcessedGeometries result;

    // Instancia o dataset vetorial S-57 via abstração nativa da GDAL
    GDALDataset* dataset = (GDALDataset*)GDALOpenEx(s57_path.c_str(), GDAL_OF_VECTOR, nullptr, nullptr, nullptr);
    if (!dataset) {
        std::cerr << "Erro: Nao foi possivel abrir a carta S-57: " << s57_path << std::endl;
        exit(1);
    }

    // Descobre o envelope geográfico global da carta para calcular o Fuso UTM dinâmico
    OGREnvelope global_envelope;
    bool is_envelope_valid = false;
    for (int i = 0; i < dataset->GetLayerCount(); ++i) {
        OGRLayer* layer = dataset->GetLayer(i);
        if (layer) {
            OGREnvelope layer_envelope;
            if (layer->GetExtent(&layer_envelope, TRUE) == OGRERR_NONE) {
                if (!is_envelope_valid) {
                    global_envelope = layer_envelope;
                    is_envelope_valid = true;
                } else {
                    global_envelope.Merge(layer_envelope);
                }
            }
        }
    }

    // Validação estrita: Se o envelope falhar, aborta imediatamente com exceção.
    if (!is_envelope_valid) {
        throw std::runtime_error("[S57Processor] Erro crítico: Envelope global da carta S-57 inválido ou vazio. Impossível determinar o sistema de coordenadas UTM.");
    }

    double center_lon = (global_envelope.MinX + global_envelope.MaxX) / 2.0;
    double center_lat = (global_envelope.MinY + global_envelope.MaxY) / 2.0;

    // Cálculo automático do Fuso UTM e código EPSG baseado na posição da carta
    int utm_zone = static_cast<int>(std::floor((center_lon + 180.0) / 6.0)) + 1;
    int epsg_utm = (center_lat >= 0) ? (32600 + utm_zone) : (32700 + utm_zone); // 326xx para Norte, 327xx para Sul

    // Armazena o EPSG calculado na estrutura de retorno
    result.dynamic_utm_epsg = epsg_utm;

    std::cout << "\n[Processador] Centro geografico da carta: Lon=" << center_lon << ", Lat=" << center_lat << std::endl;
    std::cout << "[Processador] Fuso UTM detectado: " << utm_zone << " | Sistema de Destino EPSG:" << epsg_utm << " (Metros Reais)\n";

    // Define o Sistema de Referência Espacial de origem (WGS84 Geográfico)
    OGRSpatialReference source_srs;
    source_srs.importFromEPSG(4326);
    source_srs.SetAxisMappingStrategy(OAMS_TRADITIONAL_GIS_ORDER); // Força ordem Longitude/Latitude padrão GIS

    // Define o Sistema de Referência Espacial de destino (UTM Métrico Dinâmico)
    OGRSpatialReference target_srs;
    target_srs.importFromEPSG(epsg_utm);

    // Inicializa o motor matemático de transformação e reprojeção de coordenadas
    OGRCoordinateTransformation* transformer = OGRCreateCoordinateTransformation(&source_srs, &target_srs);
    if (!transformer) {
        std::cerr << "Erro: Falha ao criar o transformador de coordenadas UTM." << std::endl;
        GDALClose(dataset);
        exit(1);
    }

    std::cout << "[Processador] Analisando e extraindo camadas do config.json..." << std::endl;
    
    // Função lambda para processamento em lote, reprojeção espacial e diagnóstico de camadas OGR
    auto extract_layers_with_diagnostic = [&](const std::vector<std::string>& classes, std::vector<OGRGeometry*>& target_vector, const std::string& layer_type_name) {
        for (const auto& class_name : classes) {
            OGRLayer* layer = dataset->GetLayerByName(class_name.c_str());
            if (!layer) {
                std::cout << "  [AVISO] Classe de " << layer_type_name << " ('" << class_name << "') NAO existe nesta carta." << std::endl;
                continue;
            }
            
            int total_features = 0;
            OGRFeature* feature;
            layer->ResetReading();
            
            // Loop de iteração sobre as features físicas contidas no layer S-57
            while ((feature = layer->GetNextFeature()) != nullptr) {
                OGRGeometry* original_geom = feature->GetGeometryRef();
                if (original_geom != nullptr) {
                    // Clona a geometria para desvincular seu ciclo de vida do OGRFeature
                    OGRGeometry* cloned_geom = original_geom->clone();
                    
                    // Executa a transposição matemática de coordenadas in-place para UTM métrico
                    cloned_geom->transform(transformer);
                    target_vector.push_back(cloned_geom);
                    total_features++;
                }
                // Liberação da feature consumida para evitar vazamento de memória
                OGRFeature::DestroyFeature(feature);
            }
            std::cout << "  [OK] Classe " << layer_type_name << " ('" << class_name << "') extraida com sucesso. Feicoes: " << total_features << std::endl;
        }
    };

    // Execução do pipeline de extração para a submalha hidrogáfica navegável
    std::cout << "--- Extraindo Areas Navegaveis ---" << std::endl;
    extract_layers_with_diagnostic(config.navigable_classes, result.navigable_area, "Navegavel");

    // Execução do pipeline de extração para a submalha de colisão e contornos de terra firme
    std::cout << "--- Extraindo Obstaculos / Terra ---" << std::endl;
    extract_layers_with_diagnostic(config.collision_classes, result.obstacles, "Colisao/Terra"); 

    // Desalocação do contexto de transformação espacial e fechamento seguro do arquivo
    OGRCoordinateTransformation::DestroyCT(transformer);
    GDALClose(dataset);
    
    return result;
}