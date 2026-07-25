#include "../../include/map/ExportadorVetor.h"
#include <iostream>
#include "map/GdalInicializador.hpp"

/**
 * @brief Exporta os componentes geográficos e a malha de navegação para arquivos ESRI Shapefile em UTM real.
 * @param navMesh Estrutura contendo as geometrias processadas e os triângulos gerados.
 * @param diretorioSaida Caminho do diretório onde as camadas (.shp) serão salvas.
 * @param epsg_utm Código EPSG UTM calculado dinamicamente para garantir distâncias reais.
 */
void ExportadorVetor::exportarShapefile(const MalhaNavegacao& navMesh, const std::string& diretorioSaida, int epsg_utm) {
    std::cout << "[Exportador] Gerando arquivos Shapefile em UTM (EPSG:" << epsg_utm << ") em: " << diretorioSaida << std::endl;

    // Inicializa e valida o driver geoespacial ESRI Shapefile
    const char *pszDriverName = "ESRI Shapefile";
    GDALDriver *poDriver = GetGDALDriverManager()->GetDriverByName(pszDriverName);
    
    if (!poDriver) {
        std::cerr << "Erro: Driver ESRI Shapefile nao disponivel na GDAL." << std::endl;
        return;
    }

    // Cria o dataset vetorial físico
    GDALDataset *poDS = poDriver->Create(diretorioSaida.c_str(), 0, 0, 0, GDT_Unknown, NULL);
    if (!poDS) {
        std::cerr << "Erro: Nao foi possivel criar o Shapefile. Certifique-se de que a pasta anterior nao esta aberta em outro programa." << std::endl;
        return;
    }

    // Define o Sistema de Referência Espacial (SRS) com o UTM dinâmico real
    OGRSpatialReference srs;
    srs.importFromEPSG(epsg_utm); 

    // Instancia as camadas lógicas estruturadas para classificação e inspeção no QGIS em distâncias reais
    OGRLayer *layerTerra      = poDS->CreateLayer("1_Terra_Firme", &srs, wkbMultiPolygon, NULL);
    OGRLayer *layerMargem     = poDS->CreateLayer("2_Margem_Seguranca", &srs, wkbMultiPolygon, NULL);
    OGRLayer *layerAreaSegura = poDS->CreateLayer("3_Area_Navegavel_Limpa", &srs, wkbMultiPolygon, NULL);
    OGRLayer *layerMalha      = poDS->CreateLayer("4_Malha_NavMesh", &srs, wkbPolygon, NULL);

    // Grava as features geográficas base do ambiente de navegação
    std::cout << "[Exportador] Gravando vetores fisicos..." << std::endl;
    inserirGeometria(layerTerra, navMesh.terraOriginal);
    inserirGeometria(layerMargem, navMesh.margemSeguranca);
    inserirGeometria(layerAreaSegura, navMesh.perimetroNavegavelSeguro);

    // Converte e exporta a lista de triângulos da malha para polígonos OGR
    std::cout << "[Exportador] Gravando " << navMesh.triangulos.size() << " triangulos matematicos..." << std::endl;
    for (const auto& tri : navMesh.triangulos) {
        OGRPolygon polyTri;
        OGRLinearRing ring;
        
        // Constrói o anel fechado do triângulo
        ring.addPoint(tri.p1.x, tri.p1.y);
        ring.addPoint(tri.p2.x, tri.p2.y);
        ring.addPoint(tri.p3.x, tri.p3.y);
        ring.addPoint(tri.p1.x, tri.p1.y); 
        polyTri.addRing(&ring);
        
        inserirGeometria(layerMalha, &polyTri);
    }

    // Fecha o dataset para descarregar os buffers e consolidar a gravação no disco
    GDALClose(poDS);
    std::cout << "[Exportador] Exportacao vetorizada em UTM concluida com sucesso!" << std::endl;
}

/**
 * @brief Envelopa uma geometria OGR pura em uma Feature e a consolida na camada alvo.
 * @param layer Ponteiro da camada OGR de destino.
 * @param geom Ponteiro da geometria a ser inserida.
 */
void ExportadorVetor::inserirGeometria(OGRLayer* layer, OGRGeometry* geom) {
    if (!geom || !layer) return;
    
    // Instancia o container de feição baseado na definição estrutural da camada
    OGRFeature *poFeature = OGRFeature::CreateFeature(layer->GetLayerDefn());
    poFeature->SetGeometry(geom);
    
    // Captura o código de erro retornado pela GDAL para satisfazer o compilador
    if (layer->CreateFeature(poFeature) != OGRERR_NONE) {
        std::cerr << "[Exportador] Aviso: Falha ao gravar uma geometria no disco." << std::endl;
    }
    
    // Destrói o ponteiro corretamente após a verificação
    OGRFeature::DestroyFeature(poFeature);
}