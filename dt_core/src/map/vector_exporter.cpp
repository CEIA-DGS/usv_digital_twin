#include "../../include/map/vector_exporter.hpp"
#include <iostream>
#include "map/gdal_initializer.hpp"

/**
 * @brief Exporta os componentes geográficos e a malha de navegação para arquivos ESRI Shapefile em UTM real.
 * @param nav_mesh Estrutura contendo as geometrias processadas e os triângulos gerados.
 * @param output_dir Caminho do diretório onde as camadas (.shp) serão salvas.
 * @param epsg_utm Código EPSG UTM calculado dinamicamente para garantir distâncias reais.
 */
void VectorExporter::export_shapefile(const NavigationMesh& nav_mesh, const std::string& output_dir, int epsg_utm) {
    std::cout << "[Exportador] Gerando arquivos Shapefile em UTM (EPSG:" << epsg_utm << ") em: " << output_dir << std::endl;

    // Inicializa e valida o driver geoespacial ESRI Shapefile
    const char* driver_name = "ESRI Shapefile";
    GDALDriver* driver = GetGDALDriverManager()->GetDriverByName(driver_name);
    
    if (!driver) {
        std::cerr << "Erro: Driver ESRI Shapefile nao disponivel na GDAL." << std::endl;
        return;
    }

    // Cria o dataset vetorial físico
    GDALDataset* dataset = driver->Create(output_dir.c_str(), 0, 0, 0, GDT_Unknown, NULL);
    if (!dataset) {
        std::cerr << "Erro: Nao foi possivel criar o Shapefile. Certifique-se de que a pasta anterior nao esta aberta em outro programa." << std::endl;
        return;
    }

    // Define o Sistema de Referência Espacial (SRS) com o UTM dinâmico real
    OGRSpatialReference spatial_ref;
    spatial_ref.importFromEPSG(epsg_utm); 

    // Instancia as camadas lógicas estruturadas para classificação e inspeção no QGIS em distâncias reais
    OGRLayer* layer_land = dataset->CreateLayer("1_Terra_Firme", &spatial_ref, wkbMultiPolygon, NULL);
    OGRLayer* layer_margin = dataset->CreateLayer("2_Margem_Seguranca", &spatial_ref, wkbMultiPolygon, NULL);
    OGRLayer* layer_safe_area = dataset->CreateLayer("3_Area_Navegavel_Limpa", &spatial_ref, wkbMultiPolygon, NULL);
    OGRLayer* layer_mesh = dataset->CreateLayer("4_Malha_NavMesh", &spatial_ref, wkbPolygon, NULL);

    // Grava as features geográficas base do ambiente de navegação
    std::cout << "[Exportador] Gravando vetores fisicos..." << std::endl;
    insert_geometry(layer_land, nav_mesh.original_land);
    insert_geometry(layer_margin, nav_mesh.safety_margin);
    insert_geometry(layer_safe_area, nav_mesh.safe_navigable_perimeter);

    // Converte e exporta a lista de triângulos da malha para polígonos OGR
    std::cout << "[Exportador] Gravando " << nav_mesh.triangles.size() << " triangulos matematicos..." << std::endl;
    for (const auto& tri : nav_mesh.triangles) {
        OGRPolygon poly_tri;
        OGRLinearRing ring;
        
        // Constrói o anel fechado do triângulo
        ring.addPoint(tri.p1.x, tri.p1.y);
        ring.addPoint(tri.p2.x, tri.p2.y);
        ring.addPoint(tri.p3.x, tri.p3.y);
        ring.addPoint(tri.p1.x, tri.p1.y); 
        poly_tri.addRing(&ring);
        
        insert_geometry(layer_mesh, &poly_tri);
    }

    // Fecha o dataset para descarregar os buffers e consolidar a gravação no disco
    GDALClose(dataset);
    std::cout << "[Exportador] Exportacao vetorizada em UTM concluida com sucesso!" << std::endl;
}

/**
 * @brief Envelopa uma geometria OGR pura em uma Feature e a consolida na camada alvo.
 * @param layer Ponteiro da camada OGR de destino.
 * @param geom Ponteiro da geometria a ser inserida.
 */
void VectorExporter::insert_geometry(OGRLayer* layer, OGRGeometry* geom) {
    if (!geom || !layer) return;
    
    // Instancia o container de feição baseado na definição estrutural da camada
    OGRFeature* feature = OGRFeature::CreateFeature(layer->GetLayerDefn());
    feature->SetGeometry(geom);
    
    // Captura o código de erro retornado pela GDAL para satisfazer o compilador (warn_unused_result)
    if (layer->CreateFeature(feature) != OGRERR_NONE) {
        std::cerr << "[Exportador] Aviso: Falha ao gravar uma geometria no disco." << std::endl;
    }
    
    // Destrói o ponteiro corretamente após a verificação
    OGRFeature::DestroyFeature(feature);
}