#include "../../include/map/vector_exporter.hpp"
#include <iostream>
#include "map/gdal_initializer.hpp"

/**
 * @brief Exports geographic components and the navigation mesh to ESRI Shapefiles in real UTM.
 * @param nav_mesh Structure containing the processed geometries and generated triangles.
 * @param output_dir Directory path where the layers (.shp) will be saved.
 * @param epsg_utm Dynamically calculated UTM EPSG code to ensure real distances.
 */
void VectorExporter::export_shapefile(const NavigationMesh& nav_mesh, const std::string& output_dir, int epsg_utm) {
    std::cout << "[Exporter] Generating Shapefiles in UTM (EPSG:" << epsg_utm << ") at: " << output_dir << std::endl;

    // Initializes and validates the ESRI Shapefile geospatial driver
    const char* driver_name = "ESRI Shapefile";
    GDALDriver* driver = GetGDALDriverManager()->GetDriverByName(driver_name);
    
    if (!driver) {
        std::cerr << "Error: ESRI Shapefile driver not available in GDAL." << std::endl;
        return;
    }

    // Creates the physical vector dataset
    GDALDataset* dataset = driver->Create(output_dir.c_str(), 0, 0, 0, GDT_Unknown, NULL);
    if (!dataset) {
        std::cerr << "Error: Could not create the Shapefile. Ensure the previous folder is not open in another program." << std::endl;
        return;
    }

    // Defines the Spatial Reference System (SRS) with the real dynamic UTM
    OGRSpatialReference spatial_ref;
    spatial_ref.importFromEPSG(epsg_utm); 

    // Instantiates structured logical layers for classification and inspection in QGIS at real distances
    OGRLayer* layer_land = dataset->CreateLayer("1_Terra_Firme", &spatial_ref, wkbMultiPolygon, NULL);
    OGRLayer* layer_margin = dataset->CreateLayer("2_Margem_Seguranca", &spatial_ref, wkbMultiPolygon, NULL);
    OGRLayer* layer_safe_area = dataset->CreateLayer("3_Area_Navegavel_Limpa", &spatial_ref, wkbMultiPolygon, NULL);
    OGRLayer* layer_mesh = dataset->CreateLayer("4_Malha_NavMesh", &spatial_ref, wkbPolygon, NULL);

    // Writes the base geographic features of the navigation environment
    std::cout << "[Exporter] Writing physical vectors..." << std::endl;
    insert_geometry(layer_land, nav_mesh.original_land);
    insert_geometry(layer_margin, nav_mesh.safety_margin);
    insert_geometry(layer_safe_area, nav_mesh.safe_navigable_perimeter);

    // Converts and exports the mesh triangle list to OGR polygons
    std::cout << "[Exporter] Writing " << nav_mesh.triangles.size() << " mathematical triangles..." << std::endl;
    for (const auto& tri : nav_mesh.triangles) {
        OGRPolygon poly_tri;
        OGRLinearRing ring;
        
        // Builds the closed ring of the triangle
        ring.addPoint(tri.p1.x, tri.p1.y);
        ring.addPoint(tri.p2.x, tri.p2.y);
        ring.addPoint(tri.p3.x, tri.p3.y);
        ring.addPoint(tri.p1.x, tri.p1.y); 
        poly_tri.addRing(&ring);
        
        insert_geometry(layer_mesh, &poly_tri);
    }

    // Closes the dataset to flush the buffers and consolidate the recording on disk
    GDALClose(dataset);
    std::cout << "[Exporter] Vector export in UTM completed successfully!" << std::endl;
}

/**
 * @brief Envelopes a pure OGR geometry into a Feature and consolidates it in the target layer.
 * @param layer Pointer to the destination OGR layer.
 * @param geom Pointer to the geometry to be inserted.
 */
void VectorExporter::insert_geometry(OGRLayer* layer, OGRGeometry* geom) {
    if (!geom || !layer) return;
    
    // Instantiates the feature container based on the layer's structural definition
    OGRFeature* feature = OGRFeature::CreateFeature(layer->GetLayerDefn());
    feature->SetGeometry(geom);
    
    // Captures the error code returned by GDAL to satisfy the compiler (warn_unused_result)
    if (layer->CreateFeature(feature) != OGRERR_NONE) {
        std::cerr << "[Exporter] Warning: Failed to write a geometry to disk." << std::endl;
    }
    
    // Properly destroys the pointer after the check
    OGRFeature::DestroyFeature(feature);
}