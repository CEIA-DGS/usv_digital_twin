#include "../../include/map/s57_processor.hpp"
#include <iostream>
#include <cmath>
#include "../../include/map/gdal_initializer.hpp"

ProcessedGeometries S57Processor::process_chart(const std::string& s57_path, const MapConfiguration& config) {
    ProcessedGeometries result;

    // Instantiates the S-57 vector dataset via native GDAL abstraction
    GDALDataset* dataset = (GDALDataset*)GDALOpenEx(s57_path.c_str(), GDAL_OF_VECTOR, nullptr, nullptr, nullptr);
    if (!dataset) {
        std::cerr << "Error: Could not open the S-57 chart: " << s57_path << std::endl;
        exit(1);
    }

    // Discovers the global geographic envelope of the chart to calculate the dynamic UTM Zone
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

    // Strict validation: If the envelope fails, abort immediately with an exception.
    if (!is_envelope_valid) {
        throw std::runtime_error("[S57Processor] Critical error: Global envelope of the S-57 chart is invalid or empty. Impossible to determine the UTM coordinate system.");
    }

    double center_lon = (global_envelope.MinX + global_envelope.MaxX) / 2.0;
    double center_lat = (global_envelope.MinY + global_envelope.MaxY) / 2.0;

    // Automatic calculation of the UTM Zone and EPSG code based on the chart's position
    int utm_zone = static_cast<int>(std::floor((center_lon + 180.0) / 6.0)) + 1;
    int epsg_utm = (center_lat >= 0) ? (32600 + utm_zone) : (32700 + utm_zone); // 326xx for North, 327xx for South

    // Stores the calculated EPSG in the return structure
    result.dynamic_utm_epsg = epsg_utm;

    std::cout << "\n[Processor] Geographic center of the chart: Lon=" << center_lon << ", Lat=" << center_lat << std::endl;
    std::cout << "[Processor] UTM Zone detected: " << utm_zone << " | Target System EPSG:" << epsg_utm << " (Real Meters)\n";

    // Defines the source Spatial Reference System (Geographic WGS84)
    OGRSpatialReference source_srs;
    source_srs.importFromEPSG(4326);
    source_srs.SetAxisMappingStrategy(OAMS_TRADITIONAL_GIS_ORDER); // Forces standard GIS Longitude/Latitude order

    // Defines the target Spatial Reference System (Dynamic Metric UTM)
    OGRSpatialReference target_srs;
    target_srs.importFromEPSG(epsg_utm);

    // Initializes the mathematical engine for coordinate transformation and reprojection
    OGRCoordinateTransformation* transformer = OGRCreateCoordinateTransformation(&source_srs, &target_srs);
    if (!transformer) {
        std::cerr << "Error: Failed to create the UTM coordinate transformer." << std::endl;
        GDALClose(dataset);
        exit(1);
    }

    std::cout << "[Processor] Analyzing and extracting layers from config.json..." << std::endl;
    
    // Lambda function for batch processing, spatial reprojection, and OGR layer diagnostics
    auto extract_layers_with_diagnostic = [&](const std::vector<std::string>& classes, std::vector<OGRGeometry*>& target_vector, const std::string& layer_type_name) {
        for (const auto& class_name : classes) {
            OGRLayer* layer = dataset->GetLayerByName(class_name.c_str());
            if (!layer) {
                std::cout << "  [WARNING] " << layer_type_name << " class ('" << class_name << "') does NOT exist in this chart." << std::endl;
                continue;
            }
            
            int total_features = 0;
            OGRFeature* feature;
            layer->ResetReading();
            
            // Iteration loop over the physical features contained in the S-57 layer
            while ((feature = layer->GetNextFeature()) != nullptr) {
                OGRGeometry* original_geom = feature->GetGeometryRef();
                if (original_geom != nullptr) {
                    // Clones the geometry to decouple its lifecycle from the OGRFeature
                    OGRGeometry* cloned_geom = original_geom->clone();
                    
                    // Executes the in-place mathematical coordinate transposition to metric UTM
                    cloned_geom->transform(transformer);
                    target_vector.push_back(cloned_geom);
                    total_features++;
                }
                // Release of the consumed feature to avoid memory leaks
                OGRFeature::DestroyFeature(feature);
            }
            std::cout << "  [OK] " << layer_type_name << " class ('" << class_name << "') extracted successfully. Features: " << total_features << std::endl;
        }
    };

    // Execution of the extraction pipeline for the navigable hydrographic sub-mesh
    std::cout << "--- Extracting Navigable Areas ---" << std::endl;
    extract_layers_with_diagnostic(config.navigable_classes, result.navigable_area, "Navigable");

    // Execution of the extraction pipeline for the collision sub-mesh and land boundaries
    std::cout << "--- Extracting Obstacles / Land ---" << std::endl;
    extract_layers_with_diagnostic(config.collision_classes, result.obstacles, "Collision/Land"); 

    // Deallocation of the spatial transformation context and safe closing of the file
    OGRCoordinateTransformation::DestroyCT(transformer);
    GDALClose(dataset);
    
    return result;
}