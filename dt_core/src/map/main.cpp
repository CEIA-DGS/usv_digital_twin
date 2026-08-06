#include <iostream>
#include <string>
#include <filesystem>
#include "../../include/map/gdal_initializer.hpp"
#include "../../include/map/config_manager.hpp"
#include "../../include/map/s57_processor.hpp"
#include "../../include/map/mesh_generator.hpp"
#include "../../include/map/vector_exporter.hpp"
#include "../../include/map/vector_visualizer.hpp"
#include "../../include/map/spatial_index.hpp"
#include "dt_core/types.hpp"
#include "gdal_priv.h"

namespace fs = std::filesystem;

int main(int argc, char* argv[]) {
    try {
        if (argc < 2) {
            std::cerr << "Correct usage: " << argv[0] << " <path_to_chart.039>\n";
            return 1;
        }

        std::string s57_chart_path = argv[1];
        std::string chart_name = fs::path(s57_chart_path).stem().string();
        std::string output_directory = "../../data/output/NavMesh_Shapefiles_" + chart_name;

        // Global initialization
        GdalInitializer::initialize();
        std::cout << "[System] Processing nautical chart: " << s57_chart_path << std::endl;

        // S-57 Processing
        MapConfiguration config = ConfigManager::load_configuration("../config.json");
        ProcessedGeometries geometries = S57Processor::process_chart(s57_chart_path, config);
        
        // Extracts the calculated EPSG from the processed structure
        int epsg_utm = geometries.dynamic_utm_epsg;

        // NavMesh Generation
        NavigationMesh nav_mesh = MeshGenerator::generate(geometries, config.safety_margin, config.simplification_tolerance);

        // Export and Visualization
        if (fs::exists(output_directory)) fs::remove_all(output_directory);
        VectorExporter::export_shapefile(nav_mesh, output_directory, epsg_utm);
        VectorVisualizer::display(output_directory, chart_name);

        // Loading Spatial Engine for Tests
        SpatialIndex spatial_engine;
        spatial_engine.load_shapefiles(output_directory + "/2_Margem_Seguranca.shp", output_directory + "/4_Malha_NavMesh.shp");

        // Quick test using the geometric center of the mesh
        GDALDataset* mesh_ds = (GDALDataset*)GDALOpenEx((output_directory + "/4_Malha_NavMesh.shp").c_str(), GDAL_OF_VECTOR, nullptr, nullptr, nullptr);
        OGREnvelope mesh_env;
        if (mesh_ds) {
            (void)mesh_ds->GetLayer(0)->GetExtent(&mesh_env);
            GDALClose(mesh_ds);
        }
        
        types::Point test_point((mesh_env.MinX + mesh_env.MaxX) / 2.0, (mesh_env.MinY + mesh_env.MaxY) / 2.0, 0.0);
        
        std::cout << "\n-> Distance to static obstacle: " << spatial_engine.get_closest_static_obstacle_distance(test_point) << " m.\n";
        std::cout << "-> Is the point in a restricted zone? " << (spatial_engine.is_inside_restricted_zone(test_point) ? "YES" : "NO") << "\n";

        // Test of dynamic targets / nearby boats within a range radius (e.g., 500m)
        std::vector<types::Target> simulated_targets = {
            types::Target(1, "Nearby_Boat (30m)",    types::Pose(test_point.get_x() + 20.0,  test_point.get_y() + 20.0,  0.0), types::Kinematics(types::Velocity())),
            types::Target(2, "Mid_Range_Boat (250m)",     types::Pose(test_point.get_x() + 200.0, test_point.get_y() + 150.0, 0.0), types::Kinematics(types::Velocity())),
            types::Target(3, "Distant_Boat (1.2km)", types::Pose(test_point.get_x() + 1000.0, test_point.get_y() + 800.0, 0.0), types::Kinematics(types::Velocity()))
        };
        spatial_engine.update_global_targets(simulated_targets);

        float search_radius_meters = 500.0f;
        auto local_targets = spatial_engine.get_active_local_targets(test_point, search_radius_meters);
        
        std::cout << "-> Targets detected within a radius of " << search_radius_meters << "m: " << local_targets.size() << "\n";
        for (const auto& target : local_targets) {
            std::cout << "   * [Local Target] " << target.get_description() << " (ID: " << target.get_id() << ")\n";
        }

        // Memory cleanup
        geometries.free_memory();
        nav_mesh.free_memory();

        std::cout << "\n[System] Execution finished successfully!\n";
    } catch (const std::exception& e) {
        std::cerr << "\n[Critical Error] " << e.what() << std::endl;
        return 1;
    }
    return 0;
}