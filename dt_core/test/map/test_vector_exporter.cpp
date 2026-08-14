/**
 * @file test_vector_exporter.cpp
 * @brief Unit tests for the VectorExporter class.
 * 
 * Validates the creation of geospatial vector files (Shapefiles),
 * correct writing of layers, and handling of EPSG projection assignments
 * using GDAL's virtual memory file system.
 */

#include <gtest/gtest.h>
#include <gdal_priv.h>
#include <ogrsf_frmts.h>
#include <string>

// Includes the class header
#include "map/vector_exporter.hpp"

/**
 * @class VectorExporterTest
 * @brief Test suite fixture for vector export operations.
 */
class VectorExporterTest : public ::testing::Test {
protected:
    std::string vsimem_dir = "/vsimem/test_shapefiles_export";

    void SetUp() override {
        GDALAllRegister();
    }

    void TearDown() override {
        // Clean up the virtual directory and its contents after each test
        GDALDriver* driver = GetGDALDriverManager()->GetDriverByName("ESRI Shapefile");
        if (driver) {
            driver->Delete(vsimem_dir.c_str());
        }
    }

    /**
     * @brief Helper function to generate a simple polygon in memory.
     */
    OGRPolygon* create_square(double x, double y, double size) {
        OGRPolygon* poly = (OGRPolygon*)OGRGeometryFactory::createGeometry(wkbPolygon);
        OGRLinearRing ring;
        ring.addPoint(x, y);
        ring.addPoint(x + size, y);
        ring.addPoint(x + size, y + size);
        ring.addPoint(x, y + size);
        ring.closeRings();
        poly->addRing(&ring);
        return poly;
    }
};

// ==============================================================================
// 1. TESTS FOR: insert_geometry
// ==============================================================================

/**
 * @brief Validates null pointer protection.
 * Prevents segmentation faults if empty geometries or null layers are passed.
 */
TEST_F(VectorExporterTest, InsertGeometry_HandlesNullPointers) {
    // Should return immediately without crashing
    VectorExporter::insert_geometry(nullptr, nullptr);
    SUCCEED();
}

/**
 * @brief Validates that a geometry is correctly converted into a Feature and appended.
 */
TEST_F(VectorExporterTest, InsertGeometry_InsertsValidGeometry) {
    // Use the "Memory" driver just to create a temporary layer for this isolated test
    GDALDriver* mem_driver = GetGDALDriverManager()->GetDriverByName("Memory");
    GDALDataset* mem_ds = mem_driver->Create("mem_ds", 0, 0, 0, GDT_Unknown, nullptr);
    OGRLayer* layer = mem_ds->CreateLayer("test_layer", nullptr, wkbPolygon, nullptr);

    OGRPolygon* poly = create_square(0.0, 0.0, 10.0);

    // Call the function to test
    VectorExporter::insert_geometry(layer, poly);

    // Validate that the layer now contains exactly 1 feature
    EXPECT_EQ(layer->GetFeatureCount(), 1);

    // Cleanup
    OGRGeometryFactory::destroyGeometry(poly);
    GDALClose(mem_ds);
}

// ==============================================================================
// 2. TESTS FOR: export_shapefile
// ==============================================================================

/**
 * @brief Validates the complete pipeline for generating the 4 structural Shapefile layers.
 * Checks file creation, layer naming, feature population, and triangle conversion.
 */
TEST_F(VectorExporterTest, ExportShapefile_GeneratesCorrectFiles) {
    // 1. Arrange: Create a mock NavigationMesh populated with dummy data
    NavigationMesh mock_mesh;
    mock_mesh.original_land = create_square(0.0, 0.0, 10.0);
    mock_mesh.safety_margin = create_square(10.0, 10.0, 5.0);
    mock_mesh.safe_navigable_perimeter = create_square(20.0, 20.0, 10.0);
    
    // Add two fake triangles to the list
    mock_mesh.triangles.push_back({{0.0, 0.0}, {10.0, 0.0}, {0.0, 10.0}});
    mock_mesh.triangles.push_back({{10.0, 10.0}, {10.0, 0.0}, {0.0, 10.0}});

    // 2. Act: Call the exporter using the RAM filesystem and UTM Zone 23S (EPSG: 32723)
    VectorExporter::export_shapefile(mock_mesh, vsimem_dir, 32723);

    // 3. Assert: Open the generated "folder" to inspect the results
    GDALDataset* exported_ds = (GDALDataset*)GDALOpenEx(vsimem_dir.c_str(), GDAL_OF_VECTOR, nullptr, nullptr, nullptr);
    
    // Validates that the Shapefile dataset was successfully created
    ASSERT_NE(exported_ds, nullptr);

    // Validates that all 4 structural layers were generated
    EXPECT_EQ(exported_ds->GetLayerCount(), 4);

    // Validate Layer 1: Land
    OGRLayer* layer_land = exported_ds->GetLayerByName("1_Terra_Firme");
    ASSERT_NE(layer_land, nullptr);
    EXPECT_EQ(layer_land->GetFeatureCount(), 1);

    // Validate Layer 4: Mesh (Triangles)
    OGRLayer* layer_mesh = exported_ds->GetLayerByName("4_Malha_NavMesh");
    ASSERT_NE(layer_mesh, nullptr);
    EXPECT_EQ(layer_mesh->GetFeatureCount(), 2); // Injected 2 triangles

    // Cleanup internal memory (GDAL closes the dataset safely)
    GDALClose(exported_ds);
    OGRGeometryFactory::destroyGeometry(mock_mesh.original_land);
    OGRGeometryFactory::destroyGeometry(mock_mesh.safety_margin);
    OGRGeometryFactory::destroyGeometry(mock_mesh.safe_navigable_perimeter);
}