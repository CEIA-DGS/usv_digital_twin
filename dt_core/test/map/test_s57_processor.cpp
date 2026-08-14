/**
 * @file test_s57_processor.cpp
 * @brief Unit tests for the S57Processor class.
 * 
 * Validates the reading of nautical charts, dynamic UTM zone calculation,
 * and the spatial reprojection of geographic coordinates to metric systems.
 */

#include <gtest/gtest.h>
#include <gdal_priv.h>
#include <ogrsf_frmts.h>
#include <string>

// Includes the class header
#include "map/s57_processor.hpp"

/**
 * @class S57ProcessorTest
 * @brief Test suite fixture for S57Processor operations.
 * 
 * Utilizes GDAL's Virtual File System (/vsimem/) to create dummy nautical
 * charts entirely in RAM, avoiding physical disk I/O and dependency on real S-57 files.
 */
class S57ProcessorTest : public ::testing::Test {
protected:
    std::string valid_vsimem_path = "/vsimem/dummy_chart.gpkg"; ///< Virtual path in RAM
    std::string missing_file_path = "non_existent_chart.000";   ///< Deliberately wrong path

    /**
     * @brief Setup phase. Registers GDAL drivers and builds the dummy chart.
     */
    void SetUp() override {
        GDALAllRegister();
        create_dummy_dataset();
    }

    /**
     * @brief Teardown phase. Unlinks (deletes) the virtual file from RAM.
     */
    void TearDown() override {
        VSIUnlink(valid_vsimem_path.c_str());
    }

    /**
     * @brief Generates an in-memory GeoPackage containing standard S-57 layer names.
     * Uses coordinates located in Brazil (Lon: -45.0, Lat: -20.0) which should
     * mathematically resolve to UTM Zone 23S (EPSG: 32723).
     */
    void create_dummy_dataset() {
        GDALDriver* driver = GetGDALDriverManager()->GetDriverByName("GPKG");
        if (!driver) return; // Fallback in case GeoPackage driver is missing

        // Creates the dataset in RAM
        GDALDataset* ds = driver->Create(valid_vsimem_path.c_str(), 0, 0, 0, GDT_Unknown, nullptr);

        OGRSpatialReference srs;
        srs.importFromEPSG(4326); // WGS84 (Geographic)

        // --- Create Layer 1: DEPARE (Navigable Water) ---
        OGRLayer* depare = ds->CreateLayer("DEPARE", &srs, wkbPolygon, nullptr);
        OGRFeature* feat1 = OGRFeature::CreateFeature(depare->GetLayerDefn());
        
        OGRPolygon poly1;
        OGRLinearRing ring1;
        ring1.addPoint(-45.0, -20.0);
        ring1.addPoint(-45.0, -19.0);
        ring1.addPoint(-44.0, -19.0);
        ring1.addPoint(-44.0, -20.0);
        ring1.closeRings();
        poly1.addRing(&ring1);
        
        feat1->SetGeometry(&poly1);
        EXPECT_EQ(depare->CreateFeature(feat1), OGRERR_NONE);
        OGRFeature::DestroyFeature(feat1);

        // --- Create Layer 2: LNDARE (Land/Obstacle) ---
        OGRLayer* lndare = ds->CreateLayer("LNDARE", &srs, wkbPolygon, nullptr);
        OGRFeature* feat2 = OGRFeature::CreateFeature(lndare->GetLayerDefn());
        
        OGRPolygon poly2;
        OGRLinearRing ring2;
        ring2.addPoint(-45.5, -20.5);
        ring2.addPoint(-45.5, -19.5);
        ring2.addPoint(-44.5, -19.5);
        ring2.addPoint(-44.5, -20.5);
        ring2.closeRings();
        poly2.addRing(&ring2);
        
        feat2->SetGeometry(&poly2);
        EXPECT_EQ(lndare->CreateFeature(feat2), OGRERR_NONE);
        OGRFeature::DestroyFeature(feat2);

        GDALClose(ds); // Flushes data to the virtual file
    }
};

// ==============================================================================
// 1. TESTS FOR: process_chart
// ==============================================================================

/**
 * @brief Validates the crash protection for missing or unreadable files.
 */
TEST_F(S57ProcessorTest, HandlesMissingChartWithExit) {
    MapConfiguration config;
    // Expects the program to abort and print the error pattern
    EXPECT_DEATH(S57Processor::process_chart(missing_file_path, config), ".*Error:.*");
}

/**
 * @brief Validates the mathematical calculation of the dynamic UTM Zone.
 * Coordinates at -45.0 Longitude and -20.0 Latitude strictly belong to UTM 23S.
 */
TEST_F(S57ProcessorTest, CalculatesCorrectUTMZone) {
    MapConfiguration config;
    config.navigable_classes = {"DEPARE"};
    config.collision_classes = {"LNDARE"};

    ProcessedGeometries result = S57Processor::process_chart(valid_vsimem_path, config);

    // EPSG 32723 = South Hemisphere (327xx) + Zone 23 (23)
    EXPECT_EQ(result.dynamic_utm_epsg, 32723);

    // Cleanup
    for (auto* geom : result.navigable_area) OGRGeometryFactory::destroyGeometry(geom);
    for (auto* geom : result.obstacles) OGRGeometryFactory::destroyGeometry(geom);
}

/**
 * @brief Validates the extraction of layers and spatial metric reprojection.
 * Checks if geographic bounds (degrees) were converted to metric bounds (meters).
 */
TEST_F(S57ProcessorTest, ExtractsAndReprojectsLayers) {
    MapConfiguration config;
    config.navigable_classes = {"DEPARE"};
    config.collision_classes = {"LNDARE"};

    ProcessedGeometries result = S57Processor::process_chart(valid_vsimem_path, config);

    // Validates that it found exactly 1 feature per requested layer
    ASSERT_EQ(result.navigable_area.size(), 1);
    ASSERT_EQ(result.obstacles.size(), 1);

    // Validates the Reprojection (Degrees -> Meters)
    OGRPolygon* nav_poly = result.navigable_area[0]->toPolygon();
    OGRLinearRing* ext_ring = nav_poly->getExteriorRing();
    
    ASSERT_NE(ext_ring, nullptr);
    
    // In Geographic (EPSG:4326), the X coordinate was -45.0. 
    // In UTM Metric (EPSG:32723), the X coordinate should be in the hundreds of thousands (e.g. 500000 m).
    // This strictly proves the `cloned_geom->transform(transformer);` line worked.
    double metric_x = ext_ring->getX(0);
    EXPECT_GT(metric_x, 1000.0); 
    EXPECT_NE(metric_x, -45.0);

    // Cleanup
    for (auto* geom : result.navigable_area) OGRGeometryFactory::destroyGeometry(geom);
    for (auto* geom : result.obstacles) OGRGeometryFactory::destroyGeometry(geom);
}

/**
 * @brief Validates diagnostic resilience when requested classes are missing from the chart.
 * Assures the pipeline doesn't crash, but skips nonexistent layers safely.
 */
TEST_F(S57ProcessorTest, IgnoresMissingConfigClasses) {
    MapConfiguration config;
    // Request a class that doesn't exist in our dummy dataset
    config.navigable_classes = {"UNKNOWN_RIVER"};
    config.collision_classes = {"GHOST_ISLAND"};

    ProcessedGeometries result = S57Processor::process_chart(valid_vsimem_path, config);

    // The system should survive and return empty vectors
    EXPECT_TRUE(result.navigable_area.empty());
    EXPECT_TRUE(result.obstacles.empty());
}