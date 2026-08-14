/**
 * @file test_spatial_index.cpp
 * @brief Unit tests for the SpatialIndex class.
 * 
 * Validates the loading of Boost.Geometry R-Trees, spatial intersection logic
 * for the navigation mesh, distance calculations to safety margins, and target management.
 */

#include <gtest/gtest.h>
#include <gdal_priv.h>
#include <ogrsf_frmts.h>
#include <string>
#include <vector>

// Includes the class header
#include "map/spatial_index.hpp"

/**
 * @class SpatialIndexTest
 * @brief Test suite fixture for SpatialIndex operations.
 */
class SpatialIndexTest : public ::testing::Test {
protected:
    std::string vsimem_margin = "/vsimem/test_margin.shp";
    std::string vsimem_mesh = "/vsimem/test_mesh.shp";

    void SetUp() override {
        GDALAllRegister();
        create_virtual_shapefiles();
    }

    void TearDown() override {
        GDALDriver* driver = GetGDALDriverManager()->GetDriverByName("ESRI Shapefile");
        if (driver) {
            driver->Delete(vsimem_margin.c_str());
            driver->Delete(vsimem_mesh.c_str());
        }
    }

    /**
     * @brief Helper function to dynamically create a perfect square polygon using GDAL.
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

    /**
     * @brief Generates minimal valid Shapefiles in RAM to feed the SpatialIndex.
     */
    void create_virtual_shapefiles() {
        GDALDriver* driver = GetGDALDriverManager()->GetDriverByName("ESRI Shapefile");
        if (!driver) return;

        // 1. Create Margin Shapefile (A 20x20 outer boundary)
        GDALDataset* ds_margin = driver->Create(vsimem_margin.c_str(), 0, 0, 0, GDT_Unknown, nullptr);
        OGRLayer* layer_margin = ds_margin->CreateLayer("margin", nullptr, wkbPolygon, nullptr);
        
        OGRFeature* feat_margin = OGRFeature::CreateFeature(layer_margin->GetLayerDefn());
        OGRPolygon* poly_margin = create_square(0.0, 0.0, 20.0);
        feat_margin->SetGeometry(poly_margin);
        EXPECT_EQ(layer_margin->CreateFeature(feat_margin), OGRERR_NONE);
        
        OGRFeature::DestroyFeature(feat_margin);
        OGRGeometryFactory::destroyGeometry(poly_margin);
        GDALClose(ds_margin);

        // 2. Create Mesh Shapefile (A 10x10 navigable area strictly inside the margin)
        GDALDataset* ds_mesh = driver->Create(vsimem_mesh.c_str(), 0, 0, 0, GDT_Unknown, nullptr);
        OGRLayer* layer_mesh = ds_mesh->CreateLayer("mesh", nullptr, wkbPolygon, nullptr);
        
        OGRFeature* feat_mesh = OGRFeature::CreateFeature(layer_mesh->GetLayerDefn());
        
        // Creating a triangle representing a safe navigable area from (5,5) to (15,5) to (5,15)
        OGRPolygon* poly_mesh = (OGRPolygon*)OGRGeometryFactory::createGeometry(wkbPolygon);
        OGRLinearRing ring;
        ring.addPoint(5.0, 5.0);
        ring.addPoint(15.0, 5.0);
        ring.addPoint(5.0, 15.0);
        ring.closeRings();
        poly_mesh->addRing(&ring);

        feat_mesh->SetGeometry(poly_mesh);
        EXPECT_EQ(layer_mesh->CreateFeature(feat_mesh), OGRERR_NONE);
        
        OGRFeature::DestroyFeature(feat_mesh);
        OGRGeometryFactory::destroyGeometry(poly_mesh);
        GDALClose(ds_mesh);
    }
};

// ==============================================================================
// 1. TESTS FOR: load_shapefiles
// ==============================================================================

/**
 * @brief Validates failure handling for missing shapefiles.
 */
TEST_F(SpatialIndexTest, LoadShapefiles_HandlesMissingFiles) {
    SpatialIndex index;
    // Should return safely and print errors internally, rather than crashing
    bool result = index.load_shapefiles("invalid_path.shp", "another_invalid_path.shp");
    
    // The current implementation returns true even if datasets fail to load, 
    // but the internal R-Trees should remain empty.
    EXPECT_TRUE(result); 
}

/**
 * @brief Validates successful loading and parsing of valid shapefiles.
 */
TEST_F(SpatialIndexTest, LoadShapefiles_PopulatesRTrees) {
    SpatialIndex index;
    bool result = index.load_shapefiles(vsimem_margin, vsimem_mesh);
    
    EXPECT_TRUE(result);
    // Since the Trees are private, indirectly verify they loaded by 
    // checking if a valid distance calculation can be made.
    EXPECT_NE(index.calculate_margin_distance(types::Point(10.0, 10.0)), -1.0);
}

// ==============================================================================
// 2. TESTS FOR: calculate_margin_distance
// ==============================================================================

/**
 * @brief Validates that distance calculation gracefully fails when R-Tree is empty.
 */
TEST_F(SpatialIndexTest, CalculateMarginDistance_EmptyTreeReturnsNegative) {
    SpatialIndex index; // No shapefiles loaded
    double dist = index.calculate_margin_distance(types::Point(0.0, 0.0));
    EXPECT_DOUBLE_EQ(dist, -1.0);
}

/**
 * @brief Validates exact distance calculation using Boost.Geometry.
 */
TEST_F(SpatialIndexTest, CalculateMarginDistance_ReturnsExactDistance) {
    SpatialIndex index;
    index.load_shapefiles(vsimem_margin, vsimem_mesh);

    // Our margin is a square from (0,0) to (20,20). 
    // The closest boundary to point (2.0, 10.0) is the left wall at X=0.0.
    // The exact distance should be 2.0 meters.
    types::Point test_distance_pt(2.0, 10.0);
    double dist = index.calculate_margin_distance(test_distance_pt);
    
    EXPECT_NEAR(dist, 2.0, 0.001); // Asserts precision to the millimeter
}

// ==============================================================================
// 3. TESTS FOR: is_navigable
// ==============================================================================

/**
 * @brief Validates that navigability defaults to false if the R-Tree is empty.
 */
TEST_F(SpatialIndexTest, IsNavigable_EmptyTreeReturnsFalse) {
    SpatialIndex index; // No shapefiles loaded
    EXPECT_FALSE(index.is_navigable(types::Point(10.0, 10.0)));
}

/**
 * @brief Validates accurate point-in-polygon detection for the NavMesh.
 */
TEST_F(SpatialIndexTest, IsNavigable_CorrectlyIdentifiesSafeAndDangerZones) {
    SpatialIndex index;
    index.load_shapefiles(vsimem_margin, vsimem_mesh);

    // Point inside the 10x10 virtual triangle created in RAM
    types::Point safe_point(7.0, 7.0); 
    EXPECT_TRUE(index.is_navigable(safe_point));

    // Point outside the mesh triangle
    types::Point dangerous_point(1.0, 1.0);
    EXPECT_FALSE(index.is_navigable(dangerous_point));
}

// ==============================================================================
// 4. TESTS FOR: export_rtree_debug
// ==============================================================================

/**
 * @brief Validates the GDAL drawing logic correctly generates the 3 debug shapefiles in RAM.
 */
TEST_F(SpatialIndexTest, ExportRtreeDebug_GeneratesDebugShapefiles) {
    SpatialIndex index;
    index.load_shapefiles(vsimem_margin, vsimem_mesh);

    std::vector<types::Point> test_points = { types::Point(7.0, 7.0) };
    std::string debug_folder = "/vsimem"; 
    
    index.export_rtree_debug(test_points, debug_folder, 32723);

    // Validate that the files were physically (virtually) created by GDAL
    GDALDataset* ds_mesh = (GDALDataset*)GDALOpenEx("/vsimem/97_RTree_Caixas_Malha.shp", GDAL_OF_VECTOR, nullptr, nullptr, nullptr);
    GDALDataset* ds_margin = (GDALDataset*)GDALOpenEx("/vsimem/98_RTree_Caixas_Margem.shp", GDAL_OF_VECTOR, nullptr, nullptr, nullptr);
    GDALDataset* ds_lines = (GDALDataset*)GDALOpenEx("/vsimem/96_RTree_Distancia.shp", GDAL_OF_VECTOR, nullptr, nullptr, nullptr);

    // Assertions: All datasets must exist
    ASSERT_NE(ds_mesh, nullptr);
    ASSERT_NE(ds_margin, nullptr);
    ASSERT_NE(ds_lines, nullptr);

    // Cleanup memory and files
    GDALClose(ds_mesh);
    GDALClose(ds_margin);
    GDALClose(ds_lines);

    GDALDriver* driver = GetGDALDriverManager()->GetDriverByName("ESRI Shapefile");
    if (driver) {
        driver->Delete("/vsimem/97_RTree_Caixas_Malha.shp");
        driver->Delete("/vsimem/98_RTree_Caixas_Margem.shp");
        driver->Delete("/vsimem/96_RTree_Distancia.shp");
    }
}

// ==============================================================================
// 5. TESTS FOR: get_closest_static_obstacle_distance
// ==============================================================================

/**
 * @brief Validates the public wrapper for obstacle distance.
 */
TEST_F(SpatialIndexTest, GetClosestStaticObstacleDistance_ReturnsCorrectFloat) {
    SpatialIndex index;
    index.load_shapefiles(vsimem_margin, vsimem_mesh);

    types::Point test_pt(2.0, 10.0);
    
    float dist = index.get_closest_static_obstacle_distance(test_pt);
    EXPECT_FLOAT_EQ(dist, 2.0f);
}

// ==============================================================================
// 6. TESTS FOR: is_inside_restricted_zone
// ==============================================================================

/**
 * @brief Validates the public wrapper properly inverts the navigability logic.
 */
TEST_F(SpatialIndexTest, IsInsideRestrictedZone_InvertsNavigability) {
    SpatialIndex index;
    index.load_shapefiles(vsimem_margin, vsimem_mesh);

    types::Point safe_point(7.0, 7.0);     // Inside mesh
    types::Point danger_point(1.0, 1.0);   // Outside mesh

    EXPECT_FALSE(index.is_inside_restricted_zone(safe_point));
    EXPECT_TRUE(index.is_inside_restricted_zone(danger_point));
}

// ==============================================================================
// 7. TESTS FOR: update_global_targets & get_active_local_targets
// ==============================================================================

/**
 * @brief Validates the storage and spatial filtering of targets via distance radius.
 * Ensures the pure C++ math logic correctly calculates distances and filters out 
 * targets that fall beyond the specified spatial boundary.
 */
TEST_F(SpatialIndexTest, TargetsManagement_FiltersByRadius) {
    SpatialIndex index;
    std::vector<types::Target> global_targets;
    
    // Create dummy dependencies to satisfy Target's strict constructor requirements
    types::Velocity dummy_vel(0.0, 0.0, 0.0);
    types::Kinematics dummy_kin(dummy_vel);
    types::Covariance dummy_cov(0.0, 0.0, 0.0);
    
    // 2. Instantiate Poses using the direct coordinate constructor
    // t1 is ~14.1 meters away from origin (sqrt(10^2 + 10^2))
    types::Pose pose1(10.0, 10.0, 0.0); 
    // t2 is ~141.4 meters away from origin (sqrt(100^2 + 100^2))
    types::Pose pose2(100.0, 100.0, 0.0); 

    // Construct the targets with the full required signature
    types::Target t1(1, "Target1", pose1, dummy_kin, dummy_cov, 1.0);
    types::Target t2(2, "Target2", pose2, dummy_kin, dummy_cov, 1.0);

    global_targets.push_back(t1);
    global_targets.push_back(t2);

    // Execute Logic: Update cache and search within a 20-meter radius
    index.update_global_targets(global_targets);
    types::Point search_center(0.0, 0.0, 0.0);
    auto local_targets = index.get_active_local_targets(search_center, 20.0f);

    // Validations
    ASSERT_EQ(local_targets.size(), 1); // Only t1 should be found within 20m
    
    // Validate if the returned target is indeed the one at X = 10.0
    EXPECT_DOUBLE_EQ(local_targets[0].get_pose().get_x(), 10.0);
}