/**
 * @file test_vector_visualizer.cpp
 * @brief Unit tests for the VectorVisualizer data extraction logic.
 * 
 * Focuses on validating the translation of GDAL geometries into 
 * OpenGL-friendly 2D structures (Polygon2D, Vertex2D) specifically 
 * supporting complex shapes with interior holes for Stencil Buffer rendering.
 */

#include <gtest/gtest.h>
#include <ogr_geometry.h>
#include <vector>

// Includes the class header
#include "map/vector_visualizer.hpp"

class VectorVisualizerTest : public ::testing::Test {
protected:
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
// 1. TESTS FOR: extract_geom_polygons
// ==============================================================================

/**
 * @brief Validates null pointer protection.
 */
TEST_F(VectorVisualizerTest, ExtractPolygons_HandlesNullPointer) {
    std::vector<Polygon2D> target_list;
    
    // Should not crash
    extract_geom_polygons(nullptr, target_list);
    
    EXPECT_TRUE(target_list.empty());
}

/**
 * @brief Validates extraction of a simple, solid polygon without holes.
 */
TEST_F(VectorVisualizerTest, ExtractPolygons_SimplePolygon) {
    OGRPolygon* poly = create_square(0.0, 0.0, 10.0);
    std::vector<Polygon2D> target_list;

    extract_geom_polygons(poly, target_list);

    // Assert the list contains exactly 1 Polygon2D object
    ASSERT_EQ(target_list.size(), 1);
    
    // GDAL requires 5 points for a square (closes the ring back to the first point)
    EXPECT_EQ(target_list[0].outer_ring.size(), 5);
    
    // There should be no holes
    EXPECT_TRUE(target_list[0].holes.empty());

    OGRGeometryFactory::destroyGeometry(poly);
}

/**
 * @brief Validates extraction of a complex polygon with an interior hole (Donut shape).
 * Essential for verifying that the Stencil Buffer rendering will receive correct data.
 */
TEST_F(VectorVisualizerTest, ExtractPolygons_PolygonWithHoles) {
    OGRPolygon* poly = (OGRPolygon*)OGRGeometryFactory::createGeometry(wkbPolygon);
    
    // 1. Create the outer boundary (10x10 square)
    OGRLinearRing ext;
    ext.addPoint(0.0, 0.0);
    ext.addPoint(10.0, 0.0);
    ext.addPoint(10.0, 10.0);
    ext.addPoint(0.0, 10.0);
    ext.closeRings();
    poly->addRing(&ext);

    // 2. Create an internal hole (4x4 square in the middle, representing an island)
    OGRLinearRing hole;
    hole.addPoint(3.0, 3.0);
    hole.addPoint(7.0, 3.0);
    hole.addPoint(7.0, 7.0);
    hole.addPoint(3.0, 7.0);
    hole.closeRings();
    poly->addRing(&hole); // Adds as an interior ring

    std::vector<Polygon2D> target_list;
    extract_geom_polygons(poly, target_list);

    ASSERT_EQ(target_list.size(), 1);
    
    // Validate the outer perimeter
    EXPECT_EQ(target_list[0].outer_ring.size(), 5);
    
    // Validate that the extraction logic successfully identified the hole
    ASSERT_EQ(target_list[0].holes.size(), 1);
    EXPECT_EQ(target_list[0].holes[0].size(), 5);

    OGRGeometryFactory::destroyGeometry(poly);
}

/**
 * @brief Validates recursion inside Geometry Collections (MultiPolygons).
 */
TEST_F(VectorVisualizerTest, ExtractPolygons_UnpacksMultiPolygon) {
    OGRMultiPolygon* multi = (OGRMultiPolygon*)OGRGeometryFactory::createGeometry(wkbMultiPolygon);
    multi->addGeometryDirectly(create_square(0.0, 0.0, 5.0));
    multi->addGeometryDirectly(create_square(20.0, 20.0, 5.0));

    std::vector<Polygon2D> target_list;
    extract_geom_polygons(multi, target_list);

    // The function must recursively extract both squares into separate Polygon2D structs
    EXPECT_EQ(target_list.size(), 2);

    OGRGeometryFactory::destroyGeometry(multi);
}