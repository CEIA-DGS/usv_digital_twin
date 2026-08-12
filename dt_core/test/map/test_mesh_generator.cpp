/**
 * @file test_mesh_generator.cpp
 * @brief Unit tests for the MeshGenerator class and its helper functions.
 * 
 * Validates geometric transformations, polygon simplifications, boolean operations,
 * and Delaunay triangulation algorithms used for NavMesh generation.
 */

#include <gtest/gtest.h>
#include <ogr_geometry.h>
#include <poly2tri/poly2tri.h>
#include <vector>
#include "map/mesh_generator.hpp"

std::vector<p2t::Point*> extract_clean_contour(OGRLinearRing* ring);
OGRGeometry* ensure_polygon(OGRGeometry* geom);

/**
 * @class MeshGeneratorTest
 * @brief Test suite fixture for MeshGenerator operations.
 */
class MeshGeneratorTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Initialization if needed for GDAL/OGR components
    }

    void TearDown() override {
        // Cleanup global states if necessary
    }

    /**
     * @brief Helper function to dynamically create a perfect square polygon using GDAL.
     * @param x Bottom-left X coordinate.
     * @param y Bottom-left Y coordinate.
     * @param size Length of the square's sides.
     * @return OGRPolygon* Pointer to the generated polygon in memory.
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
// 1. TESTS FOR: extract_clean_contour
// ==============================================================================

/**
 * @brief Validates the safety check for null pointers.
 * Expects an empty vector return rather than a segmentation fault.
 */
TEST_F(MeshGeneratorTest, ExtractCleanContour_HandlesNullPointer) {
    std::vector<p2t::Point*> contour = extract_clean_contour(nullptr);
    EXPECT_TRUE(contour.empty());
}

/**
 * @brief Validates rejection of geometrically invalid rings.
 * A linear ring must have at least 4 points to form a closed polygon in GDAL
 * (e.g., triangle requires 3 vertices + 1 closing vertex).
 */
TEST_F(MeshGeneratorTest, ExtractCleanContour_RejectsIncompleteRing) {
    OGRLinearRing ring;
    ring.addPoint(0.0, 0.0);
    ring.addPoint(10.0, 0.0);
    // Only 2 points provided (invalid ring)
    
    std::vector<p2t::Point*> contour = extract_clean_contour(&ring);
    EXPECT_TRUE(contour.empty());
}

/**
 * @brief Validates the correct extraction of a perfect square.
 * Ensures the closing duplicate point mandated by GDAL is removed for poly2tri.
 */
TEST_F(MeshGeneratorTest, ExtractCleanContour_ValidSquare) {
    OGRLinearRing ring;
    ring.addPoint(0.0, 0.0);
    ring.addPoint(10.0, 0.0);
    ring.addPoint(10.0, 10.0);
    ring.addPoint(0.0, 10.0);
    ring.closeRings(); // Automatically adds (0.0, 0.0) at the end, making it 5 points

    std::vector<p2t::Point*> contour = extract_clean_contour(&ring);

    // poly2tri needs exactly 4 unique points for a square
    ASSERT_EQ(contour.size(), 4);
    
    // Check if coordinates were copied correctly
    EXPECT_DOUBLE_EQ(contour[0]->x, 0.0);
    EXPECT_DOUBLE_EQ(contour[0]->y, 0.0);
    EXPECT_DOUBLE_EQ(contour[2]->x, 10.0);
    EXPECT_DOUBLE_EQ(contour[2]->y, 10.0);

    // Free the dynamically allocated points to prevent memory leaks
    for (auto* p : contour) delete p;
}

/**
 * @brief Validates the epsilon filter (0.1 tolerance).
 * Redundant points that are too close to the previous point should be discarded.
 */
TEST_F(MeshGeneratorTest, ExtractCleanContour_FiltersRedundantPoints) {
    OGRLinearRing ring;
    ring.addPoint(0.0, 0.0);
    ring.addPoint(0.05, 0.05); // Redundant: distance < epsilon (0.1) from previous
    ring.addPoint(10.0, 0.0);
    ring.addPoint(10.0, 10.0);
    ring.addPoint(0.0, 10.0);
    ring.closeRings();

    std::vector<p2t::Point*> contour = extract_clean_contour(&ring);

    // The redundant point (0.05, 0.05) should be ignored
    ASSERT_EQ(contour.size(), 4);
    
    for (auto* p : contour) delete p;
}

// ==============================================================================
// 2 TESTS FOR: ensure_polygon
// ==============================================================================

/**
 * @brief Validates null pointer protection.
 */
TEST_F(MeshGeneratorTest, EnsurePolygon_HandlesNullPointer) {
    OGRGeometry* result = ensure_polygon(nullptr);
    EXPECT_EQ(result, nullptr);
}

/**
 * @brief Validates that a valid Polygon passes through unharmed (just cloned).
 */
TEST_F(MeshGeneratorTest, EnsurePolygon_KeepsValidPolygon) {
    OGRPolygon* poly = create_square(0.0, 0.0, 5.0);
    
    OGRGeometry* result = ensure_polygon(poly);
    
    ASSERT_NE(result, nullptr);
    EXPECT_EQ(wkbFlatten(result->getGeometryType()), wkbPolygon);
    
    OGRGeometryFactory::destroyGeometry(poly);
    OGRGeometryFactory::destroyGeometry(result);
}

/**
 * @brief Validates the fallback shield (Buffer 1.0) for invalid geometries like Lines.
 * If the system accidentally feeds a LineString (e.g. a boat track), it should 
 * inflate it by 1 meter to force it to become a solid Polygon obstacle.
 */
TEST_F(MeshGeneratorTest, EnsurePolygon_ConvertsLineToPolygon) {
    // Creates a simple line instead of a polygon
    OGRLineString* line = (OGRLineString*)OGRGeometryFactory::createGeometry(wkbLineString);
    line->addPoint(0.0, 0.0);
    line->addPoint(10.0, 0.0);
    
    // This should trigger the "return geom->Buffer(1.0)" logic
    OGRGeometry* result = ensure_polygon(line);
    
    ASSERT_NE(result, nullptr);
    // The line must have been transformed into a polygon by the buffer!
    EXPECT_EQ(wkbFlatten(result->getGeometryType()), wkbPolygon);
    
    OGRGeometryFactory::destroyGeometry(line);
    OGRGeometryFactory::destroyGeometry(result);
}

// ==============================================================================
// 3. TESTS FOR: filter_by_area
// ==============================================================================

/**
 * @brief Validates null pointer protection.
 * Assures the filter doesn't crash when dealing with empty geometries.
 */
TEST_F(MeshGeneratorTest, FilterByArea_HandlesNullPointer) {
    OGRGeometry* result = MeshGenerator::filter_by_area(nullptr, 25.0);
    EXPECT_EQ(result, nullptr);
}

/**
 * @brief Validates preservation of polygons larger than the minimum area.
 */
TEST_F(MeshGeneratorTest, FilterByArea_KeepsLargePolygon) {
    OGRPolygon* large_poly = create_square(0.0, 0.0, 10.0); // Area = 100
    
    OGRGeometry* result = MeshGenerator::filter_by_area(large_poly, 25.0);
    
    ASSERT_NE(result, nullptr);
    EXPECT_EQ(wkbFlatten(result->getGeometryType()), wkbPolygon);
    
    OGRGeometryFactory::destroyGeometry(large_poly);
    OGRGeometryFactory::destroyGeometry(result);
}

/**
 * @brief Validates removal of polygons smaller than the minimum area (slivers).
 */
TEST_F(MeshGeneratorTest, FilterByArea_RemovesSmallPolygon) {
    OGRPolygon* small_poly = create_square(0.0, 0.0, 2.0); // Area = 4
    
    OGRGeometry* result = MeshGenerator::filter_by_area(small_poly, 25.0);
    
    EXPECT_EQ(result, nullptr); 
    OGRGeometryFactory::destroyGeometry(small_poly);
}

/**
 * @brief Validates extraction of valid geometries inside a MultiPolygon.
 */
TEST_F(MeshGeneratorTest, FilterByArea_FiltersMultiPolygonCorrectly) {
    OGRMultiPolygon* multi = (OGRMultiPolygon*)OGRGeometryFactory::createGeometry(wkbMultiPolygon);
    multi->addGeometryDirectly(create_square(0.0, 0.0, 10.0));  // Area 100 (Keep)
    multi->addGeometryDirectly(create_square(20.0, 20.0, 2.0)); // Area 4 (Discard)
    
    OGRGeometry* result = MeshGenerator::filter_by_area(multi, 25.0);
    
    ASSERT_NE(result, nullptr);
    ASSERT_EQ(wkbFlatten(result->getGeometryType()), wkbMultiPolygon);
    
    OGRMultiPolygon* result_multi = result->toMultiPolygon();
    EXPECT_EQ(result_multi->getNumGeometries(), 1);
    
    OGRGeometryFactory::destroyGeometry(multi);
    OGRGeometryFactory::destroyGeometry(result);
}

/**
 * @brief Validates cleanup when all sub-geometries of a MultiPolygon fail the area test.
 */
TEST_F(MeshGeneratorTest, FilterByArea_DestroysEmptyMultiPolygon) {
    OGRMultiPolygon* multi = (OGRMultiPolygon*)OGRGeometryFactory::createGeometry(wkbMultiPolygon);
    multi->addGeometryDirectly(create_square(0.0, 0.0, 2.0));   
    multi->addGeometryDirectly(create_square(20.0, 20.0, 3.0)); 
    
    OGRGeometry* result = MeshGenerator::filter_by_area(multi, 25.0);
    EXPECT_EQ(result, nullptr);
    
    OGRGeometryFactory::destroyGeometry(multi);
}

// ==============================================================================
// 4. TESTS FOR: generate
// ==============================================================================

/**
 * @brief Validates the pipeline behavior when fed with completely empty input.
 * Assures the system returns a safe, empty NavigationMesh without segfaulting.
 */
TEST_F(MeshGeneratorTest, Generate_HandlesEmptyInput) {
    ProcessedGeometries empty_geom; // Both navigable_area and obstacles are empty
    
    NavigationMesh mesh = MeshGenerator::generate(empty_geom, 5.0, 1.0);
    
    // Everything should be gracefully empty/null
    EXPECT_EQ(mesh.original_land, nullptr);
    EXPECT_EQ(mesh.safety_margin, nullptr);
    EXPECT_EQ(mesh.safe_navigable_perimeter, nullptr);
    EXPECT_TRUE(mesh.triangles.empty());
}

/**
 * @brief Validates standard mesh generation on an open sea scenario (no obstacles).
 * Ensures the basic pipeline (union -> difference -> filter -> triangulate) works.
 */
TEST_F(MeshGeneratorTest, Generate_CreatesMeshFromSimpleArea) {
    ProcessedGeometries geom;
    // Create a 100x100 open water area (Area = 10000, well above the 25.0 filter)
    geom.navigable_area.push_back(create_square(0.0, 0.0, 100.0));
    
    // Generate mesh with no margin and no simplification
    NavigationMesh mesh = MeshGenerator::generate(geom, 0.0, 0.0);
    
    // Validations
    ASSERT_NE(mesh.safe_navigable_perimeter, nullptr); // Must have a perimeter
    EXPECT_EQ(mesh.original_land, nullptr);            // No land was provided
    EXPECT_EQ(mesh.safety_margin, nullptr);            // No land = no margin
    EXPECT_GT(mesh.triangles.size(), 0);               // Triangulation must have happened
    
    // GDAL Memory Cleanup
    if (mesh.safe_navigable_perimeter) OGRGeometryFactory::destroyGeometry(mesh.safe_navigable_perimeter);
    for (auto* p : geom.navigable_area) OGRGeometryFactory::destroyGeometry(p);
}

/**
 * @brief Validates the advanced Boolean logic: applying safety margins around obstacles.
 * Tests if an obstacle in the middle of the water is expanded and carved out of the navmesh.
 */
TEST_F(MeshGeneratorTest, Generate_AppliesSafetyMarginToObstacles) {
    ProcessedGeometries geom;
    // 100x100 water area
    geom.navigable_area.push_back(create_square(0.0, 0.0, 100.0));
    // 20x20 island exactly in the middle of the water
    geom.obstacles.push_back(create_square(40.0, 40.0, 20.0)); 
    
    // Generate with a 5.0 meter safety margin
    NavigationMesh mesh = MeshGenerator::generate(geom, 5.0, 1.0);
    
    // Validations
    ASSERT_NE(mesh.original_land, nullptr);             // The island must be recorded
    ASSERT_NE(mesh.safety_margin, nullptr);             // The 5m margin must be created
    ASSERT_NE(mesh.safe_navigable_perimeter, nullptr);  // The carved ocean must exist
    EXPECT_GT(mesh.triangles.size(), 0);                // It must triangulate the complex shape
    
    // GDAL Memory Cleanup
    if (mesh.safe_navigable_perimeter) OGRGeometryFactory::destroyGeometry(mesh.safe_navigable_perimeter);
    if (mesh.original_land) OGRGeometryFactory::destroyGeometry(mesh.original_land);
    if (mesh.safety_margin) OGRGeometryFactory::destroyGeometry(mesh.safety_margin);
    
    for (auto* p : geom.navigable_area) OGRGeometryFactory::destroyGeometry(p);
    for (auto* p : geom.obstacles) OGRGeometryFactory::destroyGeometry(p);
}

// ==============================================================================
// 5. TESTS FOR: union_geometries
// ==============================================================================

/**
 * @brief Validates behavior when the input list is empty.
 * Assures the system returns nullptr instead of trying to access index 0.
 */
TEST_F(MeshGeneratorTest, UnionGeometries_HandlesEmptyList) {
    std::vector<OGRGeometry*> empty_list;
    
    OGRGeometry* result = MeshGenerator::union_geometries(empty_list);
    
    EXPECT_EQ(result, nullptr);
}

/**
 * @brief Validates merging of adjacent polygons into a single shape.
 * Two adjacent 10x10 squares (Area 100 each) should merge into one 20x10 rectangle (Area 200).
 */
TEST_F(MeshGeneratorTest, UnionGeometries_MergesPolygons) {
    std::vector<OGRGeometry*> poly_list;
    
    // Square 1: from X=0 to X=10 (Area = 100)
    poly_list.push_back(create_square(0.0, 0.0, 10.0));
    
    // Square 2: from X=10 to X=20 (Area = 100). It perfectly touches Square 1.
    poly_list.push_back(create_square(10.0, 0.0, 10.0));
    
    OGRGeometry* result = MeshGenerator::union_geometries(poly_list);
    
    ASSERT_NE(result, nullptr);
    
    // Validates that the result is still a valid Polygon
    EXPECT_EQ(wkbFlatten(result->getGeometryType()), wkbPolygon);
    
    // Validates that the boolean union actually summed the physical spaces
    OGRPolygon* result_poly = result->toPolygon();
    EXPECT_DOUBLE_EQ(result_poly->get_Area(), 200.0);
    
    // GDAL Memory Cleanup
    OGRGeometryFactory::destroyGeometry(result);
    for (auto* p : poly_list) OGRGeometryFactory::destroyGeometry(p);
}

// ==============================================================================
// 6. TESTS FOR: triangulate_polygon
// ==============================================================================

/**
 * @brief Validates null pointer protection.
 * Assures the triangulator doesn't crash when handed a null polygon.
 */
TEST_F(MeshGeneratorTest, TriangulatePolygon_HandlesNullPointer) {
    std::vector<Triangle> triangles;
    int fail_counter = 0;
    
    // Chamada estática com o escopo da classe
    MeshGenerator::triangulate_polygon(nullptr, triangles, fail_counter);
    
    EXPECT_TRUE(triangles.empty());
    EXPECT_EQ(fail_counter, 0);
}

/**
 * @brief Validates successful Delaunay triangulation of a simple square polygon.
 * Ensures the poly2tri engine correctly processes the external boundary 
 * and outputs valid triangles into the target list.
 */
TEST_F(MeshGeneratorTest, TriangulatePolygon_TriangulatesSquare) {
    std::vector<Triangle> triangles;
    int fail_counter = 0;
    
    // Creates a 10x10 square polygon
    OGRPolygon* square_poly = create_square(0.0, 0.0, 10.0);
    
    MeshGenerator::triangulate_polygon(square_poly, triangles, fail_counter);
    
    // Validations
    EXPECT_EQ(fail_counter, 0);             // No exceptions should occur
    EXPECT_FALSE(triangles.empty());        // Triangles must be generated
    
    // Valida os limites espaciais dos triângulos gerados
    for (const auto& tri : triangles) {
        // Verificamos os 3 pontos do triângulo usando a estrutura padrão de coordenadas (x, y)
        // Se a sua struct Triangle usa nomes como p1, p2, p3, ajuste aqui conforme necessário.
        // Como o push_back enviava initializer_lists, testamos os limites da caixa 10x10:
        
        // Ponto 1
        EXPECT_GE(tri.p1.x, 0.0); EXPECT_LE(tri.p1.x, 10.0);
        EXPECT_GE(tri.p1.y, 0.0); EXPECT_LE(tri.p1.y, 10.0);
        
        // Ponto 2
        EXPECT_GE(tri.p2.x, 0.0); EXPECT_LE(tri.p2.x, 10.0);
        EXPECT_GE(tri.p2.y, 0.0); EXPECT_LE(tri.p2.y, 10.0);
        
        // Ponto 3
        EXPECT_GE(tri.p3.x, 0.0); EXPECT_LE(tri.p3.x, 10.0);
        EXPECT_GE(tri.p3.y, 0.0); EXPECT_LE(tri.p3.y, 10.0);
    }
    
    // GDAL Memory Cleanup
    OGRGeometryFactory::destroyGeometry(square_poly);
}