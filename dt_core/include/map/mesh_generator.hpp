#pragma once
#include <vector>
#include "gdal_priv.h"
#include "s57_processor.hpp"

/**
 * @brief Geometric primitives for representing the navigation mesh.
 */
struct Point2D { double x, y; };
struct Triangle { Point2D p1, p2, p3; };

/**
 * @brief Data structure containing the processed triangular mesh and auxiliary geometries.
 */
struct NavigationMesh {
    std::vector<Triangle> triangles;
    OGRGeometry* original_land = nullptr;
    OGRGeometry* safety_margin = nullptr;
    OGRGeometry* safe_navigable_perimeter = nullptr;

    /**
     * @brief Deallocates the memory occupied by the cached OGR geometries.
     */
    void free_memory() {
        if (original_land) OGRGeometryFactory::destroyGeometry(original_land);
        if (safety_margin) OGRGeometryFactory::destroyGeometry(safety_margin);
        if (safe_navigable_perimeter) OGRGeometryFactory::destroyGeometry(safe_navigable_perimeter);
    }
};

/**
 * @brief Geometric engine for processing and generating the navigation mesh.
 * 
 * Acts as a static utility class. Executes boolean operations, 
 * spatial filtering, and Constrained Delaunay Triangulation (CDT).
 */
class MeshGenerator {
public:
    // Prevents accidental instantiation of the class
    MeshGenerator() = delete;

    /**
     * @brief Complete pipeline for generating the mesh from processed S-57 data.
     * @param geometries Structure containing the base water and land polygons.
     * @param margin_meters Distance of the safety margin to be expanded (Buffer).
     * @param simplification_tolerance Maximum allowed error (in meters) for vertex reduction (Douglas-Peucker).
     * @return NavigationMesh containing the generated triangles and auxiliary polygons.
     */
    static NavigationMesh generate(const ProcessedGeometries& geometries, double margin_meters, double simplification_tolerance);

private:
    /**
     * @brief Consolidates a set of fragmented geometries into a single unified object.
     * @param geometry_list Vector of pointers to the geometries to be merged.
     * @return OGRGeometry pointer to the resulting merged geometry.
     */
    static OGRGeometry* union_geometries(const std::vector<OGRGeometry*>& geometry_list);
    
    /**
     * @brief Executes the Constrained Delaunay Triangulation (CDT) on polygons with holes.
     * @param polygon Pointer to the base polygon to be triangulated.
     * @param target_list Output vector where the generated Triangle structures will be appended.
     * @param fail_counter Reference for counting failures in the poly2tri geometric engine.
     */
    static void triangulate_polygon(OGRPolygon* polygon, std::vector<Triangle>& target_list, int& fail_counter);
    
    /**
     * @brief Filters polygonal features based on a minimum area threshold.
     * @param geom Pointer to the original input geometry.
     * @param min_area Minimum required area in square meters for the feature to be kept.
     * @return OGRGeometry pointer to the new filtered geometry.
     */
    static OGRGeometry* filter_by_area(OGRGeometry* geom, double min_area);
};