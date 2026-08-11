#pragma once

#include <string>
#include "mesh_generator.hpp"
#include "gdal_priv.h"
#include "ogrsf_frmts.h"

/**
 * @brief Utility responsible for exporting generated NavMesh structures to vector files (Shapefiles).
 * 
 * Acts as a pure static class, encapsulating disk I/O operations 
 * and geometric conversion using the GDAL/OGR library.
 */
class VectorExporter {
public:
    // Prevents accidental instantiation of the class
    VectorExporter() = delete;

    /**
     * @brief Exports the geometries (original land, safety margin, and triangle mesh) to .shp files.
     * @param nav_mesh Constant reference to the structure containing the processed navigation mesh data.
     * @param output_dir Absolute or relative path of the directory where Shapefiles will be written.
     * @param epsg_utm EPSG code of the UTM projection to ensure correct georeferencing of the generated files.
     */
    static void export_shapefile(const NavigationMesh& nav_mesh, const std::string& output_dir, int epsg_utm);

private:
    /**
     * @brief Auxiliary function to safely append a generic geometry to an OGR Layer.
     * @param layer Pointer to the destination layer (OGR Layer) previously created in the dataset.
     * @param geom Pointer to the base geometry (Polygon, Line, etc.) that will be encapsulated in a feature.
     */
    static void insert_geometry(OGRLayer* layer, OGRGeometry* geom);
};