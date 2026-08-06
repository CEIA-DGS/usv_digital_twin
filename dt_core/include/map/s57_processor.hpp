#pragma once
#include <string>
#include <vector>
#include "gdal_priv.h"
#include "ogrsf_frmts.h"
#include "ogr_spatialref.h"
#include "config_manager.hpp"

/**
 * @brief Container for storing processed geometries extracted from S-57 charts.
 * Encapsulates the memory management logic for OGRGeometry structures.
 */
struct ProcessedGeometries {
    std::vector<OGRGeometry*> navigable_area;
    std::vector<OGRGeometry*> obstacles;
    int dynamic_utm_epsg;

    /**
     * @brief Performs forced deallocation of each OGR geometry and clears the vectors.
     */
    void free_memory() {
        for (auto geom : navigable_area) OGRGeometryFactory::destroyGeometry(geom);
        for (auto geom : obstacles) OGRGeometryFactory::destroyGeometry(geom);
        navigable_area.clear();
        obstacles.clear();
    }
};

/**
 * @brief Specialized processor for ingesting, extracting, and treating S-57 geospatial data.
 */
class S57Processor {
public:
    /**
     * @brief Opens an S-57 nautical chart, filters layers via configuration, and reprojects coordinates.
     * @param s57_path Physical path of the chart (.000).
     * @param config Configuration containing the layer classes to be extracted.
     * @return ProcessedGeometries object containing the reprojected data ready for use.
     */
    static ProcessedGeometries process_chart(const std::string& s57_path, const MapConfiguration& config);
};