#pragma once

#include <string>
#include <vector>
#include <cmath>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/box.hpp>
#include <boost/geometry/geometries/segment.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/index/rtree.hpp>
#include "dt_core/types.hpp" 

namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;

// Definitions of geometric primitives using the Boost.Geometry engine
typedef bg::model::d2::point_xy<double> BgPoint;
typedef bg::model::segment<BgPoint> BgSegment;
typedef bg::model::box<BgPoint> BgBox;
typedef bg::model::polygon<BgPoint> BgPolygon;

// Indexed pairs for efficient insertion into Boost R-Tree structures
typedef std::pair<BgBox, BgSegment> RTreeSegmentValue;
typedef std::pair<BgBox, size_t> RTreePolygonValue;

/**
 * @brief High-performance spatial indexing and querying engine.
 * 
 * Uses R-Tree spatial trees (via Boost.Geometry) to perform real-time 
 * proximity searches (OLogN), restricted zone verification (NavMesh), 
 * and dynamic target management within the robot's ecosystem.
 */
class SpatialIndex {
private:
    bgi::rtree<RTreeSegmentValue, bgi::quadratic<16>> rtree_margin_; ///< R-Tree spatial index for safety margin segments.
    bgi::rtree<RTreePolygonValue, bgi::quadratic<16>> rtree_mesh_;   ///< R-Tree spatial index for triangulated NavMesh polygons.
    std::vector<BgPolygon> mesh_triangles_;                          ///< Geometry cache of triangles for inclusion queries.
    std::vector<types::Target> global_targets_cache_;                ///< Internal cache to manage the state of dynamic targets.

public:
    /**
     * @brief Default constructor for the SpatialIndex class.
     */
    SpatialIndex() = default;

    /**
     * @brief Default destructor for the SpatialIndex class.
     */
    ~SpatialIndex() = default;

    /**
     * @brief Loads, processes, and builds spatial R-Trees from Shapefiles on disk.
     * @param margin_shp Path to the `.shp` file regarding the safety margin.
     * @param mesh_shp Path to the `.shp` file regarding the navigation mesh.
     * @return true if loading and indexing were successful, false otherwise.
     */
    bool load_shapefiles(const std::string& margin_shp, const std::string& mesh_shp);

    /**
     * @brief Calculates the Euclidean distance to the nearest static obstacle.
     * @param position Current position of the vehicle (containing X and Y coordinates).
     * @return Distance in meters to the nearest mapped margin or obstacle.
     */
    float get_closest_static_obstacle_distance(const types::Point& position) const;

    /**
     * @brief Checks if a given position is inside a restricted zone (outside the NavMesh).
     * @param position Current position of the vehicle to be tested.
     * @return true if the point is in a restricted/dangerous zone, false if it is in a safe navigable area.
     */
    bool is_inside_restricted_zone(const types::Point& position) const;
    
    /**
     * @brief Updates the internal cache of global targets monitored by the system.
     * @param targets Vector containing the new targets processed by the perception layer.
     */
    void update_global_targets(const std::vector<types::Target>& targets);

    /**
     * @brief Filters and returns active targets within a local range radius relative to a center.
     * @param center Central reference point.
     * @param radius Maximum search radius in meters.
     * @return Vector containing only the targets located within the area of interest.
     */
    std::vector<types::Target> get_active_local_targets(const types::Point& center, float radius) const;

    // -------------------------------------------------------------------------
    // Base and Debug Methods
    // -------------------------------------------------------------------------

    /**
     * @brief Auxiliary method that calculates the exact distance in meters to the safety margin.
     * @param position Reference position in `types::Point` format.
     * @return Shortest geometric distance calculated via R-Tree.
     */
    double calculate_margin_distance(const types::Point& position) const;

    /**
     * @brief Evaluates whether a specific point strictly belongs to the navigable polygon.
     * @param position Test coordinates.
     * @return true if the point is contained within the navigable mesh, false otherwise.
     */
    bool is_navigable(const types::Point& position) const;

    /**
     * @brief Generates spatial debugging files to validate R-Tree queries via QGIS.
     * @param test_points Vector of sample points for proximity testing.
     * @param output_folder Save directory for the generated vector files.
     * @param epsg_utm EPSG code of the applied UTM cartographic projection.
     */
    void export_rtree_debug(const std::vector<types::Point>& test_points, const std::string& output_folder, int epsg_utm) const;
};