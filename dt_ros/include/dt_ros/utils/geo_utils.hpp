/**
 * @file geo_utils.hpp
 * @brief Geographical coordinate utilities for the Digital Twin ROS adapter.
 */

#pragma once

namespace dt_ros {
namespace utils {

/**
 * @struct UTMCoord
 * @brief Represents a 2D planar coordinate in the Universal Transverse Mercator (UTM) system.
 */
struct UTMCoord {
    double x; ///< Easting coordinate in meters.
    double y; ///< Northing coordinate in meters.
};

/**
 * @brief Converts WGS84 latitude and longitude to UTM planar coordinates.
 * 
 * Uses the standard WGS84 ellipsoid parameters to project a geographical 
 * coordinate into the corresponding UTM zone.
 * 
 * @param lat Latitude in decimal degrees.
 * @param lon Longitude in decimal degrees.
 * @return UTMCoord The computed UTM easting (x) and northing (y) coordinates in meters.
 */
UTMCoord lat_lon_to_utm(double lat, double lon);

} // namespace utils
} // namespace dt_ros