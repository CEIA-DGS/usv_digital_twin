/**
 * @file geo_utils.cpp
 * @brief Implementation of geographical coordinate conversion utilities.
 */

#include "dt_ros/utils/geo_utils.hpp"
#include <cmath>

namespace dt_ros {
namespace utils {

UTMCoord lat_lon_to_utm(double lat, double lon) {
    UTMCoord coord;

    // WGS84 ellipsoid constants
    const double a = 6378137.0;            // Equatorial radius
    const double ecc_squared = 0.00669438; // Square of eccentricity
    const double k0 = 0.9996;              // Central meridian scale factor

    double lat_rad = lat * M_PI / 180.0;
    double lon_rad = lon * M_PI / 180.0;

    // Determine the UTM zone dynamically based on longitude
    int zone_number = static_cast<int>((lon + 180.0) / 6.0) + 1;
    double lon_origin = (zone_number - 1) * 6.0 - 180.0 + 3.0; // Central meridian of the zone
    double lon_origin_rad = lon_origin * M_PI / 180.0;

    // Standard projection variables (Snyder's equations)
    double N = a / std::sqrt(1.0 - ecc_squared * std::sin(lat_rad) * std::sin(lat_rad));
    double T = std::tan(lat_rad) * std::tan(lat_rad);
    double C = (ecc_squared / (1.0 - ecc_squared)) * std::cos(lat_rad) * std::cos(lat_rad);
    double A = std::cos(lat_rad) * (lon_rad - lon_origin_rad);

    // Calculate the Meridional Arc (M)
    double M = a * ((1.0 - ecc_squared / 4.0 - 3.0 * ecc_squared * ecc_squared / 64.0 - 5.0 * ecc_squared * ecc_squared * ecc_squared / 256.0) * lat_rad 
                  - (3.0 * ecc_squared / 8.0 + 3.0 * ecc_squared * ecc_squared / 32.0 + 45.0 * ecc_squared * ecc_squared * ecc_squared / 1024.0) * std::sin(2.0 * lat_rad) 
                  + (15.0 * ecc_squared * ecc_squared / 256.0 + 45.0 * ecc_squared * ecc_squared * ecc_squared / 1024.0) * std::sin(4.0 * lat_rad) 
                  - (35.0 * ecc_squared * ecc_squared * ecc_squared / 3072.0) * std::sin(6.0 * lat_rad));

    // Compute Easting (x) and Northing (y)
    coord.x = k0 * N * (A + (1.0 - T + C) * A * A * A / 6.0 + (5.0 - 18.0 * T + T * T + 72.0 * C - 58.0 * ecc_squared) * A * A * A * A * A / 120.0) + 500000.0;
    coord.y = k0 * (M + N * std::tan(lat_rad) * (A * A / 2.0 + (5.0 - T + 9.0 * C + 4.0 * C * C) * A * A * A * A / 24.0 + (61.0 - 58.0 * T + T * T + 600.0 * C - 330.0 * ecc_squared) * A * A * A * A * A * A / 720.0));
    
    // Northern/Southern hemisphere adjustment (False Northing)
    if (lat < 0.0) {
        coord.y += 10000000.0; 
    }

    return coord;
}

} // namespace utils
} // namespace dt_ros