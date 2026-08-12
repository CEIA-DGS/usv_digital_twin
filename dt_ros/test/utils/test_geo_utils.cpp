/**
 * @file test_geo_utils.cpp
 * @brief Unit tests for geographical coordinate conversion utilities.
 */

#include <gtest/gtest.h>
#include "dt_ros/utils/geo_utils.hpp"

namespace dt_ros {
namespace utils {

/**
 * @brief Test suite for geographical utilities.
 */
class GeoUtilsTest : public ::testing::Test {
protected:
    // Tolerance for floating-point comparisons
    const double EPSILON = 1e-2; 
};

/**
 * @brief Verifies conversion exactly at the equator and central meridian.
 * 
 * At the equator (Lat = 0) and the central meridian of a zone, 
 * Easting is exactly 500,000m and Northing is exactly 0m.
 * Longitude -45.0 corresponds to the central meridian of UTM Zone 23.
 */
TEST_F(GeoUtilsTest, LatLonToUtmEquatorCentralMeridian) {
    double lat = 0.0;
    double lon = -45.0;

    UTMCoord coord = lat_lon_to_utm(lat, lon);

    EXPECT_NEAR(coord.x, 500000.0, EPSILON);
    EXPECT_NEAR(coord.y, 0.0, EPSILON);
}

/**
 * @brief Verifies Southern Hemisphere false northing application.
 * 
 * Tests coordinates in the Southern Hemisphere (Brasilia region) to ensure
 * the 10,000,000m False Northing adjustment is correctly applied to avoid
 * negative Y coordinates.
 */
TEST_F(GeoUtilsTest, LatLonToUtmSouthernHemisphere) {
    double lat = -15.7634;
    double lon = -47.8703;

    const double EXPECTED_EASTING = 192432.90; 
    const double EXPECTED_NORTHING = 8255141.54;

    UTMCoord coord = lat_lon_to_utm(lat, lon);

    EXPECT_NEAR(coord.x, EXPECTED_EASTING, 1.0);
    EXPECT_NEAR(coord.y, EXPECTED_NORTHING, 1.0);
}

} // namespace utils
} // namespace dt_ros