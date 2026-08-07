#include "dt_ros/utils/geo_utils.hpp"
#include <cmath>

namespace dt_ros {
namespace utils {

UTMCoord latLonToUTM(double lat, double lon) {
    UTMCoord coord;

    // Constantes do Elipsóide WGS84
    const double a = 6378137.0; // Raio equatorial
    const double eccSquared = 0.00669438; // Excentricidade ao quadrado
    const double k0 = 0.9996; // Fator de escala no meridiano central

    double latRad = lat * M_PI / 180.0;
    double lonRad = lon * M_PI / 180.0;

    // Determina o fuso (Zone) do UTM
    int zoneNumber = static_cast<int>((lon + 180.0) / 6.0) + 1;
    double lonOrigin = (zoneNumber - 1) * 6.0 - 180.0 + 3.0; // Centro do fuso
    double lonOriginRad = lonOrigin * M_PI / 180.0;

    double N = a / std::sqrt(1 - eccSquared * std::sin(latRad) * std::sin(latRad));
    double T = std::tan(latRad) * std::tan(latRad);
    double C = eccSquared / (1 - eccSquared) * std::cos(latRad) * std::cos(latRad);
    double A = std::cos(latRad) * (lonRad - lonOriginRad);

    // Cálculo do Meridiano Arco (M)
    double M = a * ((1 - eccSquared/4 - 3*eccSquared*eccSquared/64 - 5*eccSquared*eccSquared*eccSquared/256) * latRad 
                  - (3*eccSquared/8 + 3*eccSquared*eccSquared/32 + 45*eccSquared*eccSquared*eccSquared/1024) * std::sin(2*latRad) 
                  + (15*eccSquared*eccSquared/256 + 45*eccSquared*eccSquared*eccSquared/1024) * std::sin(4*latRad) 
                  - (35*eccSquared*eccSquared*eccSquared/3072) * std::sin(6*latRad));

    coord.x = k0 * N * (A + (1-T+C)*A*A*A/6 + (5-18*T+T*T+72*C-58*eccSquared)*A*A*A*A*A/120) + 500000.0;
    coord.y = k0 * (M + N*std::tan(latRad)*(A*A/2 + (5-T+9*C+4*C*C)*A*A*A*A/24 + (61-58*T+T*T+600*C-330*eccSquared)*A*A*A*A*A*A/720));
    
    if (lat < 0) {
        coord.y += 10000000.0; // Deslocamento para o hemisfério sul
    }

    return coord;
}

} // namespace utils
} // namespace dt_ros