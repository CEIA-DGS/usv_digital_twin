// geo_utils.hpp
namespace dt_ros {
namespace utils {
    // Retorna um std::pair ou uma struct dedicada em vez de usar referências como saída
    struct UTMCoord {
        double x;
        double y;
    };

    UTMCoord latLonToUTM(double lat, double lon);
}
}