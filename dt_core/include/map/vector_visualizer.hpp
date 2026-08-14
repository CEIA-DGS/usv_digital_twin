#pragma once
#include <string>
#include <vector>

class OGRGeometry; 

struct Vertex2D { double x, y; };

struct Polygon2D {
    std::vector<Vertex2D> outer_ring;
    std::vector<std::vector<Vertex2D>> holes;
};

/**
 * @brief Extrai a geometria bruta do GDAL e a converte para estruturas 2D amigáveis ao OpenGL.
 * Exposta aqui no header primariamente para permitir Testes Unitários de lógicas complexas (ex: furos).
 */
void extract_geom_polygons(OGRGeometry* geom, std::vector<Polygon2D>& target_list);

/**
 * @brief Responsible for the graphical rendering of the NavMesh and processed spatial layers.
 * Utilizes the OpenGL/GLFW context for interactive visualization of vector data.
 */
class VectorVisualizer {
public:
    /**
     * @brief Starts the rendering loop and displays the geographic layers contained in the target directory.
     * @param shapefiles_folder Path to the directory containing the processed .shp files.
     * @param chart_name Chart identifier, used for labeling the display window.
     */
    static void display(const std::string& shapefiles_folder, const std::string& chart_name);
};