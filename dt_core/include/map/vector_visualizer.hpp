#pragma once
#include <string>

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