#include "../../include/map/vector_visualizer.hpp"
#include <iostream>
#include <vector>
#include <algorithm>
#include <GLFW/glfw3.h>
#include <GL/gl.h>
#include "gdal_priv.h"
#include "ogrsf_frmts.h"

// Auxiliary structures for vector rendering
struct Color { float r, g, b, a; };
struct Vertex2D { double x, y; };

struct Polygon2D {
    std::vector<Vertex2D> outer_ring;
    std::vector<std::vector<Vertex2D>> holes;
};

struct RenderLayer {
    std::vector<Polygon2D> polygons;
    Color fill_color;
    Color line_color;
    float line_width;
    bool is_filled;
};

// Camera and viewport state
struct AppState {
    double pan_x = 0.0, pan_y = 0.0;
    double zoom = 1.0;
    bool is_dragging = false;
    double last_mouse_x = 0.0, last_mouse_y = 0.0;
    int window_width = 1400;
    int window_height = 900;
    OGREnvelope bounds;
};

static void extract_geom_polygons(OGRGeometry* geom, std::vector<Polygon2D>& target_list) {
    if (!geom) return;
    OGRwkbGeometryType type = wkbFlatten(geom->getGeometryType());
    
    if (type == wkbPolygon) {
        OGRPolygon* poly = geom->toPolygon();
        Polygon2D p2d;
        
        // Extraction of the outer contour
        OGRLinearRing* ext = poly->getExteriorRing();
        if (ext) {
            for (int i = 0; i < ext->getNumPoints(); i++) 
                p2d.outer_ring.push_back({ext->getX(i), ext->getY(i)});
        }
        
        // Extraction of the internal holes
        for (int b = 0; b < poly->getNumInteriorRings(); b++) {
            OGRLinearRing* hole = poly->getInteriorRing(b);
            std::vector<Vertex2D> hole_pts;
            for (int i = 0; i < hole->getNumPoints(); i++) 
                hole_pts.push_back({hole->getX(i), hole->getY(i)});
            p2d.holes.push_back(hole_pts);
        }
        target_list.push_back(p2d);
    } else if (type == wkbMultiPolygon || type == wkbGeometryCollection) {
        OGRGeometryCollection* collection = (OGRGeometryCollection*)geom;
        for (auto&& sub : *collection) extract_geom_polygons(sub, target_list);
    }
}

static RenderLayer load_shapefile_layer(GDALDataset* dataset, const char* layer_name, Color fill_color, Color line_color, float line_width, bool is_filled) {
    RenderLayer layer;
    layer.fill_color = fill_color;
    layer.line_color = line_color;
    layer.line_width = line_width;
    layer.is_filled = is_filled;

    OGRLayer* gdal_layer = dataset->GetLayerByName(layer_name);
    if (!gdal_layer) return layer;

    gdal_layer->ResetReading();
    OGRFeature* feat;
    while ((feat = gdal_layer->GetNextFeature()) != nullptr) {
        OGRGeometry* geom = feat->GetGeometryRef();
        if (geom) extract_geom_polygons(geom, layer.polygons);
        OGRFeature::DestroyFeature(feat);
    }
    return layer;
}

// Input Callbacks

static void mouse_button_callback(GLFWwindow* window, int button, int action, int mods) {
    AppState* state = static_cast<AppState*>(glfwGetWindowUserPointer(window));
    if (button == GLFW_MOUSE_BUTTON_LEFT) {
        if (action == GLFW_PRESS) {
            state->is_dragging = true;
            glfwGetCursorPos(window, &state->last_mouse_x, &state->last_mouse_y);
        } else if (action == GLFW_RELEASE) state->is_dragging = false;
    }
}

static void cursor_position_callback(GLFWwindow* window, double xpos, double ypos) {
    AppState* state = static_cast<AppState*>(glfwGetWindowUserPointer(window));
    if (state->is_dragging) {
        state->pan_x -= (xpos - state->last_mouse_x) / state->zoom;
        state->pan_y += (ypos - state->last_mouse_y) / state->zoom;
        state->last_mouse_x = xpos;
        state->last_mouse_y = ypos;
    }
}

static void scroll_callback(GLFWwindow* window, double xoffset, double yoffset) {
    AppState* state = static_cast<AppState*>(glfwGetWindowUserPointer(window));
    double zoom_factor = (yoffset > 0) ? 1.15 : (1.0 / 1.15);
    double mouse_x, mouse_y;
    glfwGetCursorPos(window, &mouse_x, &mouse_y);

    double world_x_before = state->pan_x + (mouse_x - state->window_width / 2.0) / state->zoom;
    double world_y_before = state->pan_y + (state->window_height / 2.0 - mouse_y) / state->zoom;

    state->zoom *= zoom_factor;

    state->pan_x = world_x_before - (mouse_x - state->window_width / 2.0) / state->zoom;
    state->pan_y = world_y_before - (state->window_height / 2.0 - mouse_y) / state->zoom;
}

void VectorVisualizer::display(const std::string& shapefiles_folder, const std::string& chart_name) {
    if (!glfwInit()) return;

    GDALDataset* dataset = (GDALDataset*)GDALOpenEx(shapefiles_folder.c_str(), GDAL_OF_VECTOR, nullptr, nullptr, nullptr);
    if (!dataset) { glfwTerminate(); return; }

    AppState state;
    OGREnvelope env;
    bool is_env_ok = false;
    
    // Bounds calculation for initial framing
    OGRLayer* nav_layer = dataset->GetLayerByName("3_Area_Navegavel_Limpa");
    if (nav_layer && nav_layer->GetExtent(&env, true) == OGRERR_NONE) { state.bounds = env; is_env_ok = true; }
    else {
        for (int i = 0; i < dataset->GetLayerCount(); i++) {
            if (dataset->GetLayer(i)->GetExtent(&env, true) == OGRERR_NONE) {
                if (!is_env_ok) { state.bounds = env; is_env_ok = true; }
                else { state.bounds.Merge(env); }
            }
        }
    }

    if (!is_env_ok) { GDALClose(dataset); glfwTerminate(); return; }

    // Structured loading of layers (Normalized RGBA)
    std::vector<RenderLayer> layers;
    layers.push_back(load_shapefile_layer(dataset, "3_Area_Navegavel_Limpa", {173.0f/255.0f, 235.0f/255.0f, 255.0f/255.0f, 1.0f}, {0,0,0,0}, 0.0f, true));
    layers.push_back(load_shapefile_layer(dataset, "1_Terra_Firme", {255.0f/255.0f, 185.0f/255.0f, 141.0f/255.0f, 1.0f}, {0,0,0,0}, 0.0f, true));
    layers.push_back(load_shapefile_layer(dataset, "4_Malha_NavMesh", {0,0,0,0}, {0.0f/255.0f, 80.0f/255.0f, 150.0f/255.0f, 1.0f}, 1.0f, false));
    layers.push_back(load_shapefile_layer(dataset, "2_Margem_Seguranca", {0,0,0,0}, {1.0f, 0.0f, 0.0f, 1.0f}, 2.0f, false));

    GDALClose(dataset);

    // Window initialization with Stencil Buffer enabled (necessary for concave polygons)
    glfwWindowHint(GLFW_STENCIL_BITS, 8);
    GLFWwindow* window = glfwCreateWindow(state.window_width, state.window_height, ("Vector NavMesh - " + chart_name).c_str(), NULL, NULL);
    if (!window) { glfwTerminate(); return; }

    glfwMakeContextCurrent(window);
    glfwSetWindowUserPointer(window, &state);
    glfwSetMouseButtonCallback(window, mouse_button_callback);
    glfwSetCursorPosCallback(window, cursor_position_callback);
    glfwSetScrollCallback(window, scroll_callback);

    // Initial scale configuration
    double world_w = state.bounds.MaxX - state.bounds.MinX;
    double world_h = state.bounds.MaxY - state.bounds.MinY;
    state.zoom = std::min(state.window_width / (world_w < 1.0 ? 1000.0 : world_w), state.window_height / (world_h < 1.0 ? 1000.0 : world_h)) * 0.95;
    state.pan_x = (state.bounds.MinX + state.bounds.MaxX) / 2.0;
    state.pan_y = (state.bounds.MinY + state.bounds.MaxY) / 2.0;

    // Main rendering loop
    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();
        glfwGetWindowSize(window, &state.window_width, &state.window_height);

        glViewport(0, 0, state.window_width, state.window_height);
        glClearColor(0.27f, 0.27f, 0.27f, 1.0f); 
        glClear(GL_COLOR_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);

        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        double half_w = (state.window_width / 2.0) / state.zoom;
        double half_h = (state.window_height / 2.0) / state.zoom;
        glOrtho(state.pan_x - half_w, state.pan_x + half_w, state.pan_y - half_h, state.pan_y + half_h, -1.0, 1.0);

        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();

        // Two-pass rendering for Stencil Buffer (Odd-Even logic for complex polygons)
        for (const auto& layer : layers) {
            for (const auto& poly : layer.polygons) {
                if (poly.outer_ring.empty()) continue;

                if (layer.is_filled) {
                    glEnable(GL_STENCIL_TEST);
                    glClear(GL_STENCIL_BUFFER_BIT);
                    
                    glColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE);
                    glStencilFunc(GL_ALWAYS, 0, 1);
                    glStencilOp(GL_KEEP, GL_KEEP, GL_INVERT); 
                    glStencilMask(1);

                    // Drawing the mask (Stencil)
                    glBegin(GL_TRIANGLE_FAN);
                    for (const auto& pt : poly.outer_ring) glVertex2d(pt.x, pt.y);
                    glEnd();

                    for (const auto& hole : poly.holes) {
                        glBegin(GL_TRIANGLE_FAN);
                        for (const auto& pt : hole) glVertex2d(pt.x, pt.y);
                        glEnd();
                    }

                    // Applying color via Stencil
                    glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
                    glStencilFunc(GL_EQUAL, 1, 1);
                    glStencilOp(GL_KEEP, GL_KEEP, GL_KEEP);

                    glColor4f(layer.fill_color.r, layer.fill_color.g, layer.fill_color.b, layer.fill_color.a);
                    
                    // Bounding box delimited filling
                    double min_x = poly.outer_ring[0].x, max_x = min_x, min_y = poly.outer_ring[0].y, max_y = min_y;
                    for (const auto& pt : poly.outer_ring) {
                        min_x = std::min(min_x, pt.x); max_x = std::max(max_x, pt.x);
                        min_y = std::min(min_y, pt.y); max_y = std::max(max_y, pt.y);
                    }
                    glBegin(GL_QUADS);
                    glVertex2d(min_x, min_y); glVertex2d(max_x, min_y);
                    glVertex2d(max_x, max_y); glVertex2d(min_x, max_y);
                    glEnd();

                    glDisable(GL_STENCIL_TEST);
                }

                // Rendering outline contours (wireframe)
                if (layer.line_width > 0.0f) {
                    glLineWidth(layer.line_width);
                    glColor4f(layer.line_color.r, layer.line_color.g, layer.line_color.b, layer.line_color.a);
                    glBegin(GL_LINE_LOOP);
                    for (const auto& pt : poly.outer_ring) glVertex2d(pt.x, pt.y);
                    glEnd();
                    for (const auto& hole : poly.holes) {
                        glBegin(GL_LINE_LOOP);
                        for (const auto& pt : hole) glVertex2d(pt.x, pt.y);
                        glEnd();
                    }
                }
            }
        }
        glfwSwapBuffers(window);
    }
    glfwTerminate();
}