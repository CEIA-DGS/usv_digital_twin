#include "../include/VisualizadorVetor.h"
#include <iostream>
#include <vector>
#include <algorithm>
#include <GLFW/glfw3.h>
#include <GL/gl.h>
#include "gdal_priv.h"
#include "ogrsf_frmts.h"

// Estruturas auxiliares para renderização vetorial
struct Color { float r, g, b, a; };
struct Vertex2D { double x, y; };

struct Polygon2D {
    std::vector<Vertex2D> outerRing;
    std::vector<std::vector<Vertex2D>> holes;
};

struct RenderLayer {
    std::vector<Polygon2D> poligonos;
    Color fillColor;
    Color lineColor;
    float lineWidth;
    bool filled;
};

// Estado da câmera e viewport
struct AppState {
    double panX = 0.0, panY = 0.0;
    double zoom = 1.0;
    bool dragging = false;
    double lastMouseX = 0.0, lastMouseY = 0.0;
    int windowWidth = 1400;
    int windowHeight = 900;
    OGREnvelope bounds;
};

/**
 * @brief Realiza o parsing recursivo de geometrias GDAL para a estrutura interna de renderização.
 * Tratamento de MultiPolygons e coleções de geometrias.
 */

static void extrairPoligonosGeom(OGRGeometry* geom, std::vector<Polygon2D>& lista) {
    if (!geom) return;
    OGRwkbGeometryType tipo = wkbFlatten(geom->getGeometryType());
    
    if (tipo == wkbPolygon) {
        OGRPolygon* poly = geom->toPolygon();
        Polygon2D p2d;
        
        // Extração do contorno externo
        OGRLinearRing* ext = poly->getExteriorRing();
        if (ext) {
            for (int i = 0; i < ext->getNumPoints(); i++) 
                p2d.outerRing.push_back({ext->getX(i), ext->getY(i)});
        }
        
        // Extração dos furos internos
        for (int b = 0; b < poly->getNumInteriorRings(); b++) {
            OGRLinearRing* hole = poly->getInteriorRing(b);
            std::vector<Vertex2D> holePts;
            for (int i = 0; i < hole->getNumPoints(); i++) 
                holePts.push_back({hole->getX(i), hole->getY(i)});
            p2d.holes.push_back(holePts);
        }
        lista.push_back(p2d);
    } else if (tipo == wkbMultiPolygon || tipo == wkbGeometryCollection) {
        OGRGeometryCollection* col = (OGRGeometryCollection*)geom;
        for (auto&& sub : *col) extrairPoligonosGeom(sub, lista);
    }
}

/**
 * @brief Carrega as feições de uma camada GDAL para a memória de renderização.
 */

static RenderLayer carregarCamadaShapefile(GDALDataset* dataset, const char* nomeCamada, Color fillColor, Color lineColor, float lineWidth, bool filled) {
    RenderLayer layer;
    layer.fillColor = fillColor;
    layer.lineColor = lineColor;
    layer.lineWidth = lineWidth;
    layer.filled = filled;

    OGRLayer* poLayer = dataset->GetLayerByName(nomeCamada);
    if (!poLayer) return layer;

    poLayer->ResetReading();
    OGRFeature* feat;
    while ((feat = poLayer->GetNextFeature()) != nullptr) {
        OGRGeometry* geom = feat->GetGeometryRef();
        if (geom) extrairPoligonosGeom(geom, layer.poligonos);
        OGRFeature::DestroyFeature(feat);
    }
    return layer;
}

// Callbacks de entrada
static void mouseButtonCallback(GLFWwindow* window, int button, int action, int mods) {
    AppState* state = static_cast<AppState*>(glfwGetWindowUserPointer(window));
    if (button == GLFW_MOUSE_BUTTON_LEFT) {
        if (action == GLFW_PRESS) {
            state->dragging = true;
            glfwGetCursorPos(window, &state->lastMouseX, &state->lastMouseY);
        } else if (action == GLFW_RELEASE) state->dragging = false;
    }
}

static void cursorPositionCallback(GLFWwindow* window, double xpos, double ypos) {
    AppState* state = static_cast<AppState*>(glfwGetWindowUserPointer(window));
    if (state->dragging) {
        state->panX -= (xpos - state->lastMouseX) / state->zoom;
        state->panY += (ypos - state->lastMouseY) / state->zoom;
        state->lastMouseX = xpos;
        state->lastMouseY = ypos;
    }
}

static void scrollCallback(GLFWwindow* window, double xoffset, double yoffset) {
    AppState* state = static_cast<AppState*>(glfwGetWindowUserPointer(window));
    double fatorZoom = (yoffset > 0) ? 1.15 : (1.0 / 1.15);
    double mouseX, mouseY;
    glfwGetCursorPos(window, &mouseX, &mouseY);

    double worldXBefore = state->panX + (mouseX - state->windowWidth / 2.0) / state->zoom;
    double worldYBefore = state->panY + (state->windowHeight / 2.0 - mouseY) / state->zoom;

    state->zoom *= fatorZoom;

    state->panX = worldXBefore - (mouseX - state->windowWidth / 2.0) / state->zoom;
    state->panY = worldYBefore - (state->windowHeight / 2.0 - mouseY) / state->zoom;
}

void VisualizadorVetor::exibir(const std::string& pastaShapefiles, const std::string& nomeCarta) {
    if (!glfwInit()) return;

    GDALDataset* dataset = (GDALDataset*)GDALOpenEx(pastaShapefiles.c_str(), GDAL_OF_VECTOR, nullptr, nullptr, nullptr);
    if (!dataset) { glfwTerminate(); return; }

    AppState state;
    OGREnvelope env;
    bool envOk = false;
    
    // Cálculo dos bounds para enquadramento inicial
    OGRLayer* navLayer = dataset->GetLayerByName("3_Area_Navegavel_Limpa");
    if (navLayer && navLayer->GetExtent(&env, true) == OGRERR_NONE) { state.bounds = env; envOk = true; }
    else {
        for (int i = 0; i < dataset->GetLayerCount(); i++) {
            if (dataset->GetLayer(i)->GetExtent(&env, true) == OGRERR_NONE) {
                if (!envOk) { state.bounds = env; envOk = true; }
                else { state.bounds.Merge(env); }
            }
        }
    }

    if (!envOk) { GDALClose(dataset); glfwTerminate(); return; }

    // Carregamento estruturado das camadas (RGBA normalizado)
    std::vector<RenderLayer> camadas;
    camadas.push_back(carregarCamadaShapefile(dataset, "3_Area_Navegavel_Limpa", {173.0f/255.0f, 235.0f/255.0f, 255.0f/255.0f, 1.0f}, {0,0,0,0}, 0.0f, true));
    camadas.push_back(carregarCamadaShapefile(dataset, "1_Terra_Firme", {255.0f/255.0f, 185.0f/255.0f, 141.0f/255.0f, 1.0f}, {0,0,0,0}, 0.0f, true));
    camadas.push_back(carregarCamadaShapefile(dataset, "4_Malha_NavMesh", {0,0,0,0}, {0.0f/255.0f, 80.0f/255.0f, 150.0f/255.0f, 1.0f}, 1.0f, false));
    camadas.push_back(carregarCamadaShapefile(dataset, "2_Margem_Seguranca", {0,0,0,0}, {1.0f, 0.0f, 0.0f, 1.0f}, 2.0f, false));

    GDALClose(dataset);

    // Inicialização da janela com Stencil Buffer habilitado (necessário para polígonos côncavos)
    glfwWindowHint(GLFW_STENCIL_BITS, 8);
    GLFWwindow* window = glfwCreateWindow(state.windowWidth, state.windowHeight, ("NavMesh Vetorial - " + nomeCarta).c_str(), NULL, NULL);
    if (!window) { glfwTerminate(); return; }

    glfwMakeContextCurrent(window);
    glfwSetWindowUserPointer(window, &state);
    glfwSetMouseButtonCallback(window, mouseButtonCallback);
    glfwSetCursorPosCallback(window, cursorPositionCallback);
    glfwSetScrollCallback(window, scrollCallback);

    // Configuração inicial de escala
    double mundoW = state.bounds.MaxX - state.bounds.MinX;
    double mundoH = state.bounds.MaxY - state.bounds.MinY;
    state.zoom = std::min(state.windowWidth / (mundoW < 1.0 ? 1000.0 : mundoW), state.windowHeight / (mundoH < 1.0 ? 1000.0 : mundoH)) * 0.95;
    state.panX = (state.bounds.MinX + state.bounds.MaxX) / 2.0;
    state.panY = (state.bounds.MinY + state.bounds.MaxY) / 2.0;

    // Loop de renderização principal
    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();
        glfwGetWindowSize(window, &state.windowWidth, &state.windowHeight);

        glViewport(0, 0, state.windowWidth, state.windowHeight);
        glClearColor(0.27f, 0.27f, 0.27f, 1.0f); 
        glClear(GL_COLOR_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);

        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        double halfW = (state.windowWidth / 2.0) / state.zoom;
        double halfH = (state.windowHeight / 2.0) / state.zoom;
        glOrtho(state.panX - halfW, state.panX + halfW, state.panY - halfH, state.panY + halfH, -1.0, 1.0);

        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();

        // Renderização em duas passagens para Stencil Buffer (Lógica Odd-Even para polígonos complexos)
        for (const auto& layer : camadas) {
            for (const auto& poly : layer.poligonos) {
                if (poly.outerRing.empty()) continue;

                if (layer.filled) {
                    glEnable(GL_STENCIL_TEST);
                    glClear(GL_STENCIL_BUFFER_BIT);
                    
                    glColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE);
                    glStencilFunc(GL_ALWAYS, 0, 1);
                    glStencilOp(GL_KEEP, GL_KEEP, GL_INVERT); 
                    glStencilMask(1);

                    // Desenho da máscara (Stencil)
                    glBegin(GL_TRIANGLE_FAN);
                    for (const auto& pt : poly.outerRing) glVertex2d(pt.x, pt.y);
                    glEnd();

                    for (const auto& hole : poly.holes) {
                        glBegin(GL_TRIANGLE_FAN);
                        for (const auto& pt : hole) glVertex2d(pt.x, pt.y);
                        glEnd();
                    }

                    // Aplicação da cor via Stencil
                    glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
                    glStencilFunc(GL_EQUAL, 1, 1);
                    glStencilOp(GL_KEEP, GL_KEEP, GL_KEEP);

                    glColor4f(layer.fillColor.r, layer.fillColor.g, layer.fillColor.b, layer.fillColor.a);
                    
                    // Preenchimento delimitado pela bounding box
                    double minX = poly.outerRing[0].x, maxX = minX, minY = poly.outerRing[0].y, maxY = minY;
                    for (const auto& pt : poly.outerRing) {
                        minX = std::min(minX, pt.x); maxX = std::max(maxX, pt.x);
                        minY = std::min(minY, pt.y); maxY = std::max(maxY, pt.y);
                    }
                    glBegin(GL_QUADS);
                    glVertex2d(minX, minY); glVertex2d(maxX, minY);
                    glVertex2d(maxX, maxY); glVertex2d(minX, maxY);
                    glEnd();

                    glDisable(GL_STENCIL_TEST);
                }

                // Renderização das linhas de contorno (wireframe)
                if (layer.lineWidth > 0.0f) {
                    glLineWidth(layer.lineWidth);
                    glColor4f(layer.lineColor.r, layer.lineColor.g, layer.lineColor.b, layer.lineColor.a);
                    glBegin(GL_LINE_LOOP);
                    for (const auto& pt : poly.outerRing) glVertex2d(pt.x, pt.y);
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