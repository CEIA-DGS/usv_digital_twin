#ifndef DT_VIZ_MAIN_WINDOW_HPP
#define DT_VIZ_MAIN_WINDOW_HPP

#include "dt_core/twin_interface.hpp"
#include <memory>

// ============================================================
// Bibliotecas Qt
// ============================================================

#include <QBrush>
#include <QGraphicsEllipseItem>
#include <QGraphicsLineItem>
#include <QGraphicsPathItem>
#include <QGraphicsPolygonItem>
#include <QGraphicsScene>
#include <QGraphicsSimpleTextItem>
#include <QGraphicsView>
#include <QLabel>
#include <QMainWindow>
#include <QPainterPath>
#include <QResizeEvent>
#include <QTimer>
#include <QWidget>
#include <QWheelEvent>

// ============================================================
// Bibliotecas padrão do C++
// ============================================================

#include <cstdint>
#include <unordered_map>
#include <vector>

/**
 * @brief Representa um ponto da derrota planejada.
 */
struct RoutePoint
{
  double x;
  double y;
};

/**
 * @brief Janela principal da ferramenta de visualização.
 *
 * A classe desenha a zona livre, o USV, as embarcações
 * monitoradas, a trajetória percorrida e a derrota planejada.
 *
 * Também altera visualmente os alvos que apresentarem risco
 * de colisão.
 */
class MainWindow : public QMainWindow
{
public:
  explicit MainWindow(std::shared_ptr<dt::DigitalTwinCore> dt_core, QWidget * parent = nullptr);

  /**
   * @brief Atualiza a derrota planejada exibida na tela.
   *
   * @param route Lista ordenada de waypoints ativos.
   */
  void updatePlannedRoute(
    const std::vector<RoutePoint> & route);

  /**
   * @brief Altera a representação visual de um alvo.
   *
   * @param mmsi Identificador AIS da embarcação.
   * @param collision_imminent Indica a existência de risco.
   */
  void updateCollisionAlert(
    std::uint32_t mmsi,
    bool collision_imminent);

protected:
  /**
   * @brief Ajusta a escala quando a janela é redimensionada.
   */
  void resizeEvent(QResizeEvent * event) override;

  /**
   * @brief Intercepta eventos antes de serem processados (usado para o Zoom).
   */
  bool eventFilter(QObject *watched, QEvent *event) override;

private:
    
  std::shared_ptr<dt::DigitalTwinCore> dt_core_;
  
  // Armazena o centro geométrico da NavMesh para posicionamento inicial
  double map_center_x_ = 0.0;
  double map_center_y_ = 0.0;
  
  // Configuração inicial.
  void configureWindow();
  void createScene();
  void createInformationPanel();

  // Elementos gráficos.
  void drawGrid();
  void drawAxes();
  void drawFreeZone();
  void drawUsv();
  void drawScaleBar();

  // Derrota planejada.
  void drawPlannedRoute();
  void clearPlannedRoute();

  // Atualização dos dados simulados.
  void updateSimulation();
  void updateUsvTrajectory(double x, double y);

  void updateInformationPanel(
    double usv_x,
    double usv_y,
    double heading);

  // Aparência dos alvos.
  QBrush normalVesselBrush() const;
  QBrush collisionVesselBrush() const;

  // Componentes principais.
  QGraphicsScene * scene_;
  QGraphicsView * view_;
  QTimer * timer_;

  QWidget * central_widget_;
  QWidget * information_panel_;

  QLabel * usv_position_label_;
  QLabel * heading_label_;
  QLabel * vessel_count_label_;
  QLabel * simulation_status_label_;

  // Zona livre e USV.
  QGraphicsPolygonItem * free_zone_;
  QGraphicsPolygonItem * usv_;
  QGraphicsSimpleTextItem * usv_label_;
  QGraphicsLineItem * heading_line_;

  // Trajetória já percorrida.
  QGraphicsPathItem * trajectory_item_;
  QPainterPath trajectory_path_;
  std::vector<QPointF> trajectory_points_;

  // Derrota futura planejada.
  std::vector<RoutePoint> planned_route_;
  QGraphicsPathItem * planned_route_item_;
  std::vector<QGraphicsEllipseItem *> waypoint_items_;
  std::vector<QGraphicsSimpleTextItem *> waypoint_labels_;

  // Embarcações monitoradas.
  std::vector<QGraphicsEllipseItem *> vessels_;
  std::vector<QGraphicsSimpleTextItem *> vessel_labels_;

  // Permitem localizar um alvo específico pelo MMSI.
  std::unordered_map<
    std::uint32_t,
    QGraphicsEllipseItem *
  > vessel_items_by_mmsi_;

  std::unordered_map<
    std::uint32_t,
    QGraphicsSimpleTextItem *
  > vessel_labels_by_mmsi_;

  double simulation_time_;
};

#endif