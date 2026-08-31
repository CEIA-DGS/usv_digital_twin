#pragma once

#include <QWidget>
#include <QLabel>
#include <QGridLayout>
#include <vector>
#include <memory>

namespace dt_viz {

/**
 * @brief Manages a dynamic grid of camera feeds (supporting 1, 2, or up to 4 feeds simultaneously).
 */
class CameraGridWidget : public QWidget {
  Q_OBJECT

public:
  /**
   * @brief Constructs the CameraGridWidget.
   * @param parent Pointer to the parent widget.
   */
  explicit CameraGridWidget(QWidget * parent = nullptr);

  /**
   * @brief Changes the active grid layout mode.
   * @param mode Number of active slots (1, 2, or 4).
   */
  void setLayoutMode(int mode);

  /**
   * @brief Updates the image frame and label for a specific slot.
   * @param slot_index Index of the grid slot (0 to 3).
   * @param camera_id Text identifier of the camera.
   * @param image QImage containing the video frame.
   */
  void updateCameraFrame(int slot_index, const QString & camera_id, const QImage & image);

private:
  void setupUi();

  QGridLayout * grid_layout_;

  struct CameraSlot {
    QWidget * container;
    QLabel * video_label;
    QLabel * id_label;
  };

  std::vector<CameraSlot> slots_;
  int current_mode_;
};

} // namespace dt_viz