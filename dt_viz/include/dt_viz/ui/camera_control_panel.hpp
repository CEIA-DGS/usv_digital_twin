#pragma once

#include <QWidget>
#include <QComboBox>
#include <QVBoxLayout>
#include <QLabel>
#include <vector>
#include <string>

namespace dt_viz {

/**
 * @brief Side control panel for configuring camera layout modes and mapping camera IDs to slots.
 */
class CameraControlPanel : public QWidget {
  Q_OBJECT

public:
  /**
   * @brief Constructs the CameraControlPanel.
   * @param parent Pointer to the parent widget.
   */
  explicit CameraControlPanel(QWidget * parent = nullptr);

  /**
   * @brief Updates the list of available camera IDs detected by the core.
   * @param active_ids Vector of active camera identification strings.
   */
  void updateAvailableCameras(const std::vector<std::string> & active_ids);

signals:
  /**
   * @brief Emitted when the user changes the layout mode (1, 2, or 4 slots).
   */
  void layoutModeChanged(int mode);

  /**
   * @brief Emitted when a specific grid slot is mapped to a camera ID.
   */
  void cameraAssigned(int slot_index, const QString & camera_id);

private slots:
  void onLayoutSelectionChanged(int index);
  void onSlotCameraChanged(int slot_index, int combo_index);

private:
  void setupUi();

  QComboBox * layout_selector_;
  std::vector<QComboBox*> slot_selectors_;
  std::vector<std::string> cached_active_ids_;
};

} // namespace dt_viz