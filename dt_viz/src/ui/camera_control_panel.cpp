#include "dt_viz/ui/camera_control_panel.hpp"
#include <QHBoxLayout>
#include <QGroupBox>

namespace dt_viz {

CameraControlPanel::CameraControlPanel(QWidget * parent)
: QWidget(parent)
{
  setupUi();
}

void CameraControlPanel::setupUi() {
  setFixedWidth(260);

  setStyleSheet(
    "QWidget {"
    "  background-color: #f4f6f8;"
    "  color: #1f2933;"
    "}"
    "QLabel {"
    "  padding: 2px;"
    "  font-weight: bold;"
    "}"
  );

  auto * layout = new QVBoxLayout(this);
  layout->setContentsMargins(8, 8, 8, 8);
  layout->setSpacing(12);

  // --- Grid Layout ---
  auto * layout_group = new QGroupBox("Grid Layout Mode", this);
  auto * layout_vbox = new QVBoxLayout(layout_group);

  layout_selector_ = new QComboBox(this);
  layout_selector_->addItem("4 Câmeras (Grid 2x2)", 4);
  layout_selector_->addItem("2 Câmeras (Lado a Lado)", 2);
  layout_selector_->addItem("1 Câmera (Centralizada)", 1);
  layout_selector_->setCurrentIndex(0); // Default 4

  layout_vbox->addWidget(layout_selector_);
  layout->addWidget(layout_group);

  // --- Slots Mapping ---
  auto * mapping_group = new QGroupBox("Slot Mappings", this);
  auto * mapping_vbox = new QVBoxLayout(mapping_group);

  slot_selectors_.resize(4);
  for (int i = 0; i < 4; ++i) {
    auto * hbox = new QHBoxLayout();
    auto * label = new QLabel(QString("Slot %1:").arg(i + 1), this);
    
    auto * combo = new QComboBox(this);
    combo->addItem("-- Nenhuma --");
    
    // Connects the individual selection of each slot to the dedicated method.
    connect(combo, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, [this, i](int index) {
              onSlotCameraChanged(i, index);
            });

    slot_selectors_[i] = combo;
    
    hbox->addWidget(label);
    hbox->addWidget(combo, 1);
    mapping_vbox->addLayout(hbox);
  }

  layout->addWidget(mapping_group);
  layout->addStretch();

  // Layout selector main connection
  connect(layout_selector_, QOverload<int>::of(&QComboBox::currentIndexChanged),
          this, &CameraControlPanel::onLayoutSelectionChanged);
}

void CameraControlPanel::onLayoutSelectionChanged(int index) {
  int mode = layout_selector_->itemData(index).toInt();
  emit layoutModeChanged(mode);
}

void CameraControlPanel::updateAvailableCameras(const std::vector<std::string> & active_ids) {
  // Avoids reprocessing if the IDs are the same.
  if (active_ids == cached_active_ids_) {
    return;
  }
  cached_active_ids_ = active_ids;

  // Updates each slot combobox, maintaining the current selection if possible.
  for (int i = 0; i < 4; ++i) {
    QString current_selection = slot_selectors_[i]->currentText();
    
    slot_selectors_[i]->blockSignals(true);
    slot_selectors_[i]->clear();
    slot_selectors_[i]->addItem("-- Nenhuma --");

    for (const auto & id : active_ids) {
      slot_selectors_[i]->addItem(QString::fromStdString(id));
    }

    // Restores the previous selection if it still exists in the new list.
    int idx = slot_selectors_[i]->findText(current_selection);
    if (idx != -1) {
      slot_selectors_[i]->setCurrentIndex(idx);
    } else {
      slot_selectors_[i]->setCurrentIndex(0);
    }
    slot_selectors_[i]->blockSignals(false);
  }
}

void CameraControlPanel::onSlotCameraChanged(int slot_index, int combo_index) {
  if (slot_index < 0 || slot_index >= static_cast<int>(slot_selectors_.size())) {
    return;
  }

  QComboBox * combo = slot_selectors_[slot_index];
  QString cam_id = (combo_index > 0) ? combo->currentText() : "";
  
  emit cameraAssigned(slot_index, cam_id);
}

} // namespace dt_viz