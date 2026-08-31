#include "dt_viz/ui/camera_grid_widget.hpp"
#include <QVBoxLayout>
#include <QPixmap>

namespace dt_viz {

CameraGridWidget::CameraGridWidget(QWidget * parent)
: QWidget(parent),
  grid_layout_(new QGridLayout(this)),
  current_mode_(4)
{
  setupUi();
  setLayoutMode(4); // Starts by default with the full 4-camera grid.
}

void CameraGridWidget::setupUi() {
  grid_layout_->setContentsMargins(5, 5, 5, 5);
  grid_layout_->setSpacing(8);

  // Pre-initializes 4 fixed memory slots.
  slots_.resize(4);
  for (int i = 0; i < 4; ++i) {
    auto * container = new QWidget(this);
    container->setStyleSheet(
      "QWidget {"
      "  background-color: #111827;"
      "  border: 2px solid #374151;"
      "  border-radius: 6px;"
      "}"
    );

    auto * layout = new QVBoxLayout(container);
    layout->setContentsMargins(4, 4, 4, 4);

    // Camera ID label (Located at the top of the slot)
    auto * id_label = new QLabel(QString("Câmera %1: [Offline]").arg(i + 1), container);
    id_label->setStyleSheet("color: #9ca3af; font-weight: bold; border: none; background: transparent;");
    
    // Video Label (Will display the processed image)
    auto * video_label = new QLabel(container);
    video_label->setAlignment(Qt::AlignCenter);
    video_label->setStyleSheet("color: #6b7280; border: none; background: #000000;");
    video_label->setText("Aguardando Sinal...");

    layout->addWidget(id_label);
    layout->addWidget(video_label, 1);

    slots_[i] = {container, video_label, id_label};
  }
}

void CameraGridWidget::setLayoutMode(int mode) {
  current_mode_ = mode;

  // Hide all the slots first.
  for (int i = 0; i < 4; ++i) {
    grid_layout_->removeWidget(slots_[i].container);
    slots_[i].container->hide();
  }

  // Reorganizes the grid according to the selected mode.
  if (mode == 1) {
    slots_[0].container->show();
    grid_layout_->addWidget(slots_[0].container, 0, 0);
  } 
  else if (mode == 2) {
    slots_[0].container->show();
    slots_[1].container->show();
    grid_layout_->addWidget(slots_[0].container, 0, 0);
    grid_layout_->addWidget(slots_[1].container, 0, 1); // Lado a lado
  } 
  else { // 4 cameras (Grid 2x2)
    for (int i = 0; i < 4; ++i) {
      slots_[i].container->show();
      grid_layout_->addWidget(slots_[i].container, i / 2, i % 2);
    }
  }
}

void CameraGridWidget::updateCameraFrame(int slot_index, const QString & camera_id, const QImage & image) {
  if (slot_index < 0 || slot_index >= static_cast<int>(slots_.size())) {
    return;
  }

  if (image.isNull()) {
    slots_[slot_index].id_label->setText(QString("Slot %1: [%2] (Sem Frame)").arg(slot_index + 1).arg(camera_id));
    slots_[slot_index].video_label->setText("Sem Sinal de Vídeo");
    return;
  }

  slots_[slot_index].id_label->setText(QString("Câmera ID: %1").arg(camera_id));
  
  // Resizes while maintaining the exact aspect ratio to fit the slot label.
  QPixmap pixmap = QPixmap::fromImage(image);
  slots_[slot_index].video_label->setPixmap(
    pixmap.scaled(slots_[slot_index].video_label->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation)
  );
}

} // namespace dt_viz