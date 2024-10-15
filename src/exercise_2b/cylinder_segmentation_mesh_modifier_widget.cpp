#include <noether_roscon_2024/exercise_2b/cylinder_segmentation_mesh_modifier_widget.h>

// Include the implementation of the mesh modifier itself
#include <noether_roscon_2024/exercise_2b/cylinder_segmentation_mesh_modifier.h>

// Include for YAML getEntry utility
#include <noether_gui/utils.h>

// Includes for the UI
#include <QFormLayout>
#include <QDoubleSpinBox>
#include <QSpinBox>

namespace noether
{
CylinderSegmentationMeshModifierWidget::CylinderSegmentationMeshModifierWidget(QWidget* parent)
  : MeshModifierWidget(parent)
  , min_radius_(new QDoubleSpinBox(this))
  , max_radius_(new QDoubleSpinBox(this))
  , distance_threshold_(new QDoubleSpinBox(this))
  , axis_threshold_(new QDoubleSpinBox(this))
  , normal_distance_weight_(new QDoubleSpinBox(this))
  , min_vertices_(new QSpinBox(this))
  , max_cylinders_(new QSpinBox(this))
  , max_iterations_(new QSpinBox(this))
{
  // Create the UI for the widget

  // Create a form layout for the UI
  auto layout = new QFormLayout(this);

  // Add each parameter as a new row with a label about the parameter and a widget for adjusting the parameter (e.g., a spin box)

  // Min radius
  {
    // TODO: configure reasonable default values (API reference: https://doc.qt.io/qt-5/qdoublespinbox.html)

    layout->addRow("Min Radius (m)", min_radius_);
  }

  // Max radius
  {
    // TODO: configure reasonable default values (API reference: https://doc.qt.io/qt-5/qdoublespinbox.html)

    layout->addRow("Max Radius (m)", max_radius_);
  }

  // Distance threshold
  {
    // TODO: configure reasonable default values (API reference: https://doc.qt.io/qt-5/qdoublespinbox.html)

    layout->addRow("Distance threshold (m)", distance_threshold_);
  }

  // Axis threshold
  {
    // TODO: configure reasonable default values (API reference: https://doc.qt.io/qt-5/qdoublespinbox.html)

    layout->addRow("Axis threshold (rad)", axis_threshold_);
  }

  // Normal distance weight
  {
    // TODO: configure reasonable default values (API reference: https://doc.qt.io/qt-5/qdoublespinbox.html)

    layout->addRow("Normal distance weight", normal_distance_weight_);
  }

  // Min vertices
  {
    // TODO: configure reasonable default values (API reference: https://doc.qt.io/qt-5/qspinbox.html)

    layout->addRow("Min vertices", min_vertices_);
  }

  // Max cylinders
  {
    // TODO: configure reasonable default values (API reference: https://doc.qt.io/qt-5/qspinbox.html)

    layout->addRow("Max cylinders", max_cylinders_);
  }

  // Max iterations
  {
    // TODO: configure reasonable default values (API reference: https://doc.qt.io/qt-5/qspinbox.html)

    layout->addRow("Max iterations", max_iterations_);
  }
}

MeshModifier::ConstPtr CylinderSegmentationMeshModifierWidget::create() const
{
  // TODO: Extract the required parameters from the UI

  // TODO: Create an instance of our custom cylinder segmentation mesh modifier
}

void CylinderSegmentationMeshModifierWidget::configure(const YAML::Node& config)
{
  min_radius_->setValue(getEntry<double>(config, "min_radius"));
  max_radius_->setValue(getEntry<double>(config, "max_radius"));
  distance_threshold_->setValue(getEntry<double>(config, "distance_threshold"));
  axis_threshold_->setValue(getEntry<double>(config, "axis_threshold"));
  normal_distance_weight_->setValue(getEntry<double>(config, "normal_distance_weight"));
  min_vertices_->setValue(getEntry<int>(config, "min_vertices"));
  max_cylinders_->setValue(getEntry<int>(config, "max_cylinders"));
  max_iterations_->setValue(getEntry<int>(config, "max_iterations"));
}

void CylinderSegmentationMeshModifierWidget::save(YAML::Node& config) const
{
  config["min_radius"] = min_radius_->value();
  config["max_radius"] = max_radius_->value();
  config["distance_threshold"] = distance_threshold_->value();
  config["axis_threshold"] = axis_threshold_->value();
  config["normal_distance_weight"] = normal_distance_weight_->value();
  config["min_vertices"] = min_vertices_->value();
  config["max_cylinders"] = max_cylinders_->value();
  config["max_iterations"] = max_iterations_->value();
}

} // namespace noether

