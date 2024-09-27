#include <noether_roscon_2024/exercise_2a/cylinder_segmentation_mesh_modifier_widget.h>

// Include the implementation of the mesh modifier itself
#include <noether_roscon_2024/exercise_2a/cylinder_segmentation_mesh_modifier.h>

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
    min_radius_->setMinimum(0.0);
    min_radius_->setValue(0.050);
    min_radius_->setDecimals(3);
    min_radius_->setSingleStep(0.010);
    layout->addRow("Min Radius (m)", min_radius_);
  }

  // Max radius
  {
    max_radius_->setMinimum(0.0);
    max_radius_->setValue(0.100);
    max_radius_->setDecimals(3);
    max_radius_->setSingleStep(0.010);
    layout->addRow("Max Radius (m)", max_radius_);
  }

  // Distance threshold
  {
    distance_threshold_->setMinimum(0.0);
    distance_threshold_->setValue(0.010);
    distance_threshold_->setDecimals(3);
    distance_threshold_->setSingleStep(0.001);
    layout->addRow("Distance threshold (m)", distance_threshold_);
  }

  // Axis threshold
  {
    // TODO: use degrees instead of radians in the UI
    axis_threshold_->setMinimum(0.0);
    axis_threshold_->setValue(10.0 * M_PI / 180.0);
    axis_threshold_->setDecimals(3);
    axis_threshold_->setSingleStep(0.02);
    layout->addRow("Axis threshold (rad)", axis_threshold_);
  }

  // Normal distance weight
  {
    normal_distance_weight_->setMinimum(0.0);
    normal_distance_weight_->setMaximum(1.0);
    normal_distance_weight_->setValue(0.5);
    normal_distance_weight_->setDecimals(3);
    normal_distance_weight_->setSingleStep(0.1);
    layout->addRow("Normal distance weight", normal_distance_weight_);
  }

  // Min vertices
  {
    min_vertices_->setMinimum(1);
    min_vertices_->setValue(1);
    layout->addRow("Min vertices", min_vertices_);
  }

  // Max cylinders
  {
    max_cylinders_->setMinimum(-1);
    max_cylinders_->setValue(-1);
    layout->addRow("Max cylinders", max_cylinders_);
  }

  // Max iterations
  {
    max_iterations_->setMinimum(1);
    max_iterations_->setMaximum(10'000);
    max_iterations_->setValue(100);
    layout->addRow("Max iterations", max_iterations_);
  }
}

MeshModifier::ConstPtr CylinderSegmentationMeshModifierWidget::create() const
{
  // Extract the required parameters from the UI
  float min_radius = min_radius_->value();
  float max_radius = max_radius_->value();
  float distance_threshold = distance_threshold_->value();
  float axis_threshold = axis_threshold_->value();  // TODO: convert from degrees to radians once the UI is updated
  float normal_distance_weight = normal_distance_weight_->value();
  unsigned min_vertices = min_vertices_->value();
  unsigned max_cylinders = max_cylinders_->value();
  unsigned max_iterations = max_iterations_->value();

  // Create an instance of our custom cylinder segmentation mesh modifier
  return std::make_unique<CylinderSegmentationMeshModifier>(min_radius,
                                                            max_radius,
                                                            distance_threshold,
                                                            axis_threshold,
                                                            normal_distance_weight,
                                                            min_vertices,
                                                            max_cylinders,
                                                            max_iterations);
}

void CylinderSegmentationMeshModifierWidget::configure(const YAML::Node& config)
{
  min_radius_->setValue(getEntry<double>(config, "min_radius"));
  max_radius_->setValue(getEntry<double>(config, "max_radius"));
  distance_threshold_->setValue(getEntry<double>(config, "distance_threshold"));
  axis_threshold_->setValue(getEntry<double>(config, "axis_threshold"));  // TODO: convert from degrees to radians once the UI is updated
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
  config["axis_threshold"] = axis_threshold_->value();  // TODO: convert from degrees to radians once the UI is updated
  config["normal_distance_weight"] = normal_distance_weight_->value();
  config["min_vertices"] = min_vertices_->value();
  config["max_cylinders"] = max_cylinders_->value();
  config["max_iterations"] = max_iterations_->value();
}

} // namespace noether

