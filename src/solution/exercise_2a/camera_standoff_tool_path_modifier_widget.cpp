#include <noether_roscon_2024/exercise_2a/camera_standoff_tool_path_modifier_widget.h>

// Include existing `OffsetToolPathModifier` from `noether_tpp`
#include <noether_tpp/tool_path_modifiers/offset_modifier.h>

// Include YAML getEntry utility
#include <noether_gui/utils.h>

// Includes for the UI elements
#include <QFormLayout>
#include <QDoubleSpinBox>

namespace noether
{
CameraStandoffToolPathModifierWidget::CameraStandoffToolPathModifierWidget(QWidget* parent)
  : ToolPathModifierWidget(parent), camera_standoff_(new QDoubleSpinBox(this))
{
  // Create the UI

  // Create a form layout for the elements of the UI
  auto* layout = new QFormLayout(this);

  // Camera standoff
  {
    camera_standoff_->setMinimum(0.0);
    camera_standoff_->setMaximum(2.0);
    camera_standoff_->setValue(0.2);
    camera_standoff_->setDecimals(3);
    camera_standoff_->setSingleStep(0.010);
    layout->addRow("Camera standoff (m)", camera_standoff_);
  }
}

ToolPathModifier::ConstPtr CameraStandoffToolPathModifierWidget::create() const
{
  // Create an camera standoff pose and initialize as identity
  Eigen::Isometry3d camera_standoff(Eigen::Isometry3d::Identity());

  // Translate the pose out in the +z-axis by the camera standoff value
  double camera_standoff_z = camera_standoff_->value();
  camera_standoff.translate(Eigen::Vector3d(0.0, 0.0, camera_standoff_z));

  // Rotate about the x-axis by 180 degrees, such that the z-axis of the pose now points back at the mesh
  camera_standoff.rotate(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()));

  // Create the offset modifier given the camera standoff pose
  return std::make_unique<OffsetModifier>(camera_standoff);
}

void CameraStandoffToolPathModifierWidget::configure(const YAML::Node& config)
{
  camera_standoff_->setValue(getEntry<double>(config, "camera_standoff"));
}

void CameraStandoffToolPathModifierWidget::save(YAML::Node& config) const
{
  config["camera_standoff"] = camera_standoff_->value();
}

}  // namespace noether
