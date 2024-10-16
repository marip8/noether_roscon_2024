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
  : ToolPathModifierWidget(parent)
  , camera_standoff_(new QDoubleSpinBox(this))
{
  // Create the UI

  // Create a form layout for the elements of the UI
  auto* layout = new QFormLayout(this);

  // Camera standoff
  {
    // TODO: configure the spin box with useful default values (API reference: https://doc.qt.io/qt-5/qdoublespinbox.html)

    // Add the widget to the layout
    layout->addRow("Camera standoff (m)", camera_standoff_);
  }
}

ToolPathModifier::ConstPtr CameraStandoffToolPathModifierWidget::create() const
{
  // Create an camera standoff pose and initialize as identity
  Eigen::Isometry3d camera_standoff(Eigen::Isometry3d::Identity());

  // TODO: Translate the pose out in the +z-axis by the camera standoff value

  // TODO: Rotate about the x-axis by 180 degrees, such that the z-axis of the pose now points back at the mesh

  // TODO: Create the offset modifier given the camera standoff pose

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

} // namespace noether
