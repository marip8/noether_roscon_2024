#pragma once

#include <noether_gui/widgets.h>
#include <noether_tpp/core/tool_path_modifier.h>

// Forward declare Qt GUI classes
class QDoubleSpinBox;

namespace noether
{
class CameraStandoffToolPathModifierWidget : public ToolPathModifierWidget
{
public:
  CameraStandoffToolPathModifierWidget(QWidget* parent = nullptr);

  ToolPathModifier::ConstPtr create() const override;
  void configure(const YAML::Node& config) override;
  void save(YAML::Node& config) const override;

protected:
  // UI elements
  QDoubleSpinBox* camera_standoff_;
};

} // namespace
