#pragma once

#include <noether_gui/widgets.h>
#include <noether_tpp/core/mesh_modifier.h>

// Forward declare Qt UI classes
class QDoubleSpinBox;
class QSpinBox;

namespace noether
{
class CylinderSegmentationMeshModifierWidget : public MeshModifierWidget
{
public:
  CylinderSegmentationMeshModifierWidget(QWidget* parent = nullptr);

  MeshModifier::ConstPtr create() const override;

  void configure(const YAML::Node& config) override;
  void save(YAML::Node& config) const override;

protected:
  // UI elements needed to configure a cylinder segmentation mesh modifier
  QDoubleSpinBox* min_radius_;
  QDoubleSpinBox* max_radius_;
  QDoubleSpinBox* distance_threshold_;
  QDoubleSpinBox* axis_threshold_;
  QDoubleSpinBox* normal_distance_weight_;
  QSpinBox* min_vertices_;
  QSpinBox* max_cylinders_;
  QSpinBox* max_iterations_;
};

} // namespace noether
