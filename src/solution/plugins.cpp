#include <yaml-cpp/yaml.h>
#include <noether_gui/plugin_interface.h>

// Includes for the widgets to be used for the plugin
#include <noether_roscon_2024/exercise_2a/camera_standoff_tool_path_modifier_widget.h>
#include <noether_roscon_2024/exercise_2b/cylinder_segmentation_mesh_modifier_widget.h>
#include <noether_roscon_2024/exercise_2c/no_op_tool_path_planner_widget.h>

namespace noether
{
struct CameraStandoffToolPathModifierWidgetPlugin : public ToolPathModifierWidgetPlugin
{
  QWidget* create(QWidget* parent = nullptr, const YAML::Node& config = {}) const override
  {
    // Create the widget for the camera standoff tool path modifier
    auto* widget = new CameraStandoffToolPathModifierWidget(parent);

    // Attempt to configure the widget
    if (!config.IsNull())
      widget->configure(config);

    return widget;
  }
};

struct CylinderSegmentationMeshModifierWidgetPlugin : public MeshModifierWidgetPlugin
{
  QWidget* create(QWidget* parent = nullptr, const YAML::Node& config = {}) const override
  {
    // Create the widget for the cylinder segmentation mesh modifier
    auto* widget = new CylinderSegmentationMeshModifierWidget(parent);

    // Attempt to configure the widget
    if (!config.IsNull())
      widget->configure(config);

    return widget;
  }
};

struct NoOpToolPathPlannerWidgetPlugin : public ToolPathPlannerWidgetPlugin
{
  QWidget* create(QWidget* parent = nullptr, const YAML::Node& config = {}) const override
  {
    auto* widget = new NoOpToolPathPlannerWidget(parent);

    // Attempt to configure the widget
    if (!config.IsNull())
      widget->configure(config);

    return widget;
  }
};

}  // namespace noether

// Export the plugin
EXPORT_TOOL_PATH_MODIFIER_WIDGET_PLUGIN(noether::CameraStandoffToolPathModifierWidgetPlugin, CameraStandoff)
EXPORT_MESH_MODIFIER_WIDGET_PLUGIN(noether::CylinderSegmentationMeshModifierWidgetPlugin, CylinderSegmentation)
EXPORT_TPP_WIDGET_PLUGIN(noether::NoOpToolPathPlannerWidgetPlugin, NoOpPlanner)
