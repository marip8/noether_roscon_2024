#include <yaml-cpp/yaml.h>
#include <noether_gui/plugin_interface.h>

// Includes for the widgets to be used for the plugin

// TODO: Uncomment in exercise 2a
// #include <noether_roscon_2024/exercise_2a/camera_standoff_tool_path_modifier_widget.h>

// TODO: Uncomment in exercise 2b
// #include <noether_roscon_2024/exercise_2b/cylinder_segmentation_mesh_modifier_widget.h>

// TODO: Uncomment in exercise 2c
// #include <noether_roscon_2024/exercise_2c/no_op_tool_path_planner_widget.h>

namespace noether
{
// TODO: Uncomment in exercise 2a
// struct CameraStandoffToolPathModifierWidgetPlugin : public ToolPathModifierWidgetPlugin
// {
//   QWidget* create(QWidget* parent = nullptr, const YAML::Node& config = {}) const override
//   {
//     // Create the widget for the camera standoff tool path modifier
//     auto* widget = new CameraStandoffToolPathModifierWidget(parent);

//     // Attempt to configure the widget
//     if (!config.IsNull())
//       widget->configure(config);

//     return widget;
//   }
// };

// TODO: Uncomment in exercise 2b
// struct CylinderSegmentationMeshModifierWidgetPlugin : public MeshModifierWidgetPlugin
// {
//   QWidget* create(QWidget* parent = nullptr, const YAML::Node& config = {}) const override
//   {
//     // Create the widget for the cylinder segmentation mesh modifier
//     auto* widget = new CylinderSegmentationMeshModifierWidget(parent);

//     // Attempt to configure the widget
//     if (!config.IsNull())
//       widget->configure(config);

//     return widget;
//   }
// };

// TODO: Uncomment in exercise 2c
// struct NoOpToolPathPlannerWidgetPlugin : public ToolPathPlannerWidgetPlugin
// {
//   QWidget* create(QWidget* parent = nullptr, const YAML::Node& config = {}) const override
//   {
//     auto* widget = new NoOpToolPathPlannerWidget(parent);

//     // Attempt to configure the widget
//     if (!config.IsNull())
//       widget->configure(config);

//     return widget;
//   }
// };

}  // namespace noether

// Export the plugin

// TODO: Uncomment in exercise 2a
// EXPORT_TOOL_PATH_MODIFIER_WIDGET_PLUGIN(noether::CameraStandoffToolPathModifierWidgetPlugin, CameraStandoff)

// TODO: Uncomment in exercise 2b
// EXPORT_MESH_MODIFIER_WIDGET_PLUGIN(noether::CylinderSegmentationMeshModifierWidgetPlugin, CylinderSegmentation)

// TODO: Uncomment in exercise 2c
// EXPORT_TPP_WIDGET_PLUGIN(noether::NoOpToolPathPlannerWidgetPlugin, NoOpPlanner)
