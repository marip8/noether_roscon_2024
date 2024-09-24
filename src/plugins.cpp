#include <yaml-cpp/yaml.h>
#include <noether_gui/plugin_interface.h>
#include <noether_roscon_2024/exercise_2a/mesh_modifier_widget.h>

namespace noether
{
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

} // namespace noether

// Export the plugin
EXPORT_MESH_MODIFIER_WIDGET_PLUGIN(noether::CylinderSegmentationMeshModifierWidgetPlugin, CylinderSegmentation)
