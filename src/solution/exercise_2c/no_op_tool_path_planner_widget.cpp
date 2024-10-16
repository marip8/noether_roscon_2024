#include <noether_roscon_2024/exercise_2c/no_op_tool_path_planner_widget.h>
#include <noether_roscon_2024/exercise_2c/no_op_tool_path_planner.h>

namespace noether
{
ToolPathPlanner::ConstPtr NoOpToolPathPlannerWidget::create() const { return std::make_unique<NoOpToolPathPlanner>(); }

}  // namespace noether
