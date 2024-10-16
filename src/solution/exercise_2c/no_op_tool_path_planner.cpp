#include <noether_roscon_2024/exercise_2c/no_op_tool_path_planner.h>

namespace noether
{
ToolPaths NoOpToolPathPlanner::plan(const pcl::PolygonMesh& mesh) const { return {}; }

}  // namespace noether
