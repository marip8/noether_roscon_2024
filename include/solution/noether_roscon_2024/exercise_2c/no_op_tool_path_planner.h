#pragma once

#include <noether_tpp/core/tool_path_planner.h>

namespace noether
{
class NoOpToolPathPlanner : public ToolPathPlanner
{
public:
  using ToolPathPlanner::ToolPathPlanner;

  ToolPaths plan(const pcl::PolygonMesh& mesh) const override;
};

}  // namespace noether
