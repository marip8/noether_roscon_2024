#pragma once

#include <noether_gui/widgets.h>
#include <noether_tpp/core/tool_path_planner.h>

namespace noether
{
class NoOpToolPathPlannerWidget : public ToolPathPlannerWidget
{
public:
  using ToolPathPlannerWidget::ToolPathPlannerWidget;

  ToolPathPlanner::ConstPtr create() const override;
};

} // namespace noether
