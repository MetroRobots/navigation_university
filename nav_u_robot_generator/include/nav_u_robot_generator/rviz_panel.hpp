/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2023, Metro Robots
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Metro Robots nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: David V. Lu!! */

#pragma once

#include <rviz_common/render_panel.hpp>
#include <rviz_common/window_manager_interface.hpp>
#include <rviz_common/visualization_manager.hpp>
#include <rviz_common/view_manager.hpp>
#include <rviz_common/view_controller.hpp>
#include <rviz_default_plugins/displays/grid/grid_display.hpp>
#include <rviz_default_plugins/displays/robot_model/robot_model_display.hpp>
#include <QWidget>
#include <QCheckBox>
#include <QHBoxLayout>
#include <QVBoxLayout>

namespace nav_u_robot_generator
{
class RVizPanel : public QWidget, public rviz_common::WindowManagerInterface
{
  Q_OBJECT
public:
  RVizPanel(QWidget* parent, const rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr& node_abstraction);
  ~RVizPanel() override;

  void initialize();
  void updateFixedFrame();

  QWidget* getParentWindow() override
  {
    return parent_;
  }

  rviz_common::PanelDockWidget* addPane(const QString& /*name*/, QWidget* /*pane*/,
                                        Qt::DockWidgetArea /*area*/ = Qt::LeftDockWidgetArea,
                                        bool /*floating*/ = true) override
  {
    // Stub for now...just to define the WindowManagerInterface methods
    return nullptr;
  }

  void setStatus(const QString& /*message*/) override
  {
    // Stub for now...just to define the WindowManagerInterface methods
  }

protected:
  QWidget* parent_;
  rviz_common::RenderPanel* rviz_render_panel_{nullptr};
  rviz_common::VisualizationManager* rviz_manager_{nullptr};
  rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr node_abstraction_;
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<rclcpp::Logger> logger_;

  rviz_default_plugins::displays::GridDisplay* grid_display_{nullptr};
  rviz_default_plugins::displays::RobotModelDisplay* robot_display_{nullptr};
};
}  // namespace nav_u_robot_generator
