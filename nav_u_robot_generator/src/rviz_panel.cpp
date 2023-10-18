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

#include <nav_u_robot_generator/rviz_panel.hpp>
#include <QApplication>
#include <rviz_rendering/render_window.hpp>
#include <rviz_common/tool_manager.hpp>

namespace nav_u_robot_generator
{
RVizPanel::RVizPanel(QWidget* parent,
                     const rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr& node_abstraction)
  : QWidget(parent), parent_(parent), node_abstraction_(node_abstraction),
    node_(node_abstraction_.lock()->get_raw_node())

{
  logger_ = std::make_shared<rclcpp::Logger>(node_->get_logger().get_child("RVizPanel"));
}
void RVizPanel::initialize()
{
  // Create rviz frame
  rviz_render_panel_ = new rviz_common::RenderPanel();
  rviz_render_panel_->setMinimumWidth(200);
  rviz_render_panel_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);

  QApplication::processEvents();
  rviz_render_panel_->getRenderWindow()->initialize();

  rviz_manager_ =
      new rviz_common::VisualizationManager(rviz_render_panel_, node_abstraction_, this, node_->get_clock());
  rviz_render_panel_->initialize(rviz_manager_);
  rviz_manager_->initialize();
  rviz_manager_->startUpdate();

  auto tm = rviz_manager_->getToolManager();
  tm->addTool("rviz_default_plugins/MoveCamera");

  grid_display_ = new rviz_default_plugins::displays::GridDisplay();
  rviz_manager_->addDisplay(grid_display_, true);

  robot_display_ = new rviz_default_plugins::displays::RobotModelDisplay();
  rviz_manager_->addDisplay(robot_display_, true);

  // Set the fixed and target frame
  updateFixedFrame();

  // Set robot description
  robot_display_->subProp("Description Topic")->setValue(QString::fromStdString("robot_description"));

  // Zoom into robot
  rviz_common::ViewController* view = rviz_manager_->getViewManager()->getCurrent();
  view->subProp("Distance")->setValue(4.0f);

  // Add Rviz to Planning Groups Widget
  QVBoxLayout* rviz_layout = new QVBoxLayout();
  rviz_layout->addWidget(rviz_render_panel_);

  // visual / collision buttons
  auto btn_layout = new QHBoxLayout();
  rviz_layout->addLayout(btn_layout);

  QCheckBox* btn;
  btn_layout->addWidget(btn = new QCheckBox("visual"), 0);
  btn->setChecked(true);
  connect(btn, &QCheckBox::toggled,
          [this](bool checked) { robot_display_->subProp("Visual Enabled")->setValue(checked); });

  btn_layout->addWidget(btn = new QCheckBox("collision"), 1);
  btn->setChecked(false);
  connect(btn, &QCheckBox::toggled,
          [this](bool checked) { robot_display_->subProp("Collision Enabled")->setValue(checked); });
  setLayout(rviz_layout);
}
RVizPanel::~RVizPanel()
{
  if (rviz_manager_ != nullptr)
    rviz_manager_->removeAllDisplays();
  if (rviz_render_panel_ != nullptr)
    delete rviz_render_panel_;
  if (rviz_manager_ != nullptr)
    delete rviz_manager_;
}

void RVizPanel::updateFixedFrame()
{
  rviz_manager_->setFixedFrame(QString::fromStdString("base_footprint"));
}
}  // namespace nav_u_robot_generator
