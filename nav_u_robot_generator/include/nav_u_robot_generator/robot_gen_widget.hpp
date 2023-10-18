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
#include <rclcpp/node.hpp>
#include <nav_u_robot_generator/templates.hpp>
#include <rcl_interfaces/srv/set_parameters.hpp>
#include <QLineEdit>
#include <QHBoxLayout>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QSpinBox>
#include <QSlider>
#include <QLabel>
#include <QWidget>
#include <QPushButton>

namespace nav_u_robot_generator
{
class QDoubleSlider : public QSlider
{
public:
  QDoubleSlider(QWidget* parent, double min, double max) : QSlider(parent), min_(min), max_(max)
  {
  }

  double double_value() const
  {
    double value_i = value();
    int min_i = minimum(), max_i = maximum();
    double ratio = (value_i - min_i) / (max_i - min_i);
    return min_ + ratio * (max_ - min_);
  }

  void setDoubleValue(double d)
  {
    double ratio = (d - min_) / (max_ - min_);
    int min_i = minimum(), max_i = maximum();
    int value_i = static_cast<int>(min_i + ratio * (max_i - min_i));
    setValue(value_i);
  }

protected:
  double min_, max_;
};

class RobotGenWidget : public QWidget
{
  Q_OBJECT
public:
  void initialize(const rclcpp::Node::SharedPtr& parent_node, QWidget* parent_widget);

protected Q_SLOTS:
  void editedStrings();
  void shapeChanged(int);
  void changeNoiseLevel(int);
  void colorChanged(int);
  void geometryChanged(double);
  void speedChange(double);
  void browse();
  void generate();

protected:
  void initializeValue(QLineEdit* widget, const std::string& param_name, const std::string& default_value);
  void initializeValue(QComboBox* widget, const std::string& param_name, const std::string& default_value);
  void initializeValue(QDoubleSpinBox* widget, const std::string& param_name, double default_value);
  void initializeValue(QSpinBox* widget, const std::string& param_name, int default_value);
  void initializeValue(QDoubleSlider* widget, const std::string& param_name, double default_value);

  std::vector<TemplateVariable> getVariables();
  void generateURDF();
  QHBoxLayout* addRow(const std::string& text, QWidget* widget);

  rclcpp::Node::SharedPtr node_;
  int shape_;

  rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr set_params_srv_;

  std::filesystem::path this_package_;
  std::string package_name_, config_package_name_;

  QVBoxLayout* main_layout_;
  QPushButton *browse_button_, *generate_button_;
  QLineEdit *robot_edit_, *name_edit_, *email_edit_, *path_edit_;
  QComboBox *shape_edit_, *sensor_edit_, *body_color_edit_, *wheel_color_edit_;
  QDoubleSpinBox *width_edit_, *height_edit_, *length_edit_, *wheel_height_edit_, *sensor_range_edit_, *drive_vel_edit_,
      *turn_vel_edit_;
  QSpinBox* samples_edit_;
  QDoubleSlider *noise_edit_, *odom_noise_edit_, *laser_noise_edit_, *sensor_offset_edit_;
};
}  // namespace nav_u_robot_generator
