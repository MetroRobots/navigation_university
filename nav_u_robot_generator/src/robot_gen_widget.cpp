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

#include <nav_u_robot_generator/robot_gen_widget.hpp>
#include <QHBoxLayout>
#include <QLabel>
#include <QFont>
#include <QFileDialog>
#include <QMessageBox>

namespace nav_u_robot_generator
{
QHBoxLayout* RobotGenWidget::addRow(const std::string& text, QWidget* widget)
{
  QHBoxLayout* row = new QHBoxLayout();
  QLabel* title = new QLabel(this);
  title->setText(text.c_str());
  row->addWidget(title);
  row->addWidget(widget);
  return row;
}

void RobotGenWidget::initialize(const rclcpp::Node::SharedPtr& parent_node, QWidget* parent_widget)
{
  setParent(parent_widget);
  node_ = parent_node;
  this_package_ = getSharePath("nav_u_robot_generator");

  set_params_srv_ = node_->create_client<rcl_interfaces::srv::SetParameters>("/robot_state_publisher/set_parameters");
  using namespace std::chrono_literals;
  while (!set_params_srv_->wait_for_service(1s))
  {
    if (!rclcpp::ok())
    {
      RCLCPP_ERROR(node_->get_logger(), "Interrupted while waiting for the service. Exiting.");
      return;
    }
    RCLCPP_INFO(node_->get_logger(), "service not available, waiting again...");
  }

  main_layout_ = new QVBoxLayout(this);
  main_layout_->setAlignment(Qt::AlignTop);
  main_layout_->setContentsMargins(0, 0, 0, 0);

  QLabel* page_title = new QLabel(this);
  page_title->setText("Navigation University Robot Generator");
  QFont page_title_font(QFont().defaultFamily(), 18, QFont::Bold);
  page_title->setFont(page_title_font);
  page_title->setWordWrap(true);
  page_title->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Maximum);
  main_layout_->addWidget(page_title);

  robot_edit_ = new QLineEdit(this);
  initializeValue(robot_edit_, "robot_name", "example_bot");
  connect(robot_edit_, SIGNAL(editingFinished()), this, SLOT(editedStrings()));
  main_layout_->addLayout(addRow("Robot Name (letters/underscores only)", robot_edit_));

  name_edit_ = new QLineEdit(this);
  initializeValue(name_edit_, "author_name", "Roberta Universidad");
  connect(name_edit_, SIGNAL(editingFinished()), this, SLOT(editedStrings()));
  main_layout_->addLayout(addRow("Your Name (for package.xml)", name_edit_));

  email_edit_ = new QLineEdit(this);
  initializeValue(email_edit_, "author_email", "tullyfoote@intrinsic.ai");
  connect(email_edit_, SIGNAL(editingFinished()), this, SLOT(editedStrings()));
  main_layout_->addLayout(addRow("Your Email (for package.xml)", email_edit_));

  shape_edit_ = new QComboBox(this);
  shape_edit_->addItem("Circle");
  shape_edit_->addItem("Square");
  shape_edit_->addItem("Rectangle");
  initializeValue(shape_edit_, "shape", "Circle");
  shape_ = shape_edit_->currentIndex();
  connect(shape_edit_, SIGNAL(currentIndexChanged(int)), this, SLOT(shapeChanged(int)));
  main_layout_->addLayout(addRow("Robot Shape", shape_edit_));

  width_edit_ = new QDoubleSpinBox(this);
  width_edit_->setRange(0.05, 1.0);
  width_edit_->setSingleStep(0.05);
  initializeValue(width_edit_, "width", 0.5);
  connect(width_edit_, SIGNAL(valueChanged(double)), this, SLOT(geometryChanged(double)));
  main_layout_->addLayout(addRow("Robot Width", width_edit_));

  length_edit_ = new QDoubleSpinBox(this);
  length_edit_->setRange(0.05, 1.0);
  length_edit_->setSingleStep(0.05);
  initializeValue(length_edit_, "length", 0.8);
  if (shape_ != 2)
  {
    length_edit_->setEnabled(false);
  }
  connect(length_edit_, SIGNAL(valueChanged(double)), this, SLOT(geometryChanged(double)));
  main_layout_->addLayout(addRow("Robot Length", length_edit_));

  height_edit_ = new QDoubleSpinBox(this);
  height_edit_->setRange(0.05, 1.0);
  height_edit_->setSingleStep(0.05);
  initializeValue(height_edit_, "height", 0.1);
  connect(height_edit_, SIGNAL(valueChanged(double)), this, SLOT(geometryChanged(double)));
  main_layout_->addLayout(addRow("Robot Height", height_edit_));

  wheel_height_edit_ = new QDoubleSpinBox(this);
  wheel_height_edit_->setRange(0.025, 0.5);
  wheel_height_edit_->setSingleStep(0.05);
  initializeValue(wheel_height_edit_, "wheel_radius", 0.1);
  connect(wheel_height_edit_, SIGNAL(valueChanged(double)), this, SLOT(geometryChanged(double)));
  main_layout_->addLayout(addRow("Wheel Radius", wheel_height_edit_));

  drive_vel_edit_ = new QDoubleSpinBox(this);
  drive_vel_edit_->setRange(0.05, 1.5);
  drive_vel_edit_->setSingleStep(0.05);
  initializeValue(drive_vel_edit_, "max_vel_x", 0.8);
  connect(drive_vel_edit_, SIGNAL(valueChanged(double)), this, SLOT(speedChange(double)));
  main_layout_->addLayout(addRow("Max Drive Speed", drive_vel_edit_));

  turn_vel_edit_ = new QDoubleSpinBox(this);
  turn_vel_edit_->setRange(0.05, 1.5);
  turn_vel_edit_->setSingleStep(0.05);
  initializeValue(turn_vel_edit_, "max_vel_theta", 0.8);
  connect(turn_vel_edit_, SIGNAL(valueChanged(double)), this, SLOT(speedChange(double)));
  main_layout_->addLayout(addRow("Max Turn Speed", turn_vel_edit_));

  /*noise_edit_ = new QDoubleSlider(this, 0.0, 0.00005);
  noise_edit_->setOrientation(Qt::Horizontal);
  initializeValue(noise_edit_, "base_noise", 0.00001);
  connect(noise_edit_, SIGNAL(valueChanged(int)), this, SLOT(changeNoiseLevel(int)));
  main_layout_->addLayout(addRow("Base Noise", noise_edit_));

  odom_noise_edit_ = new QDoubleSlider(this, 1.0, 10.0);
  odom_noise_edit_->setOrientation(Qt::Horizontal);
  initializeValue(odom_noise_edit_, "odom_mult", 1.5);
  connect(odom_noise_edit_, SIGNAL(valueChanged(int)), this, SLOT(changeNoiseLevel(int)));
  main_layout_->addLayout(addRow("Odometry Noise", odom_noise_edit_));*/

  sensor_edit_ = new QComboBox(this);
  sensor_edit_->addItem("LaserScan");
  sensor_edit_->addItem("PointCloud2");
  initializeValue(sensor_edit_, "sensor_type", "LaserScan");
  connect(sensor_edit_, SIGNAL(currentIndexChanged(int)), this, SLOT(changeNoiseLevel(int)));
  main_layout_->addLayout(addRow("Sensor Type", sensor_edit_));

  samples_edit_ = new QSpinBox(this);
  samples_edit_->setRange(10, 300);
  samples_edit_->setSingleStep(10);
  initializeValue(samples_edit_, "sensor_samples", 150);
  connect(samples_edit_, SIGNAL(valueChanged(int)), this, SLOT(changeNoiseLevel(int)));
  main_layout_->addLayout(addRow("Sensor Samples", samples_edit_));

  laser_noise_edit_ = new QDoubleSlider(this, 0.0, 0.3);
  laser_noise_edit_->setOrientation(Qt::Horizontal);
  initializeValue(laser_noise_edit_, "sensor_noise", 0.1);
  connect(laser_noise_edit_, SIGNAL(valueChanged(int)), this, SLOT(changeNoiseLevel(int)));
  main_layout_->addLayout(addRow("Sensor Noise", laser_noise_edit_));

  sensor_offset_edit_ = new QDoubleSlider(this, 0.0, 0.9);
  sensor_offset_edit_->setOrientation(Qt::Horizontal);
  initializeValue(sensor_offset_edit_, "sensor_offset", 0.0);
  connect(sensor_offset_edit_, SIGNAL(valueChanged(int)), this, SLOT(colorChanged(int)));
  main_layout_->addLayout(addRow("Sensor Offset", sensor_offset_edit_));

  sensor_range_edit_ = new QDoubleSpinBox(this);
  sensor_range_edit_->setRange(10.0, 360.0);
  sensor_range_edit_->setSingleStep(10.0);
  initializeValue(sensor_range_edit_, "sensor_range", 180.0);
  connect(sensor_range_edit_, SIGNAL(valueChanged(double)), this, SLOT(geometryChanged(double)));
  main_layout_->addLayout(addRow("Sensor Angular Range (deg)", sensor_range_edit_));

  body_color_edit_ = new QComboBox(this);
  body_color_edit_->addItem("Grey");
  body_color_edit_->addItem("DarkGrey");
  body_color_edit_->addItem("White");
  body_color_edit_->addItem("FlatBlack");
  body_color_edit_->addItem("Black");
  body_color_edit_->addItem("Red");
  body_color_edit_->addItem("RedBright");
  body_color_edit_->addItem("Green");
  body_color_edit_->addItem("Blue");
  body_color_edit_->addItem("SkyBlue");
  body_color_edit_->addItem("Yellow");
  body_color_edit_->addItem("ZincYellow");
  body_color_edit_->addItem("DarkYellow");
  body_color_edit_->addItem("Purple");
  body_color_edit_->addItem("Turquoise");
  body_color_edit_->addItem("Orange");
  body_color_edit_->addItem("Indigo");
  body_color_edit_->addItem("Gold");
  initializeValue(body_color_edit_, "body_color", "Blue");
  connect(body_color_edit_, SIGNAL(currentIndexChanged(int)), this, SLOT(colorChanged(int)));
  main_layout_->addLayout(addRow("Body Color", body_color_edit_));

  wheel_color_edit_ = new QComboBox(this);
  wheel_color_edit_->addItem("Grey");
  wheel_color_edit_->addItem("DarkGrey");
  wheel_color_edit_->addItem("White");
  wheel_color_edit_->addItem("FlatBlack");
  wheel_color_edit_->addItem("Black");
  wheel_color_edit_->addItem("Red");
  wheel_color_edit_->addItem("RedBright");
  wheel_color_edit_->addItem("Green");
  wheel_color_edit_->addItem("Blue");
  wheel_color_edit_->addItem("SkyBlue");
  wheel_color_edit_->addItem("Yellow");
  wheel_color_edit_->addItem("ZincYellow");
  wheel_color_edit_->addItem("DarkYellow");
  wheel_color_edit_->addItem("Purple");
  wheel_color_edit_->addItem("Turquoise");
  wheel_color_edit_->addItem("Orange");
  wheel_color_edit_->addItem("Indigo");
  wheel_color_edit_->addItem("Gold");
  initializeValue(wheel_color_edit_, "wheel_color", "DarkGrey");
  connect(wheel_color_edit_, SIGNAL(currentIndexChanged(int)), this, SLOT(colorChanged(int)));
  main_layout_->addLayout(addRow("Wheel Color", wheel_color_edit_));

  QHBoxLayout* last_row = new QHBoxLayout();
  QLabel* title = new QLabel(this);
  title->setText("Package Parent Directory");
  last_row->addWidget(title);

  path_edit_ = new QLineEdit(this);
  initializeValue(path_edit_, "parent_path", "");
  last_row->addWidget(path_edit_);

  browse_button_ = new QPushButton(this);
  browse_button_->setText("Explore");
  connect(browse_button_, SIGNAL(clicked()), this, SLOT(browse()));
  last_row->addWidget(browse_button_);

  main_layout_->addLayout(last_row);

  QHBoxLayout* really_last_row = new QHBoxLayout();
  generate_button_ = new QPushButton(this);
  generate_button_->setText("Generate");
  connect(generate_button_, SIGNAL(clicked()), this, SLOT(generate()));
  really_last_row->addWidget(generate_button_);
  main_layout_->addLayout(really_last_row);
  generateURDF();
}

void RobotGenWidget::initializeValue(QLineEdit* widget, const std::string& param_name, const std::string& default_value)
{
  std::string value = node_->get_parameter_or(param_name, default_value);
  widget->setText(value.c_str());
}
void RobotGenWidget::initializeValue(QComboBox* widget, const std::string& param_name, const std::string& default_value)
{
  std::string value = node_->get_parameter_or(param_name, default_value);
  int my_index = widget->findText(value.c_str());
  if (my_index >= 0)
    widget->setCurrentIndex(my_index);
}
void RobotGenWidget::initializeValue(QDoubleSpinBox* widget, const std::string& param_name, double default_value)
{
  double value = node_->get_parameter_or(param_name, default_value);
  widget->setValue(value);
}
void RobotGenWidget::initializeValue(QSpinBox* widget, const std::string& param_name, int default_value)
{
  int value = node_->get_parameter_or(param_name, default_value);
  widget->setValue(value);
}
void RobotGenWidget::initializeValue(QDoubleSlider* widget, const std::string& param_name, double default_value)
{
  double value = node_->get_parameter_or(param_name, default_value);
  widget->setDoubleValue(value);
}

void RobotGenWidget::editedStrings()
{
  generateURDF();
}

void RobotGenWidget::changeNoiseLevel(int)
{
}

void RobotGenWidget::geometryChanged(double)
{
  generateURDF();
}

void RobotGenWidget::shapeChanged(int shape)
{
  shape_ = shape;
  if (shape == 2)
  {
    length_edit_->setEnabled(true);
  }
  else
  {
    length_edit_->setEnabled(false);
  }
  generateURDF();
}

void RobotGenWidget::colorChanged(int)
{
  generateURDF();
}

void RobotGenWidget::speedChange(double)
{
}

void RobotGenWidget::browse()
{
  QString path = QFileDialog::getExistingDirectory(this, "Open Parent Directory", path_edit_->text(),
                                                   QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);

  // check they did not press cancel
  if (!path.isNull())
    path_edit_->setText(path);
}

std::vector<TemplateVariable> RobotGenWidget::getVariables()
{
  std::vector<TemplateVariable> variables;

  std::string robot_name = robot_edit_->text().toStdString();
  variables.push_back(TemplateVariable("ROBOT_NAME", robot_name));
  package_name_ = robot_name + "_description";
  variables.push_back(TemplateVariable("PACKAGE_NAME", package_name_));

  config_package_name_ = robot_name + "_nav_config";
  variables.push_back(TemplateVariable("CONFIG_PACKAGE_NAME", config_package_name_));

  variables.push_back(TemplateVariable("AUTHOR_NAME", name_edit_->text().toStdString()));
  variables.push_back(TemplateVariable("AUTHOR_EMAIL", email_edit_->text().toStdString()));

  double width = width_edit_->value();
  variables.push_back(TV("WIDTH", width));
  variables.push_back(TV("HEIGHT", height_edit_->value()));
  double length = length_edit_->value();
  variables.push_back(TV("LENGTH", length));
  variables.push_back(TV("WHEEL_RADIUS", wheel_height_edit_->value()));

  double sensor_offset_mult = sensor_offset_edit_->double_value();
  std::string shape_macro = "";
  std::string sensor_offset;
  if (shape_ == 0)
  {
    shape_macro = "circle_robot ";
    sensor_offset = std::to_string(width * sensor_offset_mult / 2);
  }
  else if (shape_ == 1)
  {
    shape_macro = "box_robot length=\"" + std::to_string(width) + "\"";
    sensor_offset = std::to_string(width * sensor_offset_mult / 2);
  }
  else
  {
    shape_macro = "box_robot length=\"" + std::to_string(length) + "\"";
    sensor_offset = std::to_string(length * sensor_offset_mult / 2);
  }
  variables.push_back(TemplateVariable("SHAPE_MACRO", shape_macro));
  variables.push_back(TemplateVariable("SENSOR_OFFSET", sensor_offset));
  variables.push_back(TV("SENSOR_OFFSET_MULT", sensor_offset_mult));
  variables.push_back(TemplateVariable("SHAPE", shape_edit_->itemText(shape_).toStdString()));

  // variables.push_back(TV("BASE_NOISE", noise_edit_->double_value()));
  // variables.push_back(TV("ODOM_MULT", odom_noise_edit_->double_value()));
  variables.push_back(TV("MAX_VEL_X", drive_vel_edit_->value()));
  variables.push_back(TV("MAX_VEL_THETA", turn_vel_edit_->value()));

  variables.push_back(TemplateVariable("BODY_COLOR", body_color_edit_->currentText().toStdString()));
  variables.push_back(TemplateVariable("WHEEL_COLOR", wheel_color_edit_->currentText().toStdString()));

  variables.push_back(TV("SENSOR_RANGE", sensor_range_edit_->value()));
  variables.push_back(TV("SENSOR_SAMPLES", samples_edit_->value()));
  variables.push_back(TV("SENSOR_NOISE", laser_noise_edit_->double_value()));
  variables.push_back(TemplateVariable("SENSOR_TYPE", sensor_edit_->currentText().toStdString()));

  variables.push_back(TemplateVariable("PARENT_PATH", path_edit_->text().toStdString()));
  return variables;
}

void RobotGenWidget::generateURDF()
{
  std::vector<TemplateVariable> variables = getVariables();

  std::filesystem::path temp_path = "/tmp/robot.xacro";
  renderTemplate(this_package_ / "templates" / "robot.urdf.xacro", variables, temp_path);
  std::string cmd = "ros2 run xacro xacro " + temp_path.string();

  std::string buffer = "";
  FILE* pipe = popen(cmd.c_str(), "r");
  char pipe_buffer[128];
  while (!feof(pipe))
  {
    if (fgets(pipe_buffer, 128, pipe) != nullptr)
      buffer += pipe_buffer;
  }
  pclose(pipe);

  auto request = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
  request->parameters.resize(1);
  request->parameters[0].name = "robot_description";
  request->parameters[0].value.type = 4;  // string
  request->parameters[0].value.string_value = buffer;

  auto result = set_params_srv_->async_send_request(request);
}

void RobotGenWidget::generate()
{
  std::filesystem::path parent_path = path_edit_->text().toStdString();
  std::vector<TemplateVariable> variables = getVariables();

  std::filesystem::path package = parent_path / package_name_;
  std::filesystem::path templates = this_package_ / "templates";

  renderTemplate(templates / "package.xml.template", variables, package / "package.xml");
  renderTemplate(templates / "CMakeLists.txt", variables, package / "CMakeLists.txt");
  renderTemplate(templates / "description.launch.py.template", variables, package / "launch" / "description.launch.py");
  renderTemplate(templates / "display.launch.py.template", variables, package / "launch" / "display.launch.py");
  renderTemplate(templates / "gazebo.launch.py.template", variables, package / "launch" / "gazebo.launch.py");
  renderTemplate(templates / "generator.launch.py.template", variables, package / "launch" / "generator.launch.py");
  renderTemplate(templates / "variables.yaml", variables, package / "config" / "variables.yaml");
  renderTemplate(templates / "kinematics.yaml", variables, package / "config" / "kinematics.yaml");
  renderTemplate(templates / "robot.urdf.xacro", variables, package / "urdf" / "robot.urdf.xacro");

  std::filesystem::path package2 = parent_path / config_package_name_;

  renderTemplate(templates / "package2.xml.template", variables, package2 / "package.xml.template");
  renderTemplate(templates / "CMakeLists2.txt", variables, package2 / "CMakeLists.txt");
  renderTemplate(templates / "amcl.rviz", variables, package2 / "config" / "amcl.rviz", false);
  renderTemplate(templates / "amcl.yaml", variables, package2 / "config" / "amcl.yaml", false);
  renderTemplate(templates / "bt_nav.yaml", variables, package2 / "config" / "bt_nav.yaml", false);
  renderTemplate(templates / "controller.yaml", variables, package2 / "config" / "controller.yaml", false);
  renderTemplate(templates / "global.rviz", variables, package2 / "config" / "global.rviz", false);
  renderTemplate(templates / "local.rviz", variables, package2 / "config" / "local.rviz", false);
  renderTemplate(templates / "mapping.rviz", variables, package2 / "config" / "mapping.rviz", false);
  renderTemplate(templates / "recoveries.yaml", variables, package2 / "config" / "recoveries.yaml", false);
  renderTemplate(templates / "planner.yaml", variables, package2 / "config" / "planner.yaml", false);
  renderTemplate(templates / "global_plan.launch.py", variables, package2 / "launch" / "global_plan.launch.py");
  renderTemplate(templates / "localization.launch.py", variables, package2 / "launch" / "localization.launch.py");
  renderTemplate(templates / "mapping.launch.py", variables, package2 / "launch" / "mapping.launch.py");
  renderTemplate(templates / "local_plan.launch.py", variables, package2 / "launch" / "local_plan.launch.py");
  renderTemplate(templates / "trees.launch.py", variables, package2 / "launch" / "trees.launch.py");

  QMessageBox::information(this, package_name_.c_str(), QString("Files generated successfully!"));
}
}  // namespace nav_u_robot_generator
