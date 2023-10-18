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

#include <nav_u_robot_generator/templates.hpp>
#include <boost/algorithm/string.hpp>  // for string find and replace in templates
#include <ament_index_cpp/get_package_share_directory.hpp>

namespace nav_u_robot_generator
{
std::filesystem::path getSharePath(const std::string& package_name)
{
  return std::filesystem::path(ament_index_cpp::get_package_share_directory(package_name));
}

std::string renderTemplate(const std::filesystem::path template_path, const std::vector<TemplateVariable>& variables)
{
  // Error check file
  if (!std::filesystem::is_regular_file(template_path))
  {
    throw std::runtime_error(std::string("Unable to find template file ") + template_path.string());
  }

  // Load file
  std::ifstream template_stream(template_path);
  if (!template_stream.good())  // File not found
  {
    throw std::runtime_error(std::string("Unable to load file ") + template_path.string());
  }

  // Load the file to a string using an efficient memory allocation technique
  std::string template_string;
  template_stream.seekg(0, std::ios::end);
  template_string.reserve(template_stream.tellg());
  template_stream.seekg(0, std::ios::beg);
  template_string.assign((std::istreambuf_iterator<char>(template_stream)), std::istreambuf_iterator<char>());
  template_stream.close();

  // Replace keywords in string ------------------------------------------------------------
  for (const auto& variable : variables)
  {
    std::string key_with_brackets = "[" + variable.key + "]";
    boost::replace_all(template_string, key_with_brackets, variable.value);
  }
  return template_string;
}

void renderTemplate(const std::filesystem::path template_path, const std::vector<TemplateVariable>& variables,
                    const std::filesystem::path output_path, bool overwrite)
{
  if (std::filesystem::is_regular_file(output_path) && !overwrite)
  {
    return;
  }

  std::string s = renderTemplate(template_path, variables);

  std::filesystem::path parent = output_path.parent_path();
  if (!std::filesystem::is_directory(parent))
  {
    std::filesystem::create_directories(parent);
  }

  std::ofstream output_stream(output_path, std::ios_base::trunc);
  if (!output_stream.good())
  {
    throw std::runtime_error(std::string("Unable to open file for writing ") + output_path.string());
  }

  output_stream << s.c_str();
  output_stream.close();
}
}  // namespace nav_u_robot_generator
