/***********************************************************************/
/**                                                                    */
/** PeoplePlugin.h                                              */
/**                                                                    */
/** Copyright (c) 2022, Service Robotics Lab (SRL).                    */
/**                     http://robotics.upo.es                         */
/**                                                                    */
/** All rights reserved.                                               */
/**                                                                    */
/** Authors:                                                           */
/** Noé Pérez-Higueras (maintainer)                                    */
/** email: noeperez@upo.es                                             */
/**                                                                    */
/** This software may be modified and distributed under the terms      */
/** of the MIT license. See the LICENSE file for details.              */
/**                                                                    */
/**                                                                    */
/***********************************************************************/

#ifndef HUMAN_GAZEBO__PEOPLE_PLUGIN_H_
#define HUMAN_GAZEBO__PEOPLE_PLUGIN_H_

// C++
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <string>
#include <vector>

// Gazebo
#include "gazebo/common/Plugin.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/util/system.hh"
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo_ros/conversions/builtin_interfaces.hpp>
#include <gazebo_ros/conversions/geometry_msgs.hpp>
#include <gazebo_ros/node.hpp>

// ROS
#include "social_nav_msgs/msg/agent.hpp"
#include "social_nav_msgs/msg/agents.hpp"
#include "social_nav_msgs/srv/compute_agents.hpp"
#include "social_nav_msgs/srv/get_agents.hpp"
#include "social_nav_msgs/srv/reset_agents.hpp"
#include "rclcpp/rclcpp.hpp"
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// namespace gazebo_ros {
namespace crowd {

class PeoplePluginPrivate;

class PeoplePlugin : public gazebo::WorldPlugin {

public:
  /// \brief Constructor
  PeoplePlugin();

  /// \brief Destructor
  virtual ~PeoplePlugin();

  /// \brief Load the robot model plugin.
  /// \param[in] _model Pointer to the parent model.
  /// \param[in] _sdf Pointer to the plugin's SDF elements.
  void Load(gazebo::physics::WorldPtr _world, sdf::ElementPtr _sdf) override;

  void Reset() override;

private:
  std::unique_ptr<PeoplePluginPrivate> hnav_;
};
} // namespace crowd
#endif
