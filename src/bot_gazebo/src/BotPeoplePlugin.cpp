/***********************************************************************/
/**                                                                    */
/** BotPeoplePlugin.cpp                                                    */
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

// #include <functional>
#include <stdio.h>
#include <string>

// #include <ignition/math.hh>
// #include <ignition/math/gzmath.hh>
#include <bot_gazebo/BotPeoplePlugin.h>
#include <bot_gazebo/actor_motion.hpp>
#include <bot_gazebo/gait_orientation.hpp>
#include <gazebo/common/Mesh.hh>
#include <gazebo/common/Skeleton.hh>
#include <gazebo/common/SkeletonAnimation.hh>
#include <unordered_map>

// namespace gazebo_ros {
namespace bot
{

// GZ_REGISTER_MODEL_PLUGIN(HumanNavPlugin)

using namespace std::chrono_literals;

class BotPeoplePluginPrivate
{
public:
  /// Callback when world is updated.
  /// \param[in] _info Updated simulation info.
  void OnUpdate(const gazebo::common::UpdateInfo& _info);

  void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

  /// \brief Helper function to collect the pedestrians data from Gazebo
  bool GetPedestrians();
  void SetAnimation(const gazebo::physics::ActorPtr &actor, int id,
                    bool walking, double distance);
  double WalkingTimePerMeter(const gazebo::physics::ActorPtr &actor);
  ignition::math::Quaterniond AnimatedOrientation(
    const gazebo::physics::ActorPtr &actor, double navigation_heading);
  /// \brief Helper function to collect the robot data from Gazebo
  bool GetRobot();
  /// \brief Helper function to collect obtacles from Gazebo
  void HandleObstacles();
  /// \brief Helper function to collect obtacles from Gazebo
  void HandleObstacles2();
  /// \brief Helper function to update the Gazebo actors
  void UpdateGazeboPedestrians(const gazebo::common::UpdateInfo& _info, const bot_interfaces::msg::Agents& _agents);
  /// @brief  Helper function to get the closest point on a bounding box
  /// @param _point 
  /// @param _box 
  /// @return closest point on the bounding box
  ignition::math::Vector3d GetClosestPointOnBoundingBox(const ignition::math::Vector3d& _point,
                                                        const ignition::math::AxisAlignedBox& _box);
  /// \brief load the stored animations for the agents behaviors
  // void LoadAnimations();
  /// \brief Update the animation of the agent according to the behavior
  // void UpdateAnimation(const human_nav_msgs::msg::Agent &_agent);
  /// \brief Helper function to initialize the agents at the initial step
  void InitializeAgents();
  /// \brief Helper function to initialize the robot the initial step
  bool InitializeRobot();
  inline double normalizeAngle(double a)
  {
    double value = a;
    while (value <= -M_PI)
      value += 2 * M_PI;
    while (value > M_PI)
      value -= 2 * M_PI;
    return value;
  }

  /// \brief ros node required to call the service
  // std::shared_ptr<rclcpp::Node> rosnode;
  gazebo_ros::Node::SharedPtr rosnode;

  /// \brief ros service to update the actor states
  rclcpp::Client<bot_interfaces::srv::ComputeAgents>::SharedPtr rosSrvClient;

  /// \brief ros service to get the initial agents setup
  rclcpp::Client<bot_interfaces::srv::GetAgents>::SharedPtr rosSrvGetAgentsClient;

  /// \brief ros service to reset the agents in the bot_manager
  rclcpp::Client<bot_interfaces::srv::ResetAgents>::SharedPtr rosSrvResetClient;

  /// \brief ros service to reset the agents in the bot_manager
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub;

  /// \brief the robot as a agent msg
  bot_interfaces::msg::Agent robotAgent;
  bot_interfaces::msg::Agent init_robotAgent;

  /// \brief vector of pedestrians detected.
  std::vector<bot_interfaces::msg::Agent> pedestrians;
  std::unordered_map<int, double> walkingTimePerMeter;
  // Navigation heading is separate from the pelvis rotation in the gait clip.
  std::unordered_map<int, double> navigationHeading;
  // init state of agents
  bot_interfaces::msg::Agents init_pedestrians;

  /// \brief Pointer to the parent actor.
  gazebo::physics::ModelPtr robotModel;

  /// \brief Pointer to the world, for convenience.
  gazebo::physics::WorldPtr world;

  /// \brief Pointer to the sdf element.
  sdf::ElementPtr sdf;

  /// \brief connection to world iteration
  gazebo::event::ConnectionPtr connection;

  /// \brief Time of the last update.
  gazebo::common::Time lastUpdate;

  rclcpp::Time rostime;
  double dt;
  double update_rate_secs;
  bool reset;

  std::string robotName;
  std::string globalFrame;

  bool waitForGoal;
  bool goalReceived;
  int counter;
  bool useCollision;
  std::string goalTopic;

  /// \brief List of models to ignore. Used for vector field
  std::vector<std::string> ignoreModels;
};

/////////////////////////////////////////////////
BotPeoplePlugin::BotPeoplePlugin() : hnav_(std::make_unique<BotPeoplePluginPrivate>())
{
}

BotPeoplePlugin::~BotPeoplePlugin()
{
}

/////////////////////////////////////////////////
void BotPeoplePlugin::Load(gazebo::physics::WorldPtr _world, sdf::ElementPtr _sdf)
{
  hnav_->world = _world;

  hnav_->rosnode = gazebo_ros::Node::Get(_sdf);
  RCLCPP_INFO(hnav_->rosnode->get_logger(), "Loading Bot plugin...");

  // Get QoS profiles
  // const gazebo_ros::QoS &qos = hnav_->rosnode->get_qos();
  hnav_->sdf = _sdf;

  if (_sdf->HasElement("robot_name"))
    hnav_->robotName = _sdf->Get<std::string>("robot_name");
  else
    hnav_->robotName = "robot";

  if (_sdf->HasElement("global_frame_to_publish"))
    hnav_->globalFrame = _sdf->Get<std::string>("global_frame_to_publish");
  else
    hnav_->globalFrame = "map";

  if (_sdf->HasElement("use_navgoal_to_start"))
    hnav_->waitForGoal = _sdf->Get<bool>("use_navgoal_to_start");
  else
  {
    hnav_->waitForGoal = false;
    RCLCPP_INFO(hnav_->rosnode->get_logger(), "PARAMETER USE_NAVGOAL_TO_START IS NOT IN THE WORLD FILE!!");
  }
  if (_sdf->HasElement("use_collision"))
    hnav_->useCollision = _sdf->Get<bool>("use_collision");
  else
  {
    hnav_->useCollision = false;
    RCLCPP_INFO(hnav_->rosnode->get_logger(), "PARAMETER USE_COLLISION IS NOT IN THE WORLD FILE!!");
  }

  if (hnav_->waitForGoal)
  {
    hnav_->goalReceived = false;
    if (_sdf->HasElement("navgoal_topic"))
      hnav_->goalTopic = _sdf->Get<std::string>("navgoal_topic");
    else
      hnav_->goalTopic = "bot_goal_pose";
  }
  else
  {
    hnav_->goalReceived = true;
  }
  hnav_->counter = 0;

  // Read models to be ignored
  if (_sdf->HasElement("ignore_models"))
  {
    sdf::ElementPtr modelElem = _sdf->GetElement("ignore_models")->GetElement("model");
    while (modelElem)
    {
      hnav_->ignoreModels.push_back(modelElem->Get<std::string>());
      RCLCPP_INFO(hnav_->rosnode->get_logger(), "Ignoring model: %s", (modelElem->Get<std::string>()).c_str());
      modelElem = modelElem->GetNextElement("model");
    }
  }
  hnav_->reset = false;
  hnav_->rostime = hnav_->rosnode->get_clock()->now();
  hnav_->dt = 0;
  // Update rate
  auto update_rate = _sdf->Get<double>("update_rate", 100.0).first;
  if (update_rate > 0.0)
  {
    hnav_->update_rate_secs = 1.0 / update_rate;
  }
  else
  {
    hnav_->update_rate_secs = 0.0;
  }
  RCLCPP_INFO(hnav_->rosnode->get_logger(), "update_rate: %.2f, secs:%.4f", update_rate, hnav_->update_rate_secs);

  hnav_->rosSrvClient = hnav_->rosnode->create_client<bot_interfaces::srv::ComputeAgents>("compute_agents");

  hnav_->rosSrvGetAgentsClient = hnav_->rosnode->create_client<bot_interfaces::srv::GetAgents>("get_agents");

  hnav_->rosSrvResetClient = hnav_->rosnode->create_client<bot_interfaces::srv::ResetAgents>("reset_agents");

  if (hnav_->waitForGoal)
  {
    hnav_->goal_sub = hnav_->rosnode->create_subscription<geometry_msgs::msg::PoseStamped>(
        hnav_->goalTopic, 1, std::bind(&BotPeoplePluginPrivate::goalCallback, hnav_.get(), std::placeholders::_1));
  }

  hnav_->connection = gazebo::event::Events::ConnectWorldUpdateBegin(
      std::bind(&BotPeoplePluginPrivate::OnUpdate, hnav_.get(), std::placeholders::_1));

  // Reset();
  hnav_->InitializeAgents();
  hnav_->HandleObstacles2();

  hnav_->lastUpdate = _world->SimTime();

  for (const auto &agent : hnav_->pedestrians)
  {
    auto actor = boost::dynamic_pointer_cast<gazebo::physics::Actor>(
      hnav_->world->ModelByName(agent.name));
    hnav_->walkingTimePerMeter[agent.id] = hnav_->WalkingTimePerMeter(actor);
    hnav_->navigationHeading[agent.id] = agent.yaw;
    hnav_->SetAnimation(actor, agent.id, false, 0.0);
  }

  // Reset();
}

// /////////////////////////////////////////////////
// void PedestriansPlugin::Reset() {
//   // this->velocity = 0.8;
//   lastUpdate = 0;

//   // auto skelAnims = this->actor->SkeletonAnimations();
//   // if (skelAnims.find(this->animationName) == skelAnims.end()) {
//   //   gzerr << "Skeleton animation " << this->animationName << " not
//   found.\n";
//   // } else {
//   //   // Create custom trajectory
//   //   this->trajectoryInfo.reset(new physics::TrajectoryInfo());
//   //   this->trajectoryInfo->type = this->animationName;
//   //   this->trajectoryInfo->duration = 1.0;

//   //   this->actor->SetCustomTrajectory(this->trajectoryInfo);
//   // }
// }

void BotPeoplePlugin::Reset()
{
  hnav_->reset = true;
  hnav_->lastUpdate = hnav_->world->SimTime();
  hnav_->rostime = hnav_->rosnode->get_clock()->now();
  hnav_->InitializeRobot();
  hnav_->pedestrians = hnav_->init_pedestrians.agents;
  for (auto &agent : hnav_->pedestrians)
  {
    auto actor = boost::dynamic_pointer_cast<gazebo::physics::Actor>(
      hnav_->world->ModelByName(agent.name));
    if (!actor) continue;
    actor->SetWorldPose(ignition::math::Pose3d(
      agent.position.position.x, agent.position.position.y, actorHeight(agent.skin),
      kActorRoll, 0.0, agent.yaw + kActorYawOffset), false, false);
    agent.velocity = geometry_msgs::msg::Twist();
    agent.linear_vel = 0.0;
    agent.angular_vel = 0.0;
    actor->ResetCustomTrajectory();
    hnav_->navigationHeading[agent.id] = agent.yaw;
    hnav_->SetAnimation(actor, agent.id, false, 0.0);
    auto pose = actor->WorldPose();
    pose.Rot() = hnav_->AnimatedOrientation(actor, agent.yaw);
    actor->SetWorldPose(pose, false, false);
  }
}

bool BotPeoplePluginPrivate::InitializeRobot()
{
  RCLCPP_INFO(rosnode->get_logger(), "Initializing robot...");
  robotModel = world->ModelByName(robotName);
  if (!robotModel)
  {
    RCLCPP_ERROR(rosnode->get_logger(), "Robot model %s not found (yet)!!!!", robotName.c_str());
    return false;
  }
  else
  {
    RCLCPP_INFO(rosnode->get_logger(), "Robot %s detected!!! initializing params...", robotName.c_str());
    // Initialize robot agent
    ignition::math::Vector3d pos = robotModel->WorldPose().Pos();
    ignition::math::Vector3d rpy = robotModel->WorldPose().Rot().Euler();
    robotAgent.id = robotModel->GetId();
    robotAgent.type = bot_interfaces::msg::Agent::ROBOT;
    robotAgent.behavior.type = bot_interfaces::msg::AgentBehavior::BEH_REGULAR;
    robotAgent.behavior.state = bot_interfaces::msg::AgentBehavior::BEH_NO_ACTIVE;
    robotAgent.name = robotName;
    robotAgent.group_id = -1;
    robotAgent.radius = 0.35;
    robotAgent.position.position.x = pos.X();
    robotAgent.position.position.y = pos.Y();
    tf2::Quaternion myQuaternion;
    myQuaternion.setRPY(0, 0, rpy.Z());
    robotAgent.position.orientation = tf2::toMsg(myQuaternion);
    robotAgent.yaw = rpy.Z();
    ignition::math::Vector3d linvel = robotModel->WorldLinearVel();
    robotAgent.velocity.linear.x = linvel.X();
    robotAgent.velocity.linear.y = linvel.Y();
    robotAgent.linear_vel = linvel.Length();
    ignition::math::Vector3d angvel = robotModel->WorldAngularVel();
    robotAgent.velocity.angular.z = angvel.Z();
    robotAgent.angular_vel = angvel.Z();
    init_robotAgent = robotAgent;
    return true;
  }
}

void BotPeoplePluginPrivate::InitializeAgents()
{
  RCLCPP_INFO(rosnode->get_logger(), "Initializing agents...");

  InitializeRobot();

  // Fill the service request
  auto request = std::make_shared<bot_interfaces::srv::GetAgents::Request>();
  request->empty = 0;

  // Wait for the service to be available
  while (!rosSrvGetAgentsClient->wait_for_service(5s))
  {
    if (!rclcpp::ok())
    {
      RCLCPP_ERROR(rosnode->get_logger(), "Interrupted while waiting for the service. Exiting.");
      return;
    }
    RCLCPP_WARN(rosnode->get_logger(), "service /get_agents not available, waiting again...");
  }
  // Call the service
  auto result = rosSrvGetAgentsClient->async_send_request(request);
  std::chrono::duration<int, std::milli> ms(200);
  if (result.wait_for(ms) == std::future_status::ready)
  {
    // Initialize the actors
    auto res = *result.get();
    init_pedestrians = res.agents;

    // if (result.success) {
    // init_pedestrians = result.get()->agents;
    RCLCPP_INFO(rosnode->get_logger(), "Received %i agents from service /get_agents", (int)init_pedestrians.agents.size());
    pedestrians.clear();

    for (auto agent : init_pedestrians.agents)
    {
      bot_interfaces::msg::Agent ag = agent;

      gazebo::physics::ModelPtr model = world->ModelByName(agent.name);
      if (!model)
      {
        RCLCPP_ERROR(rosnode->get_logger(), "Pedestrian model '%s' not found!!!!", agent.name.c_str());
        return;
      }

      // ---- update Gazebo models ---
      gazebo::physics::ActorPtr actor = boost::dynamic_pointer_cast<gazebo::physics::Actor>(model);

      ignition::math::Pose3d actorPose = actor->WorldPose();
      double yaw = normalizeAngle(agent.yaw + kActorYawOffset);
      auto entity_lin_vel = gazebo_ros::Convert<ignition::math::Vector3d>(agent.velocity.linear);
      auto entity_ang_vel = gazebo_ros::Convert<ignition::math::Vector3d>(agent.velocity.angular);
      actorPose.Pos().X(agent.position.position.x);
      actorPose.Pos().Y(agent.position.position.y);
      actorPose.Pos().Z(actorHeight(agent.skin));
      actorPose.Rot() = ignition::math::Quaterniond(kActorRoll, 0, yaw);
      model->SetWorldPose(actorPose);
      if (useCollision)
      {
        gazebo::physics::ModelPtr body_model = world->ModelByName(agent.name + "_body");
        actorPose.Rot() = ignition::math::Quaterniond(0, 0, yaw);
        body_model->SetWorldPose(actorPose);
      }
      //-----------------------------------

      ignition::math::Vector3d pos = model->WorldPose().Pos();
      // The front (x axis) of the actors in Gazebo is
      // looking to the person left, we rotate it
      yaw = normalizeAngle(model->WorldPose().Rot().Yaw() - kActorYawOffset);
      ag.position.position.x = pos.X();
      ag.position.position.y = pos.Y();
      tf2::Quaternion myQuaternion;
      myQuaternion.setRPY(0, 0, yaw);
      ag.position.orientation = tf2::toMsg(myQuaternion);
      ag.yaw = yaw;
      ignition::math::Vector3d linvel = model->WorldLinearVel();
      ag.desired_velocity = agent.desired_velocity;
      ag.velocity.linear.x = linvel.X();
      ag.velocity.linear.y = linvel.Y();
      ag.linear_vel = linvel.Length();
      ignition::math::Vector3d angvel = model->WorldAngularVel();
      ag.velocity.angular.z = angvel.Z();
      ag.angular_vel = angvel.Z();

      pedestrians.push_back(ag);
      RCLCPP_INFO(rosnode->get_logger(),
                  "Adding agent: %s, type: %i, id:%i, beh:%i, x:%.2f, "
                  "y:%.2f, th:%.2f, dvel:%.2f",
                  ag.name.c_str(), ag.type, ag.id, (int)ag.behavior.type, pos.X(), pos.Y(), yaw,
                  pedestrians.back().desired_velocity);
    }
    //} else {
    //  RCLCPP_ERROR(rosnode->get_logger(), "Call service /get_agents not success!!");
    //}
  }
  else
  {
    RCLCPP_ERROR(rosnode->get_logger(), "Failed to call service get_agents");
  }
}

void BotPeoplePluginPrivate::goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  RCLCPP_INFO(rosnode->get_logger(), "\n\nPlugin! GOAL RECEIVED!!!!\n\n");
  goalReceived = true;
}

/////////////////////////////////////////////////
// void BotPeoplePluginPrivate::HandleObstacles()
// {
//   for (size_t i = 0; i < pedestrians.size(); ++i)
//   {
//     gazebo::physics::ModelPtr agent = world->ModelByName(pedestrians[i].name);

//     double minDist = 10000.0;
//     ignition::math::Vector3d closest_obstacle;
//     // ignition::math::Vector3d closest_obs2;
//     pedestrians[i].closest_obs.clear();

//     for (size_t j = 0; j < world->ModelCount(); ++j)
//     {
//       gazebo::physics::ModelPtr modelObstacle = world->ModelByIndex(j);

//       // Avoid to compute the other actors as obstacles
//       for (size_t k = 0; k < pedestrians.size(); ++k)
//       {
//         if ((int)modelObstacle->GetId() == pedestrians[k].id)
//           break;
//       }

//       // Avoid the agent itself and the indicated models
//       if (agent->GetId() != modelObstacle->GetId() && std::find(this->ignoreModels.begin(), this->ignoreModels.end(),
//                                                                 modelObstacle->GetName()) == this->ignoreModels.end())
//       {
//         ignition::math::Vector3d actorPos = agent->WorldPose().Pos();
//         ignition::math::Vector3d obsPos = modelObstacle->WorldPose().Pos();

//         // This if is used to avoid an invisible obstacle that Gazebo is setting at (0,0). We don't know why it's
//         // happening.
//         if (obsPos.X() != 0.0 && obsPos.Y() != 0.0)
//         {
//           // RCLCPP_INFO(rosnode->get_logger(), "\n\nX --> %.2f and Y --> %.2f\n\n", obsPos.X(), obsPos.Y());
//           ignition::math::Line3d act_obs_line(actorPos, obsPos);
//           std::tuple<bool, double, ignition::math::Vector3d> obs_intersect =
//               modelObstacle->BoundingBox().Intersect(act_obs_line);

//           ignition::math::Vector3d intersecPos;
//           double dist = -1;

//           if (std::get<0>(obs_intersect) == true)
//           {
//             intersecPos = std::get<2>(obs_intersect);
//             dist = std::get<1>(obs_intersect);
//           }

//           /*ignition::math::Vector3d goalPos(pedestrians[i].goals[0].position.x,
//                                           pedestrians[i].goals[0].position.y,
//                                           actorPos.Z());
//           ignition::math::Line3d act_goal_line(actorPos, goalPos);
//           std::tuple<bool, double, ignition::math::Vector3d> goal_intersect =
//               modelObstacle->BoundingBox().Intersect(act_goal_line);
//           if (std::get<0>(goal_intersect) == true) {
//             if (dist > 0.0 && std::get<1>(goal_intersect) < dist) {
//               intersecPos = std::get<2>(goal_intersect);
//               dist = std::get<1>(goal_intersect);
//             }
//           }*/

//           if (dist > 0)
//           {
//             ignition::math::Vector3d offset = intersecPos - actorPos;
//             double modelDist = offset.Length();  //-approximated_radius;
//             // double dist2 = actorPos.Distance(std::get<2>(intersect));

//             if (modelDist < minDist)
//             {
//               minDist = modelDist;
//               // closest_obs = offset;
//               closest_obstacle = intersecPos;
//             }
//           }
//         }
//         // std::tuple<bool, double, ignition::math::Vector3d> intersect =
//         //     modelObstacle->BoundingBox().Intersect(obsPos, actorPos,
//         //     0.05, 8.0);
//       }
//     }
//     if (minDist <= 10.0)
//     {
//       geometry_msgs::msg::Point p;
//       p.x = closest_obstacle.X();
//       p.y = closest_obstacle.Y();
//       p.z = 0.0;
//       pedestrians[i].closest_obs.push_back(p);
//     }
//   }
// }



ignition::math::Vector3d ClosestPointUsingCollisions(
  const ignition::math::Vector3d &point,
  const gazebo::physics::WorldPtr &world,
  const std::vector<std::string> &ignoreModels)
{
  double minDist = std::numeric_limits<double>::max();
  ignition::math::Vector3d closestPoint;

  // Iterate through all models in the world
  for (size_t i = 0; i < world->ModelCount(); ++i)
  {
      gazebo::physics::ModelPtr model = world->ModelByIndex(i);

      // Skip ignored models
      if (std::find(ignoreModels.begin(), ignoreModels.end(), model->GetName()) != ignoreModels.end())
          continue;

      // Iterate through all links in the model
      for (auto link : model->GetLinks())
      {
          // Iterate through all collisions in the link
          for (auto collision : link->GetCollisions())
          {
              // Get the bounding box of the collision
              auto bbox = collision->BoundingBox();

              // Compute the closest point on the bounding box
              auto minCorner = bbox.Min();
              auto maxCorner = bbox.Max();
              double x = std::max(minCorner.X(), std::min(point.X(), maxCorner.X()));
              double y = std::max(minCorner.Y(), std::min(point.Y(), maxCorner.Y()));
              double z = std::max(minCorner.Z(), std::min(point.Z(), maxCorner.Z()));
              ignition::math::Vector3d candidatePoint(x, y, z);

              // Compute the distance to the point
              double dist = (candidatePoint - point).Length();
              if (dist < minDist)
              {
                  minDist = dist;
                  closestPoint = candidatePoint;
              }
          }
      }
  }

  return closestPoint;
}



void BotPeoplePluginPrivate::HandleObstacles2()
{
  for (size_t i = 0; i < pedestrians.size(); ++i)
  {
    gazebo::physics::ModelPtr agent = world->ModelByName(pedestrians[i].name);

    double minDist = 5.0; //10000.0;
    pedestrians[i].closest_obs.clear();

    for (size_t j = 0; j < world->ModelCount(); ++j)
    {
      gazebo::physics::ModelPtr modelObstacle = world->ModelByIndex(j);

      // Avoid to compute the other actors as obstacles
      for (size_t k = 0; k < pedestrians.size(); ++k)
      {
        if ((int)modelObstacle->GetId() == pedestrians[k].id)
          break;
      }

      // Avoid the agent itself and the indicated models
      if (agent->GetId() != modelObstacle->GetId() && std::find(this->ignoreModels.begin(), this->ignoreModels.end(),
                                                                modelObstacle->GetName()) == this->ignoreModels.end())
      {

        // Iterate through all links in the model
        for (auto link : modelObstacle->GetLinks())
        {
          // Iterate through all collisions in the link
          for (auto collision : link->GetCollisions())
          {
            // Get the bounding box of the collision
            auto bbox = collision->BoundingBox();

            ignition::math::Vector3d actorPos = agent->WorldPose().Pos();
            auto closestObs = GetClosestPointOnBoundingBox(actorPos, modelObstacle->BoundingBox()); 
            ignition::math::Vector3d offset = closestObs - actorPos;
            double dist = offset.Length();
            if (dist > 0 && dist < minDist)
            {
              geometry_msgs::msg::Point p;
              p.x = closestObs.X();
              p.y = closestObs.Y();
              p.z = 0.0;
              pedestrians[i].closest_obs.push_back(p);
            }
          }
        }
      }
    }
  }
}







ignition::math::Vector3d BotPeoplePluginPrivate::GetClosestPointOnBoundingBox(const ignition::math::Vector3d& _point,
  const ignition::math::AxisAlignedBox& _box)
{
  // Get the minimum and maximum corners of the bounding box
  auto minCorner = _box.Min();
  auto maxCorner = _box.Max();

  // Clamp the point to the bounding box
  double x = std::max(minCorner.X(), std::min(_point.X(), maxCorner.X()));
  double y = std::max(minCorner.Y(), std::min(_point.Y(), maxCorner.Y()));
  double z = std::max(minCorner.Z(), std::min(_point.Z(), maxCorner.Z()));

  // Return the closest point
  return ignition::math::Vector3d(x, y, z);
}


void BotPeoplePluginPrivate::HandleObstacles()
{
  for (size_t i = 0; i < pedestrians.size(); ++i)
  {
    gazebo::physics::ModelPtr agent = world->ModelByName(pedestrians[i].name);

    double minDist = 5.0; //10000.0;
    pedestrians[i].closest_obs.clear();

    for (size_t j = 0; j < world->ModelCount(); ++j)
    {
      gazebo::physics::ModelPtr modelObstacle = world->ModelByIndex(j);

      // Avoid to compute the other actors as obstacles
      for (size_t k = 0; k < pedestrians.size(); ++k)
      {
        if ((int)modelObstacle->GetId() == pedestrians[k].id)
          break;
      }

      // Avoid the agent itself and the indicated models
      if (agent->GetId() != modelObstacle->GetId() && std::find(this->ignoreModels.begin(), this->ignoreModels.end(),
                                                                modelObstacle->GetName()) == this->ignoreModels.end())
      {
        ignition::math::Vector3d actorPos = agent->WorldPose().Pos();
        ignition::math::Vector3d obsPos = modelObstacle->WorldPose().Pos();

        //RCLCPP_INFO(rosnode->get_logger(), "\n model %s, X --> %.2f and Y --> %.2f\n\n", modelObstacle->GetName().c_str(), obsPos.X(), obsPos.Y());

        // This if is used to avoid an invisible obstacle that Gazebo is setting at (0,0). We don't know why it's
        // happening.
        if (obsPos.X() != 0.0 && obsPos.Y() != 0.0)
        {
          auto closestObs = GetClosestPointOnBoundingBox(actorPos, modelObstacle->BoundingBox()); 
          ignition::math::Vector3d offset = closestObs - actorPos;
          double dist = offset.Length();
          if (dist > 0 && dist < minDist)
          {
            geometry_msgs::msg::Point p;
            p.x = closestObs.X();
            p.y = closestObs.Y();
            p.z = 0.0;
            pedestrians[i].closest_obs.push_back(p);
          }
        }
      }
    }
  }
}




/**
 * \brief Get the robot position and velocity
 */
bool BotPeoplePluginPrivate::GetRobot()
{
  if (!robotModel)
  {
    if (!InitializeRobot())
    {
      //RCLCPP_ERROR(rosnode->get_logger(), "Robot model %s not found!!!!", robotName.c_str());
      return false;
    }
    return true;
  }
  // Update robot agent
  ignition::math::Vector3d pos = robotModel->WorldPose().Pos();
  // ignition::math::Vector3d rpy = robotModel->WorldPose().Rot().Euler();
  robotAgent.position.position.x = pos.X();
  robotAgent.position.position.y = pos.Y();
  tf2::Quaternion myQuaternion;
  myQuaternion.setRPY(0, 0, robotModel->WorldPose().Rot().Yaw());
  robotAgent.position.orientation = tf2::toMsg(myQuaternion);
  robotAgent.yaw = robotModel->WorldPose().Rot().Yaw();
  ignition::math::Vector3d linvel = robotModel->WorldLinearVel();
  robotAgent.velocity.linear.x = linvel.X();
  robotAgent.velocity.linear.y = linvel.Y();
  robotAgent.linear_vel = linvel.Length();
  ignition::math::Vector3d angvel = robotModel->WorldAngularVel();
  robotAgent.velocity.angular.z = angvel.Z();
  robotAgent.angular_vel = angvel.Z();
  return true;
}

/////////////////////////////////////////////////
bool BotPeoplePluginPrivate::GetPedestrians()
{
  const double elapsed = dt > 1e-9 ? dt : update_rate_secs;
  if (elapsed <= 0.0) return false;
  for (auto &agent : pedestrians)
  {
    auto model = world->ModelByName(agent.name);
    if (!model)
    {
      RCLCPP_ERROR(rosnode->get_logger(), "Pedestrian model %s not found", agent.name.c_str());
      return false;
    }
    const auto pose = model->WorldPose();
    const auto heading = navigationHeading.find(agent.id);
    const double yaw = heading != navigationHeading.end() ? heading->second :
      normalizeAngle(pose.Rot().Yaw() - kActorYawOffset);
    const double vx = reset ? 0.0 : (pose.Pos().X() - agent.position.position.x) / elapsed;
    const double vy = reset ? 0.0 : (pose.Pos().Y() - agent.position.position.y) / elapsed;
    agent.velocity.linear.x = vx;
    agent.velocity.linear.y = vy;
    agent.linear_vel = std::hypot(vx, vy);
    agent.angular_vel = reset ? 0.0 : normalizeAngle(yaw - agent.yaw) / elapsed;
    agent.velocity.angular.z = agent.angular_vel;
    agent.position.position.x = pose.Pos().X();
    agent.position.position.y = pose.Pos().Y();
    agent.yaw = yaw;
    tf2::Quaternion orientation;
    orientation.setRPY(0, 0, yaw);
    agent.position.orientation = tf2::toMsg(orientation);
  }
  reset = false;
  return true;
}

// Custom Gazebo trajectories consume animation seconds, even when interpolate_x
// is true. Derive seconds/metre from the clip's native root motion instead of
// assuming one metre of travel equals one second of animation.
double BotPeoplePluginPrivate::WalkingTimePerMeter(const gazebo::physics::ActorPtr &actor)
{
  const auto &animations = actor->SkeletonAnimations();
  const auto clip = animations.find("walking");
  if (clip == animations.end() || !actor->Mesh() || !actor->Mesh()->GetSkeleton())
    return 4.5;
  auto *animation = clip->second;
  const double duration = animation->GetLength();
  const auto root = actor->Mesh()->GetSkeleton()->GetRootNode()->GetName();
  const auto first = animation->NodePoseAt(root, 0.0, false).Translation();
  const auto last = animation->NodePoseAt(root, duration, false).Translation();
  const double distance = std::hypot(last.X() - first.X(), last.Y() - first.Y());
  const double factor = distance > 0.01 ? duration / distance : 4.5;
  RCLCPP_INFO(rosnode->get_logger(), "%s walking cadence: %.3f animation seconds/metre",
              actor->GetName().c_str(), factor);
  return factor;
}

void BotPeoplePluginPrivate::SetAnimation(const gazebo::physics::ActorPtr &actor,
                                         int id, bool walking, double distance)
{
  const std::string clip = walking ? "walking" : "idle";
  const auto animation = actor->SkeletonAnimations().find(clip);
  if (animation == actor->SkeletonAnimations().end())
  {
    RCLCPP_WARN_ONCE(rosnode->get_logger(), "Missing walking/idle clips; regenerate the world");
    return;
  }
  const auto current = actor->CustomTrajectory();
  if (!current || current->type != clip)
  {
    gazebo::physics::TrajectoryInfoPtr trajectory(new gazebo::physics::TrajectoryInfo());
    trajectory->id = id;
    trajectory->type = clip;
    trajectory->duration = animation->second->GetLength();
    trajectory->translated = false;
    actor->SetCustomTrajectory(trajectory);
    actor->SetScriptTime(0.0);
    actor->Play();
  }
  const auto factor = walkingTimePerMeter.find(id);
  const double cadence = factor == walkingTimePerMeter.end() ? 4.5 : factor->second;
  actor->SetScriptTime(walking ? actor->ScriptTime() + distance * cadence : 0.0);
}

// Gazebo's custom-trajectory path replaces the animated root rotation with the
// actor's world rotation. Preserve the clip's pelvis rotation here; otherwise
// the spine's compensating roll plays against a rigid pelvis and rocks the torso.
ignition::math::Quaterniond BotPeoplePluginPrivate::AnimatedOrientation(
  const gazebo::physics::ActorPtr &actor, double navigation_heading)
{
  const auto trajectory = actor->CustomTrajectory();
  if (trajectory && actor->Mesh() && actor->Mesh()->GetSkeleton())
  {
    const auto clip = actor->SkeletonAnimations().find(trajectory->type);
    if (clip != actor->SkeletonAnimations().end())
    {
      const auto root = actor->Mesh()->GetSkeleton()->GetRootNode()->GetName();
      const auto pelvis = clip->second->NodePoseAt(root, actor->ScriptTime()).Rotation();
      return gaitOrientation(navigation_heading, pelvis);
    }
  }
  return ignition::math::Quaterniond(kActorRoll, 0.0, navigation_heading + kActorYawOffset);
}

void BotPeoplePluginPrivate::UpdateGazeboPedestrians(const gazebo::common::UpdateInfo&,
                                                   const bot_interfaces::msg::Agents& agents)
{
  if (!goalReceived) return;
  for (const auto &agent : agents.agents)
  {
    auto actor = boost::dynamic_pointer_cast<gazebo::physics::Actor>(world->ModelByName(agent.name));
    if (!actor) continue;
    const auto previous = actor->WorldPose();
    const double dx = agent.position.position.x - previous.Pos().X();
    const double dy = agent.position.position.y - previous.Pos().Y();
    const double distance = std::hypot(dx, dy);
    const bool walking = dt > 0.0 && distance / dt > 0.02;
    // Face the displacement actually applied in this same frame. A separate
    // 1%-per-frame yaw filter lets the body slide sideways around turns.
    const double heading = walking ? movementHeading(dx, dy, agent.yaw) : agent.yaw;
    navigationHeading[agent.id] = heading;
    SetAnimation(actor, agent.id, walking, distance);
    const ignition::math::Pose3d pose(
      ignition::math::Vector3d(agent.position.position.x, agent.position.position.y, actorHeight(agent.skin)),
      AnimatedOrientation(actor, heading));
    actor->SetWorldPose(pose, false, false);
    if (useCollision)
    {
      auto body = world->ModelByName(agent.name + "_body");
      if (body) body->SetWorldPose(ignition::math::Pose3d(pose.Pos(),
        ignition::math::Quaterniond(0.0, 0.0, heading)), false, false);
    }
    for (auto &state : pedestrians)
    {
      if (state.id != agent.id) continue;
      state.goals = agent.goals;
      state.behavior = agent.behavior;
      break;
    }
  }
}

/////////////////////////////////////////////////
void BotPeoplePluginPrivate::OnUpdate(const gazebo::common::UpdateInfo& _info)
{
  // Time delta
  // double dt = (_info.simTime - this->lastUpdate).Double();

  // rclcpp::Time now = rosnode->get_clock()->now();

  double seconds_since_last_update = (_info.simTime - lastUpdate).Double();
  // // RCLCPP_INFO(rosnode->get_logger(), "seconds since last update: %.4f",
  // //             seconds_since_last_update);

  if (seconds_since_last_update < update_rate_secs)
    return;
  dt = seconds_since_last_update;
  // // update_rate_secs = 0.1;
  // if (seconds_since_last_update < update_rate_secs) {
  //   // get pedestrian states (fill pedestrians)
  //   // GetPedestrians();
  //   // bot_interfaces::msg::Agents ags;
  //   // ags.agents = pedestrians;
  //   // ags.header.frame_id = "world";
  //   // ags.header.stamp = now;
  //   // UpdateGazeboPedestrians(_info, ags);
  //   RCLCPP_INFO(rosnode->get_logger(), "skipping!!!");
  //   return;
  // }

  // update closest obstacle (fill pedestrians obstacles)
  HandleObstacles2();
  // get robot state (fill robotAgent)
  if (!GetRobot())
    return;
  // get pedestrian states (fill pedestrians)
  if (!GetPedestrians())
    return;

  // Fill the service request
  auto request = std::make_shared<bot_interfaces::srv::ComputeAgents::Request>();

  // Wait for the service to be available
  while (!rosSrvClient->wait_for_service(1s))
  {
    if (!rclcpp::ok())
    {
      RCLCPP_ERROR(rosnode->get_logger(), "Interrupted while waiting for the service. Exiting.");
      return;
    }
    RCLCPP_WARN(rosnode->get_logger(), "Service /compute_agents not available, waiting again...");
  }

  bot_interfaces::msg::Agents agents;
  agents.header.frame_id = globalFrame;
  agents.header.stamp = gazebo_ros::Convert<builtin_interfaces::msg::Time>(_info.simTime);
  // agents.header.stamp = now;
  agents.agents = pedestrians;
  request->robot = robotAgent;
  request->current_agents = agents;

  // Call the service
  auto result = rosSrvClient->async_send_request(request);
  // Wait for the result.
  std::chrono::duration<int, std::milli> ms(150);
  // RCLCPP_INFO(rosnode->get_logger(), "Waiting for service result...");
  //  This provokes a lock here so we can not use it
  //  if (rclcpp::spin_until_future_complete(rosnode, result, ms) ==
  //     rclcpp::FutureReturnCode::SUCCESS)
  //  {
  //  Update the agents data

  // std::chrono::duration<int, std::milli> ms(300);
  auto res = result.wait_for(ms);
  if (res == std::future_status::ready)
  {
    //  RCLCPP_INFO(rosnode->get_logger(), "Service result received!");

    // for (auto ped : pedestrians) {
    //   for (auto a : result.get()->updated_agents.agents)
    //     if (ped.id == a.id) {
    //       RCLCPP_INFO(rosnode->get_logger(),
    //                   "OnUpdate Actor %s:\npx:%.2f, py:%.2f, plv:%.2f, "
    //                   "pvx:%.2f, pvy:%.2f, pva:%.2f\nnx:%.2f, ny:%.2f, "
    //                   "nlv:%.2f, nvx:%.2f, nvy:%.2f, nva: %.2f ",
    //                   ped.name.c_str(), ped.position.position.x,
    //                   ped.position.position.y, ped.linear_vel,
    //                   ped.velocity.linear.x, ped.velocity.linear.y,
    //                   ped.velocity.angular.z, a.position.position.x,
    //                   a.position.position.y, a.linear_vel,
    //                   a.velocity.linear.x, a.velocity.linear.y,
    //                   a.velocity.angular.z);
    //     }
    // }

    // update the Gazebo actors
    UpdateGazeboPedestrians(_info, result.get()->updated_agents);
  }
  else if (res == std::future_status::timeout)
  {
    RCLCPP_ERROR(rosnode->get_logger(), "Service /compute_agents timeout!!");
  }
  else
  {
    RCLCPP_ERROR(rosnode->get_logger(), "Failed to call service /compute_agents");
  }
  lastUpdate = _info.simTime;
}

GZ_REGISTER_WORLD_PLUGIN(BotPeoplePlugin)
}  // namespace bot
