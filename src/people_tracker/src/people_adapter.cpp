// Ground-truth people source: simulator /human_states (social_nav_msgs/Agents) -> /people
// (social_nav_msgs/People: name = id, position x/y, velocity vx/vy). Same output as the leg
// tracker, so the navigation stack does not care which one runs.
#include <memory>
#include <string>

#include "social_nav_msgs/msg/agents.hpp"
#include "social_nav_msgs/msg/people.hpp"
#include "rclcpp/rclcpp.hpp"

class PeopleAdapter : public rclcpp::Node
{
public:
  PeopleAdapter()
  : Node("people_adapter")
  {
    frame_ = declare_parameter<std::string>("frame_id", "");   // '' keeps the source frame
    pub_ = create_publisher<social_nav_msgs::msg::People>("people", 5);
    sub_ = create_subscription<social_nav_msgs::msg::Agents>(
      "human_states", 5, [this](const social_nav_msgs::msg::Agents::SharedPtr msg) {
        social_nav_msgs::msg::People out;
        out.header = msg->header;
        if (!frame_.empty()) {
          out.header.frame_id = frame_;
        }
        out.people.reserve(msg->agents.size());
        for (const auto & agent : msg->agents) {
          social_nav_msgs::msg::Person person;
          person.name = std::to_string(agent.id);
          person.position.x = agent.position.position.x;
          person.position.y = agent.position.position.y;
          person.velocity.x = agent.velocity.linear.x;
          person.velocity.y = agent.velocity.linear.y;
          person.reliability = 1.0;
          out.people.push_back(person);
        }
        pub_->publish(out);
      });
  }

private:
  std::string frame_;
  rclcpp::Publisher<social_nav_msgs::msg::People>::SharedPtr pub_;
  rclcpp::Subscription<social_nav_msgs::msg::Agents>::SharedPtr sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PeopleAdapter>());
  rclcpp::shutdown();
  return 0;
}
