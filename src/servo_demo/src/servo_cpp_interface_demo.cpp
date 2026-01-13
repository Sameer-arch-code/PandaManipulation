#include <rclcpp/rclcpp.hpp>
#include <control_msgs/msg/joint_jog.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <moveit_servo/servo.h>
#include <moveit_servo/servo_parameters.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <tf2_ros/buffer.h>
#include <chrono>
#include <memory>
#include <geometry_msgs/msg/vector3.hpp>

using namespace std::chrono_literals;

static const rclcpp::Logger LOGGER = rclcpp::get_logger("servo_cpp_interface_demo");

// Setup
// First we declare pointers to the node and publisher that will publish commands to Servo
rclcpp::Node::SharedPtr node_;
rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr joint_cmd_pub_;
rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_cmd_pub_;
size_t count_ = 0;
rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr input_sub_;
// Sending Commands
// Here is the timer callback for publishing commands. The C++ interface sends commands through
// internal ROS topics, just like if Servo was launched using ServoNode.

// Global variables to store the incoming floats
double target_x = 0.0, target_y = 0.0, target_z = 0.0;

void inputCallback(const geometry_msgs::msg::Vector3::SharedPtr msg)
{
  target_x = msg->x;
  target_y = msg->y;
  target_z = msg->z;
  RCLCPP_INFO(LOGGER, "Received: x:%.2f, y:%.2f, z:%.2f", target_x, target_y, target_z);
}


void publishCommands()
{
  // First we will publish 100 joint jogging commands. The joint_names field allows you to specify
  // individual joints to move, at the velocity in the corresponding velocities field. It is important
  // that the message contains a recent timestamp, or Servo will think the command is stale and will
  // not move the robot.
  if (count_ < 100)
  {
    auto msg = std::make_unique<control_msgs::msg::JointJog>();
    msg->header.stamp = node_->now();
    msg->joint_names.push_back("panda_joint1");
    msg->velocities.push_back(0.3);
    joint_cmd_pub_->publish(std::move(msg));
    ++count_;
  }
  // After a while, we switch to publishing twist commands. The provided frame is the frame in which
  // the twist is defined, not the robot frame that will follow the command. Again, we need a recent
  // timestamp in the message
  else
  {
    auto msg = std::make_unique<geometry_msgs::msg::TwistStamped>();
    msg->header.stamp = node_->now();
    msg->header.frame_id = "panda_link0";
    msg->twist.linear.x = target_x;
    msg->twist.linear.y = target_y;
    msg->twist.linear.z = target_z;
    twist_cmd_pub_->publish(std::move(msg));
  }
}

// Next we will set up the node and planning_scene_monitor
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  // This is false for now until we fix the QoS settings in moveit to enable intra process comms
  node_options.use_intra_process_comms(false);
  node_ = std::make_shared<rclcpp::Node>("servo_demo_node", node_options);

  // Pause for RViz to come up. This is necessary in an integrated demo with a single launch file
  rclcpp::sleep_for(std::chrono::seconds(4));

  // Create the planning_scene_monitor. We need to pass this to Servo's constructor
  auto tf_buffer = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  auto planning_scene_monitor = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(
      node_, "robot_description", tf_buffer, "planning_scene_monitor");

  // Here we make sure the planning_scene_monitor is updating in real time from the joint states topic
  if (planning_scene_monitor->getPlanningScene())
  {
    planning_scene_monitor->startStateMonitor("/joint_states");
    planning_scene_monitor->setPlanningScenePublishingFrequency(25);
    planning_scene_monitor->startPublishingPlanningScene(
        planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE,
        "/moveit_servo/publish_planning_scene");
    planning_scene_monitor->startSceneMonitor();
    planning_scene_monitor->providePlanningSceneService();
  }
  else
  {
    RCLCPP_ERROR(LOGGER, "Planning scene not configured");
    return EXIT_FAILURE;
  }

  // These are the publishers that will send commands to MoveIt Servo. Two command types are supported:
  // JointJog messages which will directly jog the robot in the joint space, and TwistStamped messages
  // which will move the specified link with the commanded Cartesian velocity. In this demo, we jog
  // the end effector link.
  joint_cmd_pub_ = node_->create_publisher<control_msgs::msg::JointJog>(
      "servo_demo_node/delta_joint_cmds", 10);
  twist_cmd_pub_ = node_->create_publisher<geometry_msgs::msg::TwistStamped>(
      "servo_demo_node/delta_twist_cmds", 10);
  input_sub_ = node_->create_subscription<geometry_msgs::msg::Vector3>(
    "cmd_coordinates", 10, inputCallback);

  // Initializing Servo
  // Servo requires a number of parameters to dictate its behavior. These can be read automatically
  // by using the makeServoParameters helper function
  auto servo_parameters = moveit_servo::ServoParameters::makeServoParameters(node_);
  if (!servo_parameters)
  {
    RCLCPP_FATAL(LOGGER, "Failed to load the servo parameters");
    return EXIT_FAILURE;
  }

  // Initialize the Servo C++ interface by passing a pointer to the node, the parameters, and the PSM
  auto servo = std::make_unique<moveit_servo::Servo>(node_, servo_parameters, planning_scene_monitor);

  // You can start Servo directly using the C++ interface. If launched using the alternative ServoNode,
  // a ROS service is used to start Servo. Before it is started, MoveIt Servo will not accept any
  // commands or move the robot
  servo->start();

  // For this demo, we will use a simple ROS timer to send joint and twist commands to the robot
  rclcpp::TimerBase::SharedPtr timer = node_->create_wall_timer(50ms, publishCommands);

  // We use a multithreaded executor here because Servo has concurrent processes for moving the robot
  // and avoiding collisions
  auto executor = std::make_unique<rclcpp::executors::MultiThreadedExecutor>();
  executor->add_node(node_);
  executor->spin();

  rclcpp::shutdown();
  return EXIT_SUCCESS;
}