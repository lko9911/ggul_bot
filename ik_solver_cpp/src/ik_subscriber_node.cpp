#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include <memory>  // std::enable_shared_from_this

class IKSubscriber : public rclcpp::Node, public std::enable_shared_from_this<IKSubscriber>
{
public:
    IKSubscriber() : Node("ik_subscriber_node")
    {
        // 현재 노드의 shared_ptr을 얻기
        auto node = shared_from_this();

        // RobotModelLoader 초기화 (노드 전달 필요)
        robot_model_loader_ = std::make_shared<robot_model_loader::RobotModelLoader>(node);
        robot_model_ = robot_model_loader_->getModel();
        robot_state_ = std::make_shared<moveit::core::RobotState>(robot_model_);
        planning_group_ = "manipulator2";
        joint_model_group_ = robot_model_->getJointModelGroup(planning_group_);

        // Pose 구독
        pose_sub_ = this->create_subscription<geometry_msgs::msg::Pose>(
            "/target_pose", 10,
            std::bind(&IKSubscriber::poseCallback, this, std::placeholders::_1));

        // Joint State 구독 (옵션)
        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            std::bind(&IKSubscriber::jointStateCallback, this, std::placeholders::_1));

        // IK 결과 Publish
        ik_solution_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/ik_solution", 10);

        RCLCPP_INFO(this->get_logger(), "IK Subscriber node initialized.");
    }

private:
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        current_joint_state_ = msg;  // 필요 시 활용
    }

    void poseCallback(const geometry_msgs::msg::Pose::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received pose, calculating IK...");

        bool found_ik = robot_state_->setFromIK(joint_model_group_, *msg);

        if (found_ik)
        {
            std::vector<double> joint_values;
            robot_state_->copyJointGroupPositions(joint_model_group_, joint_values);

            RCLCPP_INFO(this->get_logger(), "IK Solution:");
            for (size_t i = 0; i < joint_values.size(); ++i)
            {
                RCLCPP_INFO(this->get_logger(), "  Joint %zu: %.3f", i, joint_values[i]);
            }

            // Publish 결과
            std_msgs::msg::Float64MultiArray solution_msg;
            solution_msg.data = joint_values;
            ik_solution_pub_->publish(solution_msg);
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "No IK solution found");
        }
    }

    // ROS 인터페이스
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr pose_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr ik_solution_pub_;

    sensor_msgs::msg::JointState::SharedPtr current_joint_state_;

    // MoveIt 관련
    robot_model_loader::RobotModelLoaderPtr robot_model_loader_;
    moveit::core::RobotModelConstPtr robot_model_;
    moveit::core::RobotStatePtr robot_state_;
    const moveit::core::JointModelGroup* joint_model_group_;
    std::string planning_group_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    // 반드시 shared_ptr로 생성 (enable_shared_from_this를 위해)
    auto node = std::make_shared<IKSubscriber>();
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}



