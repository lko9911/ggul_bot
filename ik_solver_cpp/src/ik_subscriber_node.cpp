#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <iostream>
#include <vector>
#include <jsoncpp/json/json.h>  // JSON 변환을 위한 라이브러리
#include <asio.hpp>  // Asynchronous I/O library (websocket을 사용하기 위한 라이브러리)

class IKSubscriber
{
public:
    IKSubscriber(const rclcpp::Node::SharedPtr& node)
        : node_(node)
    {
        // 로봇 모델 로더 초기화 (URDF 로딩)
        robot_model_loader_ = std::make_shared<robot_model_loader::RobotModelLoader>(node_, "robot_description");

        robot_model_ = robot_model_loader_->getModel();
        robot_state_ = std::make_shared<moveit::core::RobotState>(robot_model_);

        planning_group_ = "manipulator2";  
        joint_model_group_ = robot_model_->getJointModelGroup(planning_group_);

        pose_sub_ = node_->create_subscription<geometry_msgs::msg::Pose>(
            "/target_pose", 10,
            std::bind(&IKSubscriber::poseCallback, this, std::placeholders::_1));

        joint_state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            std::bind(&IKSubscriber::jointStateCallback, this, std::placeholders::_1));

        // WebSocket 클라이언트 설정 (Python 서버와 통신)
        websocket_endpoint_ = "ws://192.168.150.77:8765"; // 웹소켓 서버 주소
        asio::io_context io_context;
        socket_ = std::make_shared<asio::ip::tcp::socket>(io_context);
        
        // WebSocket 연결을 위한 코드 추가
        connectWebSocket();

        RCLCPP_INFO(node_->get_logger(), "IK Subscriber initialized");
    }

    void poseCallback(const geometry_msgs::msg::Pose::SharedPtr msg)
    {
        RCLCPP_INFO(node_->get_logger(), "Received pose, calculating IK...");

        bool found_ik = robot_state_->setFromIK(joint_model_group_, *msg);

        if (found_ik)
        {
            std::vector<double> joint_values;
            robot_state_->copyJointGroupPositions(joint_model_group_, joint_values);

            // IK 솔루션을 JSON으로 변환 후 웹소켓으로 전송
            sendIKSolution(joint_values);

            RCLCPP_INFO(node_->get_logger(), "IK Solution:");
            for (size_t i = 0; i < joint_values.size(); ++i)
            {
                RCLCPP_INFO(node_->get_logger(), "  Joint %zu: %.3f", i, joint_values[i]);
            }
        }
        else
        {
            RCLCPP_WARN(node_->get_logger(), "No IK solution found");
        }
    }

private:
    void sendIKSolution(const std::vector<double>& joint_values)
    {
        // JSON 형식으로 IK 솔루션 변환
        Json::Value json_data;
        json_data["IK_Solution"] = Json::arrayValue;

        for (size_t i = 0; i < joint_values.size(); ++i)
        {
            json_data["IK_Solution"].append(joint_values[i]);
        }

        // JSON 객체를 문자열로 변환
        Json::StreamWriterBuilder writer;
        std::string json_string = Json::writeString(writer, json_data);

        // 웹소켓으로 메시지 전송 (Python 서버로)
        asio::write(*socket_, asio::buffer(json_string));
    }

    void connectWebSocket()
    {
        try
        {
            asio::ip::tcp::resolver resolver(socket_->get_io_service());
            asio::connect(*socket_, resolver.resolve("localhost", "8765"));
        }
        catch (const std::exception& e)
        {
            RCLCPP_ERROR(node_->get_logger(), "Failed to connect to WebSocket server: %s", e.what());
        }
    }

    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr pose_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;

    std::shared_ptr<asio::ip::tcp::socket> socket_;
    std::string websocket_endpoint_;
    std::shared_ptr<robot_model_loader::RobotModelLoader> robot_model_loader_;
    moveit::core::RobotModelConstPtr robot_model_;
    moveit::core::RobotStatePtr robot_state_;
    const moveit::core::JointModelGroup* joint_model_group_;
    std::string planning_group_;
};

