#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

class DiffDriveNode : public rclcpp::Node {
public:
	DiffDriveNode() : Node("morbo_diff_drive") {
		RCLCPP_INFO(get_logger(), "morbo_diff_drive node started");

		velSubscriber = create_subscription<geometry_msgs::msg::Twist>(
				"turtle1/cmd_vel",
				10,
				std::bind(&DiffDriveNode::CmdVelCallback,
				this,
				std::placeholders::_1));
	}

private:
	void CmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr) const {
		RCLCPP_INFO(get_logger(), "I heard");
	}

	rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velSubscriber;
};

int main(int argc, char **argv) {
	rclcpp::init(argc, argv);

	auto node = std::make_shared<DiffDriveNode>();

	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}
