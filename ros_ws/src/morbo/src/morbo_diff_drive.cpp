#include "rclcpp/rclcpp.hpp"
#include <fcntl.h>
#include <sys/ioctl.h>
#include "geometry_msgs/msg/twist.hpp"
#include <sstream>

extern "C" {
	#include <linux/i2c-dev.h>
	#include <i2c/smbus.h>
}

#define TRANSFER_CMD_LEN	16
#define TRANSFER_PL_LEN		(TRANSFER_CMD_LEN - 1)

#define TRANSFER_ADDRESS	0x3f

enum MorboCmds {
	CODE_ACK    = 0,
	CODE_NACK,
	CODE_PING,
	CODE_STOP,
	CODE_PWM
};

class DiffDriveNode : public rclcpp::Node {
public:
	DiffDriveNode() : Node("morbo_diff_drive") {
		if ((i2c = open("/dev/i2c-1", O_RDWR)) < 0) {
			RCLCPP_INFO(get_logger(), "[FATAL] Failed to open /dev/i2c-1");
			exit(-1);
		}

		if (ioctl(i2c, I2C_SLAVE, TRANSFER_ADDRESS) < 0)
		{
			RCLCPP_INFO(get_logger(), "[FATAL] Failed to set i2c slave addr");
			exit(-1);
		}

		RCLCPP_INFO(get_logger(), "morbo_diff_drive node started");

		velSubscriber = create_subscription<geometry_msgs::msg::Twist>(
				"cmd_vel",
				10,
				std::bind(&DiffDriveNode::CmdVelCallback,
				this,
				std::placeholders::_1));
	}

private:
	void CmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) const {
		std::ostringstream oss;
		oss << "{" << msg->linear.x << " " << msg->linear.y
			<< " " << msg->linear.z << "} ";
		oss << "{" << msg->angular.x << " " << msg->angular.y
			<< " " << msg->angular.z << "} ";
		RCLCPP_INFO(get_logger(), oss.str());

		auto pwm = VelsToPwm(msg->linear.x, msg->angular.z);

		SendPwm(pwm);
	}

	std::pair<uint8_t, uint8_t> VelsToPwm(double x, double z) const
	{
		if (x > 0)
			return std::pair<uint8_t, uint8_t>{0x7f, 0x7f};
		if (x < 0)
			return std::pair<uint8_t, uint8_t>{0xff, 0xff};
		if (z > 0)
			return std::pair<uint8_t, uint8_t>{0x7f, 0xff};
		if (z < 0)
			return std::pair<uint8_t, uint8_t>{0xff, 0x7f};
		return std::pair<uint8_t, uint8_t>{0, 0};
	}

	void SendPwm(const std::pair<uint8_t, uint8_t>& pwm) const
	{
		std::ostringstream oss;
		oss << "{" << (int)(pwm.first) << " " << (int)(pwm.second) << "}";
		RCLCPP_INFO(get_logger(), oss.str());

		std::vector<uint8_t> cmd(TRANSFER_PL_LEN);

		std::fill(cmd.begin(), cmd.end(), 0xff);
		cmd[0] = pwm.first;
		cmd[1] = pwm.second;

		i2c_smbus_write_i2c_block_data(i2c, CODE_PWM, cmd.size(), &cmd[0]);
	}

	rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velSubscriber;
	int i2c;
};

int main(int argc, char **argv) {
	rclcpp::init(argc, argv);

	auto node = std::make_shared<DiffDriveNode>();

	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}
