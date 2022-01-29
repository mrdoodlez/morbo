#include "rclcpp/rclcpp.hpp"
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include "geometry_msgs/msg/twist.hpp"
#include <sstream>

extern "C" {
	#include <linux/i2c-dev.h>
}

#define TRANSFER_CMD_LEN	8
#define TRANSFER_ADDRESS	0x3f

enum MorboCmds {
	CODE_SET_PWM,
	CODE_GET_ENCODERS,
	CODE_GET_IMU
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

		tmr = create_wall_timer(std::chrono::milliseconds(100),
		   std::bind(&DiffDriveNode::TimerCallback, this));
	}

private:
	void CmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) const {
		auto set_linear = msg->linear.x;
		auto set_angular = msg->angular.z;

		auto [curr_linear, curr_angular] = GetCurrVels();

		std::pair<uint8_t, uint8_t> pwm {0, 0};

		if (set_linear > 0)
			pwm = {0x7f, 0x7f};
		else if (set_linear < 0)
			pwm = {0xff, 0xff};
		else if (set_angular > 0)
			pwm = {0x7f, 0xff};
		else if (set_angular < 0)
			pwm = {0xff, 0x7f};

		//SendPwm(pwm);

		std::ostringstream oss;
		oss << "set: {" << set_linear << " " << set_angular << "}";
		//oss << " curr: {" << curr_linear << " " << curr_angular << "}";

		//RCLCPP_INFO(get_logger(), oss.str());
	}

	std::pair<double, double> GetCurrVels() const {
		/*
		*/

		std::vector<uint8_t> cmd(TRANSFER_CMD_LEN);

		std::fill(cmd.begin(), cmd.end(), 0xff);
		cmd[0] = CODE_GET_ENCODERS;

		if (write(i2c, &cmd[0], cmd.size()) == cmd.size())
		{
			usleep(100);
			if (read(i2c, &cmd[0], 8) != 8)
				; //TODO: handle error
		}


		bool flag = false;

		std::ostringstream oss;
		for (int i = 0; i < 8; i++)
			if (cmd[i] != 255) {
				oss << "{" << i << " " << (int)(cmd[i]) << "} ";
				flag = true;
			}

		if (flag)
			RCLCPP_INFO(get_logger(), oss.str());


		/*
		std::vector<uint8_t> cmd(TRANSFER_PL_LEN - 2);
		std::fill(cmd.begin(), cmd.end(), 0xff);

		i2c_smbus_write_i2c_block_data(i2c, CODE_GET_ENCODERS, cmd.size(), &cmd[0]);
		i2c_smbus_read_i2c_block_data(i2c, CODE_GET_ENCODERS, 4, &cmd[0]);

		for (int i = 0; i < 4; i++)
			std::cout << (int)(cmd[i]) << " ";
		std::cout << std::endl;
		*/

		return {0, 0};
	}

	void TimerCallback() {
		/*
		RCLCPP_INFO(get_logger(), "timer");

		*/
	}

	void SendPwm(const std::pair<uint8_t, uint8_t>& pwm) const
	{
		std::ostringstream oss;
		oss << "send pwm: {" << (int)(pwm.first) << " " << (int)(pwm.second) << "}";
		RCLCPP_INFO(get_logger(), oss.str());

		std::vector<uint8_t> cmd(TRANSFER_CMD_LEN);

		std::fill(cmd.begin(), cmd.end(), 0xff);
		cmd[0] = CODE_SET_PWM;
		cmd[1] = pwm.first;
		cmd[2] = pwm.second;

		if (write(i2c, &cmd[0], cmd.size()) != cmd.size())
			; //TODO: handle error
	}

	rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velSubscriber;
	rclcpp::TimerBase::SharedPtr tmr;
	int i2c;
};

int main(int argc, char **argv) {
	rclcpp::init(argc, argv);

	auto node = std::make_shared<DiffDriveNode>();

	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}
