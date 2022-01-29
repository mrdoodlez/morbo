#include "rclcpp/rclcpp.hpp"
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <termios.h>
#include "geometry_msgs/msg/twist.hpp"
#include <sstream>

extern "C" {
	//#include <linux/i2c-dev.h>
	#include "../../../../machine_control/motor_control/mc_proto.h"
}

//#define TRANSFER_ADDRESS	0x3f

class DiffDriveNode : public rclcpp::Node {
public:
	DiffDriveNode() : Node("morbo_diff_drive") {
		/*
		if ((i2c = open("/dev/i2c-1", O_RDWR)) < 0) {
			RCLCPP_INFO(get_logger(), "[FATAL] Failed to open /dev/i2c-1");
			exit(-1);
		}

		if (ioctl(i2c, I2C_SLAVE, TRANSFER_ADDRESS) < 0)
		{
			RCLCPP_INFO(get_logger(), "[FATAL] Failed to set i2c slave addr");
			exit(-1);
		}
		*/

		const char uartFileName[] = "/dev/ttyUSB0";

		struct termios tio;

		uart = open(uartFileName, O_RDWR | O_NOCTTY);
		if (uart < 0) {
			RCLCPP_INFO(get_logger(), "[FATAL] Failed to open uart file");
			exit(-1);
		}

		bzero(&tio, sizeof(tio));
		tio.c_cflag = B115200 | CS8 | CLOCAL | CREAD;
		tio.c_iflag = IGNPAR;
		tio.c_oflag = 0;

		// set input mode (non-canonical, no echo,...)
		tio.c_lflag = 0;

		tio.c_cc[VTIME]    = 0;  // inter-character timer unused
		tio.c_cc[VMIN]     = BOARD_TRANSFER_CHUNK;  // blocking read until 5 chars received

		tcflush(uart, TCIOFLUSH);
		if (tcsetattr(uart, TCSANOW, &tio)) {
			RCLCPP_INFO(get_logger(), "[FATAL] Failed to set tcattr");
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

		SendPwm(pwm);

		std::ostringstream oss;
		oss << "set: {" << set_linear << " " << set_angular << "}";
		//oss << " curr: {" << curr_linear << " " << curr_angular << "}";

		RCLCPP_INFO(get_logger(), oss.str());
	}

	std::pair<double, double> GetCurrVels() const {
		mc_control_cmd_t cmd;
		cmd.m = 'm';
		cmd.b = 'b';
		cmd.code = MC_RC_CODE_GET_ENCODERS;

		if (write(uart, &(cmd), sizeof(cmd)) == sizeof(cmd)) {
			usleep(100);
			memset(&cmd, 0, sizeof(cmd));
			if (read(uart, &cmd, sizeof(cmd)) == sizeof(cmd)) {
				if ((cmd.m == 'm') && (cmd.b == 'b')) {
					mc_control_encoders_t *encoders
						= (mc_control_encoders_t*)(cmd.payload);

					std::ostringstream oss;
					oss << "curr: {" << encoders->pulses_l << " " << encoders->pulses_r << "}";

					RCLCPP_INFO(get_logger(), oss.str());
				}
			} else
				; //TODO: handle error
		} else
			; //TODO: handle error

		return {0, 0};
	}

	void TimerCallback() {
		/*
		RCLCPP_INFO(get_logger(), "timer");

		*/
	}

	void SendPwm(const std::pair<uint8_t, uint8_t>& pwm) const {
		std::ostringstream oss;
		oss << "send pwm: {" << (int)(pwm.first) << " " << (int)(pwm.second) << "}";
		RCLCPP_INFO(get_logger(), oss.str());

		mc_control_cmd_t cmd;
		cmd.m = 'm';
		cmd.b = 'b';
		cmd.code = MC_RC_CODE_SET_PWM;

		mc_control_speeds_t *speeds = (mc_control_speeds_t*)&(cmd.payload);
		speeds->speed_l = pwm.first;
		speeds->speed_r = pwm.second;

		if (write(uart, &cmd, sizeof(cmd)) != sizeof(cmd))
			; //TODO: handle error
	}

	rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velSubscriber;
	rclcpp::TimerBase::SharedPtr tmr;
	int i2c;
	int uart;
};

int main(int argc, char **argv) {
	rclcpp::init(argc, argv);

	auto node = std::make_shared<DiffDriveNode>();

	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}
