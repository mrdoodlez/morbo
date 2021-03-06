#include "rclcpp/rclcpp.hpp"
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <termios.h>
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "example_interfaces/msg/float32_multi_array.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include <sstream>
#include <deque>
#include <RTIMULib.h>

#define DT	30

extern "C" {
	#include "../../../../machine_control/motor_control/mc_proto.h"
}

class DiffDriveNode : public rclcpp::Node {
public:
	DiffDriveNode() : Node("morbo_diff_drive")
					, prevTs(0)
                    , runMode(eRunMode_Normal)
					, fLinear(4)
					, fAngular(4)
					, fTh(4) {

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
		
		turretSubscriber = create_subscription<example_interfaces::msg::Float32MultiArray>(
				"turret_pos",
				10,
				std::bind(&DiffDriveNode::TurretPosCallback,
				this,
				std::placeholders::_1));

		auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
		odomPublisher = create_publisher<nav_msgs::msg::Odometry>("odom", qos);
	
		transformBroadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);

		tmr = create_wall_timer(std::chrono::milliseconds(DT),
		   std::bind(&DiffDriveNode::TimerCallback, this));

		declare_parameter("inipath", "./");
		auto inipath = get_parameter("inipath").as_string();

		RTIMUSettings *settings = new RTIMUSettings(inipath.c_str());

		imu = RTIMU::createIMU(settings);

		if ((imu == NULL) || (imu->IMUType() == RTIMU_TYPE_NULL)) {
			RCLCPP_INFO(get_logger(), "[FATAL] No IMU found!");
			exit(-1);
		}

		imu->IMUInit();

		imu->setSlerpPower(0.02);
		imu->setGyroEnable(true);
		imu->setAccelEnable(true);
		imu->setCompassEnable(true);
	}

private:
	void CmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
		setLinear = msg->linear.x;
		setAngular = msg->angular.z;
	}

	void TurretPosCallback(const example_interfaces::msg::Float32MultiArray::SharedPtr msg) {
		setH = msg->data[0];
		setV = msg->data[1];
		cannonOn = msg->data[2] > 0;
	}

	void GetOdom(std::pair<double, double>& vels, RTIMU_DATA& imuData) {
		mc_control_cmd_t cmd;
		cmd.m = 'm';
		cmd.b = 'b';
		cmd.code = MC_RC_CODE_GET_ENCODERS;

		if (write(uart, &(cmd), sizeof(cmd)) == sizeof(cmd)) {
			memset(&cmd, 0, sizeof(cmd));
			if (read(uart, &cmd, sizeof(cmd)) == sizeof(cmd)) {
				if ((cmd.m == 'm') && (cmd.b == 'b')) {
					mc_control_encoders_t *encoders
						= (mc_control_encoders_t*)(cmd.payload);

					rcutils_time_point_value_t now;
					rcutils_system_time_now(&now);

					uint64_t currTime = RCL_NS_TO_US(now);

					if (prevTs != 0) {
						double dt = currTime - prevTs;
						int dpl = encoders->pulses_l - prevEnc.pulses_l;
						int dpr = encoders->pulses_r - prevEnc.pulses_r;

						vels.first = 0.5 * (dpl + dpr) * mpp / dt * 1e6;
						vels.second = (dpr - dpl) * rpp / dt * 1e6;
					}

					if (runMode == DiffDriveNode::eRunMode_Calib) {
						if (encoders->pulses_l != prevEnc.pulses_l) {
							std::ostringstream oss;
							oss << " enc: {" << encoders->pulses_l << " "
								<< encoders->pulses_r << "}";
							RCLCPP_INFO(get_logger(), oss.str());
						}
					}

					prevTs = currTime;
					prevEnc = *encoders;

				}
			} else
				; //TODO: handle error
		} else
			; //TODO: handle error

		if (!imu->IMURead()) {
			RCLCPP_INFO(get_logger(), "[FATAL] Failed to read IMU data");
			exit(-1);
		}

		imuData = imu->getIMUData();
	}

	void TimerCallback() {
		static struct {
			float posV;
			float posH;
			bool cannonOn;
		} turretState;

		if ((setV != turretState.posV)
			|| (setH != turretState.posH)
			|| (cannonOn != turretState.cannonOn)) {

			SetTurret(setV, setH, cannonOn);

			turretState.posV = setV;
			turretState.posH = setH;
			turretState.cannonOn = cannonOn;

			return;
		}

		std::pair<double, double> vels;
		RTIMU_DATA imuData;

		GetOdom(vels, imuData);

		double currLinear  = 0;
		double currAngular = fAngular.Add(-imuData.gyro.z());

		float pwmLeft = 0;
		float pwmRight = 0;

		if ((setLinear == 0) && (setAngular == 0)) {
			Sl = 0;
			Sr = 0;
			fLinear.Clear();
		} else if (runMode == DiffDriveNode::eRunMode_Calib) {
			if (setLinear > 0)
				pwmLeft = pwmRight = 1;
			else if (setLinear < 0)
				pwmLeft = pwmRight = -1;
			else if (setAngular > 0) {
				pwmLeft = -1;
				pwmRight = 1;
			} else {
				pwmLeft = 1;
				pwmRight = -1;
			}
		} else {
			currLinear = fLinear.Add(vels.first);
			if (setLinear == 0) {
				Sl = 0;
				Sr = 0;
				if (setAngular > 0) {
					pwmLeft = -1;
					pwmRight = 1;
				} else {
					pwmLeft = 1;
					pwmRight = -1;
				}
			} else {
				auto eLinear = setLinear - currLinear;
				auto eAngular = setAngular - currAngular;

				auto el = kl * eLinear + kal * eAngular;
				auto er = kl * eLinear + kar * eAngular;

				Sl += el;
				Sr += er;

				pwmLeft  = kp * el + ki * Sl;
				pwmRight = kp * er + ki * Sr;

				if (pwmLeft >  1) pwmLeft  = 1;
				if (pwmLeft < -1) pwmLeft = -1;

				if (pwmRight >  1) pwmRight  = 1;
				if (pwmRight < -1) pwmRight = -1;
			}
		}


		double dt = DT * 1.0e-3; //TODO: fix it!

		th = fTh.Add(-imuData.fusionPose.z());

		auto vx = currLinear * cos (th);
		auto vy = currLinear * sin (th);

		x += vx * dt;
		y += vy * dt;

		std::ostringstream oss;

		oss << "{" << currLinear << " " << currAngular <<
			" " << x << " " << y << " " << th << "} ";

		RCLCPP_INFO(get_logger(), oss.str());

		rcutils_time_point_value_t now;
		rcutils_system_time_now(&now);

		auto odom = std::make_unique<nav_msgs::msg::Odometry>();

		odom->header.stamp.sec = RCL_NS_TO_S(now);
		odom->header.stamp.nanosec = now - RCL_S_TO_NS(odom->header.stamp.sec);
		odom->header.frame_id = "odom";

		odom->pose.pose.position.x = x;
		odom->pose.pose.position.y = y;
		odom->pose.pose.position.z = 0.0;

		tf2::Quaternion q;
		q.setRPY(0, 0, th);

		odom->pose.pose.orientation.x = q.x();
		odom->pose.pose.orientation.y = q.y();
		odom->pose.pose.orientation.z = q.z();
		odom->pose.pose.orientation.w = q.w();

		odom->twist.twist.linear.x = vx;
		odom->twist.twist.linear.y = vy;
		odom->twist.twist.angular.z = currAngular;

		auto odom_tf = std::make_unique<geometry_msgs::msg::TransformStamped>();
		odom_tf->header.frame_id = "odom";
		odom_tf->child_frame_id = "base_link";

		odom_tf->header.stamp = odom->header.stamp;
		odom_tf->transform.translation.x = x;
		odom_tf->transform.translation.y = y;
		odom_tf->transform.translation.z = 0.0;
		odom_tf->transform.rotation.x = q.x();
		odom_tf->transform.rotation.y = q.y();
		odom_tf->transform.rotation.z = q.z();
		odom_tf->transform.rotation.w = q.w();

		transformBroadcaster->sendTransform(*odom_tf);
		odomPublisher->publish(move(odom));

		SendPwm( {pwmLeft, pwmRight} );
	}

	void SendPwm(const std::pair<float, float>& pwm) const {
		mc_control_cmd_t cmd;
		cmd.m = 'm';
		cmd.b = 'b';
		cmd.code = MC_RC_CODE_SET_PWM;

		mc_control_speeds_t *speeds = (mc_control_speeds_t*)&(cmd.payload);
		speeds->pwm_l = pwm.first;
		speeds->pwm_r = pwm.second;

		if (write(uart, &cmd, sizeof(cmd)) != sizeof(cmd))
			; //TODO: handle error
	}

	void SetTurret(float posV, float posH, bool cannonOn) {
		mc_control_cmd_t cmd;
		cmd.m = 'm';
		cmd.b = 'b';
		cmd.code = MC_RC_CODE_SET_TURRET;

		mc_control_turret_t *trt = (mc_control_turret_t*)&(cmd.payload);
		trt->angle_v = posV;
		trt->angle_h = posH;
		trt->cannon = cannonOn;

		if (write(uart, &cmd, sizeof(cmd)) != sizeof(cmd))
			; //TODO: handle error
	}

	rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velSubscriber;
	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odomPublisher;
	std::shared_ptr<tf2_ros::TransformBroadcaster> transformBroadcaster;
	rclcpp::TimerBase::SharedPtr tmr;
	rclcpp::Subscription<example_interfaces::msg::Float32MultiArray>::SharedPtr turretSubscriber;

	int uart;

	mc_control_encoders_t prevEnc;
	uint64_t prevTs;

	double setLinear;
	double setAngular;

	float setV;
	float setH;
	bool cannonOn;

	double x;
	double y;
	double th;

	class MovAvg {
	public:
		MovAvg(int len_) : len(len_) {}

		double Add(double newSample) {
			if (samples.size() == len)
				samples.pop_front();

			samples.push_back(newSample);

			double res = 0;
			for (auto x : samples)
				res += x;

			return  res / samples.size();
		}

		void Clear() {
			samples.clear();
		}

	private:
		const int len;
		std::deque<double> samples;
	} fLinear, fAngular, fTh;

	static constexpr double ppr = 103;
	static constexpr double rpp = 1.0 / ppr;

	static constexpr double ppm = 342;
	static constexpr double mpp = 1.0 / ppm;

	static constexpr double ki		= 0.1;
	static constexpr double kp		= 2;
	static constexpr double kl		= 1;
	static constexpr double kal		= -0.01;
	static constexpr double kar		=  0.01;

	double Sl;
	double Sr;

	RTIMU *imu;

	enum RunMode {
		eRunMode_Normal,
		eRunMode_Calib,
	} runMode;
};

int main(int argc, char **argv) {
	rclcpp::init(argc, argv);

	auto node = std::make_shared<DiffDriveNode>();

	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}
