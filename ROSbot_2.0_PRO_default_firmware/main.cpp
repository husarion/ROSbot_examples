#include "hFramework.h"
#include "hCloudClient.h"
#include "ros.h"
#include "tf/tf.h"
#include "transform_broadcaster.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TransformStamped.h"
#include "sensor_msgs/BatteryState.h"
#include "std_msgs/Bool.h"
#include "sensor_msgs/Range.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/Vector3.h"
#include "ROSbot.h"

using namespace hFramework;

// Uncomment one of these lines, accordingly to range sensor type of your ROSbot
// If you have version with infrared sensor:
// static const SensorType sensor_type = SENSOR_INFRARED;
// If you have version with laser sensor:
static const SensorType sensor_type = SENSOR_LASER;
// If you want to use your own sensor:
// static const SensorType sensor_type = NO_DISTANCE_SENSOR;

// Uncomment one of these lines, accordingly to IMU sensor type of your device
// If you have version with MPU9250:
static const ImuType imu_type = MPU9250;
// If you want to use your own sensor:
// static const ImuType imu_type = NO_IMU;

// Uncomment one of these lines, accordingly version of your device
// uint32_t baudrate = 500000; // for ROSbot 2.0
uint32_t baudrate = 230400; // for ROSbot 2.0 PRO 

ros::NodeHandle nh;
sensor_msgs::BatteryState battery;
ros::Publisher *battery_pub;
sensor_msgs::JointState joint_states;
ros::Publisher *joint_state_pub;
geometry_msgs::PoseStamped pose;
ros::Publisher *pose_pub;
geometry_msgs::Vector3 imuArray;
ros::Publisher *imu_pub;

sensor_msgs::Range range_fl;
sensor_msgs::Range range_fr;
sensor_msgs::Range range_rl;
sensor_msgs::Range range_rr;

ros::Publisher *range_pub_fl;
ros::Publisher *range_pub_fr;
ros::Publisher *range_pub_rl;
ros::Publisher *range_pub_rr;

std::vector<float> rosbot_pose;
std::vector<float> rpy;
std::vector<float> ranges;
wheelsState ws;

geometry_msgs::TransformStamped robot_tf;
tf::TransformBroadcaster broadcaster;

int publish_counter = 0;

void twistCallback(const geometry_msgs::Twist &twist)
{
	rosbot.setSpeed(twist.linear.x, twist.angular.z);
}

void initCmdVelSubscriber()
{
	ros::Subscriber<geometry_msgs::Twist> *cmd_sub = new ros::Subscriber<geometry_msgs::Twist>("/cmd_vel", &twistCallback);
	nh.subscribe(*cmd_sub);
}

void resetCallback(const std_msgs::Bool &msg)
{
	if (msg.data == true)
	{
		rosbot.reset_odometry();
	}
}

void initResetOdomSubscriber()
{
	ros::Subscriber<std_msgs::Bool> *odom_reset_sub = new ros::Subscriber<std_msgs::Bool>("/reset_odom", &resetCallback);
	nh.subscribe(*odom_reset_sub);
}

void initDistanceSensorsPublisher()
{
	range_fl.header.frame_id = "range_fl";
	range_fr.header.frame_id = "range_fr";
	range_rl.header.frame_id = "range_rl";
	range_rr.header.frame_id = "range_rr";

	switch (sensor_type)
	{
	case SENSOR_LASER:
		range_fl.field_of_view = 0.26;
		range_fl.min_range = 0.03;
		range_fl.max_range = 0.90;

		range_fr.field_of_view = 0.26;
		range_fr.min_range = 0.03;
		range_fr.max_range = 0.90;

		range_rl.field_of_view = 0.26;
		range_rl.min_range = 0.03;
		range_rl.max_range = 0.90;

		range_rr.field_of_view = 0.26;
		range_rr.min_range = 0.03;
		range_rr.max_range = 0.90;
		break;
	case SENSOR_INFRARED:
		range_fl.radiation_type = sensor_msgs::Range::INFRARED;
		range_fl.field_of_view = 0.26;
		range_fl.min_range = 0.05;
		range_fl.max_range = 0.299;

		range_fr.radiation_type = sensor_msgs::Range::INFRARED;
		range_fr.field_of_view = 0.26;
		range_fr.min_range = 0.05;
		range_fr.max_range = 0.299;

		range_rl.radiation_type = sensor_msgs::Range::INFRARED;
		range_rl.field_of_view = 0.26;
		range_rl.min_range = 0.05;
		range_rl.max_range = 0.299;

		range_rr.radiation_type = sensor_msgs::Range::INFRARED;
		range_rr.field_of_view = 0.26;
		range_rr.min_range = 0.05;
		range_rr.max_range = 0.299;
		break;
	case NO_DISTANCE_SENSOR:
		// Do your own implementation
		break;
	}

	if (sensor_type != SensorType::NO_DISTANCE_SENSOR)
	{
		range_pub_fl = new ros::Publisher("/range/fl", &range_fl);
		range_pub_fr = new ros::Publisher("/range/fr", &range_fr);
		range_pub_rl = new ros::Publisher("/range/rl", &range_rl);
		range_pub_rr = new ros::Publisher("/range/rr", &range_rr);
		nh.advertise(*range_pub_fl);
		nh.advertise(*range_pub_fr);
		nh.advertise(*range_pub_rl);
		nh.advertise(*range_pub_rr);
	}
}

void initBatteryPublisher()
{
	battery_pub = new ros::Publisher("/battery", &battery);
	nh.advertise(*battery_pub);
}

void initPosePublisher()
{
	pose.header.frame_id = "odom";
	pose.pose.orientation = tf::createQuaternionFromYaw(0);
	pose_pub = new ros::Publisher("/pose", &pose);
	nh.advertise(*pose_pub);
}

void initIMUPublisher()
{
	switch (imu_type)
	{
	case MPU9250:
		imu_pub = new ros::Publisher("/rpy", &imuArray);
		nh.advertise(*imu_pub);
		break;
	case NO_IMU:
		break;
	}
}

void initJointStatePublisher()
{
	joint_state_pub = new ros::Publisher("/joint_states", &joint_states);
	nh.advertise(*joint_state_pub);
}

void initTfPublisher()
{
	robot_tf.header.frame_id = "odom";
	robot_tf.child_frame_id = "base_link";
	robot_tf.transform.translation.x = 0.0;
	robot_tf.transform.translation.y = 0.0;
	robot_tf.transform.translation.z = 0.0;
	robot_tf.transform.rotation.x = 0.0;
	robot_tf.transform.rotation.y = 0.0;
	robot_tf.transform.rotation.z = 0.0;
	robot_tf.transform.rotation.w = 1.0;
	broadcaster.init(nh);
}

void hMain()
{
	uint32_t t = sys.getRefTime();
	rosbot.initROSbot(sensor_type, imu_type);
	RPi.init(baudrate);
	platform.begin(&RPi);
	nh.getHardware()->initWithDevice(&platform.LocalSerial);
	// nh.getHardware()->initWithDevice(&RPi);
	nh.initNode();

	initBatteryPublisher();
	initPosePublisher();
	initIMUPublisher();
	initDistanceSensorsPublisher();
	initCmdVelSubscriber();
	initResetOdomSubscriber();
	initJointStatePublisher();
	initTfPublisher();

	joint_states.header.frame_id = "base_link";

	//creating the arrays for the message
	char *name[] = {"front_left_wheel_hinge", "front_right_wheel_hinge", "rear_left_wheel_hinge", "rear_right_wheel_hinge"};
	float pos[] = {0, 0, 0, 0};
	float vel[] = {0, 0, 0, 0};
	float eff[] = {0, 0, 0, 0};

	//assigning the arrays to the message
	joint_states.name = name;
	joint_states.position = pos;
	joint_states.velocity = vel;
	joint_states.effort = eff;

	//setting the length
	joint_states.name_length = 4;
	joint_states.position_length = 4;
	joint_states.velocity_length = 4;
	joint_states.effort_length = 4;

	while (true)
	{
		nh.spinOnce();
		publish_counter++;
		if (publish_counter > 10)
		{
			// get ROSbot pose
			rosbot_pose = rosbot.getPose();
			pose.pose.position.x = rosbot_pose[0];
			pose.pose.position.y = rosbot_pose[1];
			pose.pose.orientation = tf::createQuaternionFromYaw(rosbot_pose[2]);
			// publish pose
			pose_pub->publish(&pose);

			// get ROSbot tf
			robot_tf.header.stamp = nh.now();
			robot_tf.transform.translation.x = pose.pose.position.x;
			robot_tf.transform.translation.y = pose.pose.position.y;
			robot_tf.transform.rotation.x = pose.pose.orientation.x;
			robot_tf.transform.rotation.y = pose.pose.orientation.y;
			robot_tf.transform.rotation.z = pose.pose.orientation.z;
			robot_tf.transform.rotation.w = pose.pose.orientation.w;
			// publish tf
			broadcaster.sendTransform(robot_tf);

			if (sensor_type != SensorType::NO_DISTANCE_SENSOR)
			{
				// get ranges from distance sensors
				ranges = rosbot.getRanges(sensor_type);
				range_fl.range = ranges[0];
				range_fr.range = ranges[1];
				range_rl.range = ranges[2];
				range_rr.range = ranges[3];
				Serial.printf("Ranges %f %f %f %f\n", ranges[0], ranges[1], ranges[2], ranges[3]);
				// publish ranges
				range_pub_fl->publish(&range_fl);
				range_pub_fr->publish(&range_fr);
				range_pub_rl->publish(&range_rl);
				range_pub_rr->publish(&range_rr);
			}

			if (imu_type != ImuType::NO_IMU)
			{
				// get roll, pitch and yaw angles from IMU
				rpy = rosbot.getRPY();
				imuArray.x = rpy[0];
				imuArray.y = rpy[1];
				imuArray.z = rpy[2];
				// publish RPY
				imu_pub->publish(&imuArray);
			}

			ws = rosbot.getWheelsState();
			pos[0] = ws.FL;
			pos[1] = ws.FR;
			pos[2] = ws.RL;
			pos[3] = ws.RR;
			joint_states.position = pos;
			joint_states.header.stamp = nh.now();
			joint_state_pub->publish(&joint_states);

			// get battery voltage
			battery.voltage = rosbot.getBatteryLevel();
			// publish battery voltage
			battery_pub->publish(&battery);
			publish_counter = 0;
		}
		sys.delaySync(t, 10);
	}
}
