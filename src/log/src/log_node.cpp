/**
 * Read a messages from other nodes and plot information
 */


//ROS
#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <path_planning/Q_real.h>
#include <path_planning/Q_desired.h>

//STD
#include <iostream>
#include <fstream>
#include <array>
#include <deque>
#include <mutex>

// This must be defined before the first time that "gnuplot-iostream.h" is included.
#define GNUPLOT_ENABLE_CXX11
#include "gnuplot-iostream.h"

//ROS Paths
#define SUB_BALL "balltracker/points"
#define SUB_KALMAN "kalman_filter/points"
#define SUB_CAMERA_POS_DESIRED "path_planning/camera_pose_desired"
#define SUB_CAMERA_POS_REAL "path_planning/camera_pose_real"
#define SUB_Q_DESIRED "path_planning/q_desired"
#define SUB_Q_REAL "path_planning/q_real"
#define SRV_GET_JOINT "/getJointConfig"
#define PARAM_DEBUGGING "/points_server/debugging"
#define PARAM_FRAME_RATE "/frame_rate"

constexpr int BUFFERS_SIZE = 10;

std::deque<geometry_msgs::PointStamped> g_ball_points;
std::deque<geometry_msgs::PointStamped> g_kalman_points;
std::deque<geometry_msgs::PoseStamped> g_camera_desired_poses;
std::deque<geometry_msgs::PoseStamped> g_camera_real_poses;
std::deque<std::array<float, 7>> g_real_qs;
std::deque<std::array<float, 7>> g_desired_qs;

std::mutex g_mutex_ball_points;
std::mutex g_mutex_kalman_points;
std::mutex g_mutex_camera_desired_poses;
std::mutex g_mutex_camera_real_poses;
std::mutex g_mutex_real_qs;
std::mutex g_mutex_desired_qs;

// Tells gnuplot-iostream how to print these objects.
namespace gnuplotio
{
template<>
struct BinfmtSender<geometry_msgs::PointStamped> {
	static void send(std::ostream &stream)
	{
		BinfmtSender<double>::send(stream);
		BinfmtSender<double>::send(stream);
		BinfmtSender<double>::send(stream);
	}
};

template<>
struct BinarySender<geometry_msgs::PointStamped> {
	static void send(std::ostream &stream, geometry_msgs::PointStamped const &ps)
	{
		BinarySender<double>::send(stream, ps.point.x);
		BinarySender<double>::send(stream, ps.point.y);
		BinarySender<double>::send(stream, ps.point.z);
	}
};
} // namespace gnuplotio

// Writes CSV file
class FileWriter
{
private:
	typedef geometry_msgs::PointStamped point_type;
	typedef geometry_msgs::PoseStamped pose_type;
	typedef std::array<float, 7> q_type;

public:
	FileWriter(std::string const &filename)
		: ofs_(filename)
	{
		ofs_ << "ball_x,ball_y,ball_z,"
				<< "kalman_x,kalman_y,kalman_z,"
				<< "cam_desired_pos_x,cam_desired_pos_y,cam_desired_pos_z,"
				<< "cam_desired_orient_x,cam_desired_orient_y,cam_desired_orient_z,cam_desired_orient_w,"
				<< "cam_real_pos_x,cam_real_pos_y,cam_real_pos_z,"
				<< "cam_real_orient_x,cam_real_orient_y,cam_real_orient_z,cam_real_orient_w,"
				<< "q_real_0,q_real_1,q_real_2,q_real_3,q_real_4,q_real_5,q_real_6,"
				<< "q_desired_0,q_desired_1,q_desired_2,q_desired_3,q_desired_4,q_desired_5,q_desired_6\n";
	}

	~FileWriter()
	{
		ofs_.close();
	}

	void write(point_type const &ball, point_type const &kalman,
		pose_type const &camera_desired, pose_type const &camera_real,
		q_type const &q_real, q_type const &q_desired)
	{
		ofs_ << ball.point.x << "," << ball.point.y << "," << ball.point.z << ","
				<< kalman.point.x << "," << kalman.point.y << "," << kalman.point.z << ","
				<< camera_desired.pose.position.x << "," << camera_desired.pose.position.y << "," << camera_desired.pose.position.z << ","
				<< camera_desired.pose.orientation.x << "," << camera_desired.pose.orientation.y << "," << camera_desired.pose.orientation.z << ","<< camera_desired.pose.orientation.w << ","
				<< camera_real.pose.position.x << "," << camera_real.pose.position.y << "," << camera_real.pose.position.z << ","
				<< camera_real.pose.orientation.x << "," << camera_real.pose.orientation.y << "," << camera_real.pose.orientation.z << "," << camera_real.pose.orientation.w << ",";

		for (int i = 0; i < 7; ++i) {
			ofs_ << q_real[i] << ",";
		}

		for (int i = 0; i < 7; ++i) {
			ofs_ << q_desired[i];

			if (i == 6) {
				 ofs_ << "\n"; // Last column
			} else {
				ofs_ << ",";
			}
		}
	}

private:
	std::ofstream ofs_;
};

/**
 * Callback for the ball's position
 * @param point
 */
void callbackBall(const geometry_msgs::PointStamped &ps)
{
	std::lock_guard<std::mutex> lock(g_mutex_ball_points);
	g_ball_points.push_front(ps);

	if (g_ball_points.size() > BUFFERS_SIZE) {
		g_ball_points.pop_back();
	}
}

/**
 * Callback for the kalman's point position
 * @param point
 */
void callbackKalman(const geometry_msgs::PointStamped &ps)
{
	std::lock_guard<std::mutex> lock(g_mutex_kalman_points);
	g_kalman_points.push_front(ps);

	if (g_kalman_points.size() > BUFFERS_SIZE) {
		g_kalman_points.pop_back();
	}
}

/**
 * Callback for the Camera Pose Desired
 * @param pose
 */
void callbackCameraPoseDesired(const geometry_msgs::PoseStamped &ps)
{
	std::lock_guard<std::mutex> lock(g_mutex_camera_desired_poses);
	g_camera_desired_poses.push_front(ps);

	if (g_camera_desired_poses.size() > BUFFERS_SIZE) {
		g_camera_desired_poses.pop_back();
	}
}

/**
 * Callback for the Camera Pose Real
 * @param pose
 */
void callbackCameraPoseReal(const geometry_msgs::PoseStamped &ps)
{
	std::lock_guard<std::mutex> lock(g_mutex_camera_real_poses);
	g_camera_real_poses.push_front(ps);

	if (g_camera_real_poses.size() > BUFFERS_SIZE) {
		g_camera_real_poses.pop_back();
	}
}

/**
 * Callback for the Q desired
 * @param q
 */
void callbackQdesired(const path_planning::Q_desired &q)
{

	std::array<float, 7> q_desired;

	for (unsigned char joint = 0; joint < 7; ++joint) {
		q_desired[joint] = q.positions[joint];
	}

	std::lock_guard<std::mutex> lock(g_mutex_desired_qs);
	g_desired_qs.push_front(q_desired);

	if (g_desired_qs.size() > BUFFERS_SIZE) {
		g_desired_qs.pop_back();
	}
}

/**
 * Callback for the Q real
 * @param q
 */
void callbackQreal(const path_planning::Q_real &q)
{
	std::array<float, 7> q_real;

	for (unsigned char joint = 0; joint < 7; ++joint) {
		q_real[joint] = q.positions[joint];
	}

	std::lock_guard<std::mutex> lock(g_mutex_real_qs);
	g_real_qs.push_front(q_real);

	if (g_real_qs.size() > BUFFERS_SIZE) {
		g_real_qs.pop_back();
	}
}

/**
 *
 * @param argc
 * @param argv
 * @return
 */
int main(int argc, char **argv)
{
	ros::init(argc, argv, "log_node");

	//ROS
	ros::NodeHandle nh;

	//Subscribers
	ros::Subscriber sub_ball = nh.subscribe(SUB_BALL, 1, callbackBall);
	ros::Subscriber sub_kalman = nh.subscribe(SUB_KALMAN, 1, callbackKalman);
	ros::Subscriber sub_camera_pose_desired = nh.subscribe(SUB_CAMERA_POS_DESIRED, 1, callbackCameraPoseDesired);
	ros::Subscriber sub_camera_pose_real = nh.subscribe(SUB_CAMERA_POS_REAL, 1, callbackCameraPoseReal);
	ros::Subscriber sub_q_desired = nh.subscribe(SUB_Q_DESIRED, 1, callbackQdesired);
	ros::Subscriber sub_q_real = nh.subscribe(SUB_Q_REAL, 1, callbackQreal);

	//File
	FileWriter fw("log.csv");

	//GNUPlot
	Gnuplot gp;

	//Debugging parameter
	bool debugging = true;
	int frame_rate = 1;
	bool write_file = true;
	bool feed_gnuplot = true;

	while (nh.ok()) {
		//Debugging
		ros::param::get(PARAM_DEBUGGING, debugging);

		//Frame Rate
		ros::param::get(PARAM_FRAME_RATE, frame_rate);
		ros::Rate loop_rate(frame_rate);

		std::lock_guard<std::mutex> lock_ball(g_mutex_ball_points);
		std::lock_guard<std::mutex> lock_kalman(g_mutex_kalman_points);
		std::lock_guard<std::mutex> lock_cam_desired(g_mutex_camera_desired_poses);
		std::lock_guard<std::mutex> lock_cam_real(g_mutex_camera_real_poses);
		std::lock_guard<std::mutex> lock_real_qs(g_mutex_real_qs);
		std::lock_guard<std::mutex> locl_desired_qs(g_mutex_desired_qs);

		if (!g_ball_points.empty()
				&& g_kalman_points.size() >= 2
				&& !g_camera_desired_poses.empty()
				&& !g_camera_real_poses.empty()
				&& !g_real_qs.empty()
				&& !g_desired_qs.empty()) {
			geometry_msgs::PointStamped ball = g_ball_points.front();
			geometry_msgs::PointStamped kalman = g_kalman_points.at(1);
			geometry_msgs::PoseStamped cam_desired = g_camera_desired_poses.front();
			geometry_msgs::PoseStamped cam_real = g_camera_real_poses.front();
			std::array<float, 7> q_real = g_real_qs.front();
			std::array<float, 7> q_desired = g_desired_qs.front();

			if (write_file) {
				fw.write(ball, kalman, cam_desired, cam_real, q_real, q_desired);
			}

			if (feed_gnuplot) {
				gp << "splot ";
				gp << gp.binFile1d(g_ball_points, "record") << "with lines title 'test'";
				gp << "\n";
			}
		}

		// Mutexes must be released before ROS callbacks are processed.
		locl_desired_qs.~lock_guard();
		lock_real_qs.~lock_guard();
		lock_cam_real.~lock_guard();
		lock_cam_desired.~lock_guard();
		lock_kalman.~lock_guard();
		lock_ball.~lock_guard();

		//And sleep
		ros::AsyncSpinner(0).start();
		loop_rate.sleep();
	}

	return 0;
}
