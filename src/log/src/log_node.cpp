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
#include <vector>
#include <mutex>

// This must be defined before the first time that "gnuplot-iostream.h" is included.
// #define GNUPLOT_ENABLE_PTY
#define GNUPLOT_ENABLE_CXX11
#include "gnuplot-iostream.h"

using namespace std;

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

geometry_msgs::PointStamped ball_point;

std::vector<geometry_msgs::PointStamped> g_vec_ball_points;
std::mutex g_mutex_ball_points;

geometry_msgs::PointStamped kalman_point;
geometry_msgs::PoseStamped camera_pose;
geometry_msgs::PoseStamped camera_pose_real;
float Q_real[7];
float Q_desired[7];

// Writes CSV file
class FileWriter
{
public:
    typedef geometry_msgs::PointStamped point_type_;

    FileWriter(std::string const &filename)
        : ofs_(filename)
    {
        ofs_ << "ball_x, ball_y, ball_z" << std::endl;
    }

    ~FileWriter()
    {
        ofs_.close();
    }

    void write(point_type_ ball)
    {
        ofs_ << ball.point.x << ", " << ball.point.y << ", " << ball.point.z << std::endl;
    }

private:
    std::ofstream ofs_;
};

// Tells gnuplot-iostream how to print objects of class geometry_msgs::PointStamped.
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

/**
 * Callback for the ball's position
 * @param point
 */
void callbackBall(const geometry_msgs::PointStamped & point){
	ball_point.header.frame_id = point.header.frame_id;
	ball_point.header.seq = point.header.seq;
	ball_point.header.stamp = point.header.stamp;
	ball_point.point.x = point.point.x;
	ball_point.point.y = point.point.y;
	ball_point.point.z = point.point.z;

    std::lock_guard<std::mutex> lock(g_mutex_ball_points);
    g_vec_ball_points.push_back(ball_point);

    if (g_vec_ball_points.size() > 10) {
        g_vec_ball_points.pop_back();
    }
}

/**
 * Callback for the kalman's point position
 * @param point
 */
void callbackKalman(const geometry_msgs::PointStamped & point){
	kalman_point.header.frame_id = point.header.frame_id;
	kalman_point.header.seq = point.header.seq;
	kalman_point.header.stamp = point.header.stamp;
	kalman_point.point.x = point.point.x;
	kalman_point.point.y = point.point.y;
	kalman_point.point.z = point.point.z;
}

/**
 * Callback for the Camera Pose Desired
 * @param pose
 */
void callbackCameraPoseDesired(const geometry_msgs::PoseStamped & pose){
	camera_pose.header.frame_id = pose.header.frame_id;
	camera_pose.header.seq = pose.header.seq;
	camera_pose.header.stamp = pose.header.stamp;
	camera_pose.pose.orientation.x = pose.pose.orientation.x;
	camera_pose.pose.orientation.y = pose.pose.orientation.y;
	camera_pose.pose.orientation.z = pose.pose.orientation.z;
	camera_pose.pose.orientation.w = pose.pose.orientation.w;
	camera_pose.pose.position.x = pose.pose.position.x;
	camera_pose.pose.position.y = pose.pose.position.y;
	camera_pose.pose.position.z = pose.pose.position.z;
}

/**
* Callback for the Camera Pose Real
 * @param pose
 */
void callbackCameraPoseReal(const geometry_msgs::PoseStamped & pose){
	camera_pose_real.header.frame_id = pose.header.frame_id;
	camera_pose_real.header.seq = pose.header.seq;
	camera_pose_real.header.stamp = pose.header.stamp;
	camera_pose_real.pose.orientation.x = pose.pose.orientation.x;
	camera_pose_real.pose.orientation.y = pose.pose.orientation.y;
	camera_pose_real.pose.orientation.z = pose.pose.orientation.z;
	camera_pose_real.pose.orientation.w = pose.pose.orientation.w;
	camera_pose_real.pose.position.x = pose.pose.position.x;
	camera_pose_real.pose.position.y = pose.pose.position.y;
	camera_pose_real.pose.position.z = pose.pose.position.z;
}

/**
 * Callback for the Q desired
 * @param q
 */
void callbackQdesired(const path_planning::Q_desired & q){
	for (unsigned char joint = 0; joint < 7; ++joint)
		Q_desired[joint] = q.positions[joint];
}

/**
 * Callback for the Q real
 * @param q
 */
void callbackQreal(const path_planning::Q_real & q){
	for (unsigned char joint = 0; joint < 7; ++joint)
		Q_real[joint] = q.positions[joint];
}


/**
 *
 * @param argc
 * @param argv
 * @return
 */
int main(int argc, char **argv)
{
	ROS_INFO("Log Started!");
    ros::init(argc, argv, "log_node");

	//ROS
	ros::NodeHandle nh;

	//Subscribers
	ros::Subscriber sub_ball = nh.subscribe(SUB_BALL, 1, callbackBall);
	ros::Subscriber sub_kalman = nh.subscribe(SUB_BALL, 1, callbackKalman);
	ros::Subscriber sub_camera_pose_desired = nh.subscribe(SUB_BALL, 1, callbackCameraPoseDesired);
	ros::Subscriber sub_camera_pose_real = nh.subscribe(SUB_BALL, 1, callbackCameraPoseReal);
	ros::Subscriber sub_q_desired = nh.subscribe(SUB_Q_DESIRED, 1, callbackQdesired);
	ros::Subscriber sub_q_real = nh.subscribe(SUB_Q_REAL, 1, callbackQreal);

	//Debugging parameter
	bool debugging = true;
	nh.setParam(PARAM_DEBUGGING, true);

	//Point to store the prediction
	geometry_msgs::PointStamped point_file;

	//File
    FileWriter fw("log.csv");

    //GNUPlot
    Gnuplot gp;

	//WallTime
    //	ros::WallTime walltime;
	int frame_rate = 1;

    while(nh.ok()){
		//Debugging
		ros::param::get(PARAM_DEBUGGING, debugging);
		//Frame Rate
		ros::param::get(PARAM_FRAME_RATE, frame_rate);
		ros::Rate loop_rate(frame_rate);

		//TODO
        std::lock_guard<std::mutex> lock(g_mutex_ball_points);
        gp << "plot ";
        gp << gp.binFile1d(g_vec_ball_points, "record") << "with lines title 'test'";
        gp << "\n";
        lock.~lock_guard();

		//And sleep
		loop_rate.sleep();
		ros::spinOnce();
	}
	return 0;
}
