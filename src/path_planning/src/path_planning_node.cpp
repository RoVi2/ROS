/*
 * Path Planning Node
 *
 *  Created on: Mar 28, 2015
 *      Author: Jorge Rodriguez Marin
 */

// STD
#include <iostream>

// RobWork
#include <rw/rw.hpp>
#include <rwlibs/pathplanners/rrt/RRTPlanner.hpp>
#include <rwlibs/pathplanners/rrt/RRTQToQPlanner.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>

// ROS
#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>

// Services
#include <path_planning/getJointConfig.h>
#include <path_planning/setJointConfig.h>
#include <path_planning/addToQueue.h>
#include <path_planning/addToQueue.h>

using namespace std;
using namespace rw::common;
using namespace rw::kinematics;
using namespace rw::loaders;
using namespace rw::models;
using namespace rw::pathplanning;
using namespace rw::proximity;
using namespace rw::trajectory;
using namespace rwlibs::pathplanners;
using namespace rwlibs::proximitystrategies;

// RobWork Paths
#define WORKCELL_PATH "../../res/PA10Scene/sceneComplete.wc.xml"
#define ROBOT_NAME "PA10"

// ROS Paths
//#define SUBSCRIBER "/points_server/points"
#define SUBSCRIBER "/kalman_filter/points"
#define PARAM_DEBUGGING "/path_planning/debugging"
#define PARAM_FRAME_RATE "/frame_rate"
#define PARAM_PATH_NOT_FOUND_TIME_LIMIT \
  "/path_planning/path_not_found_time_limit"
#define PARAM_SPLINE_PATH_DT "/path_planning/spline_path_dt"  //
#define PARAM_STEP_DT \
  "/path_planning/step_dt"  // For each time steps, how many times we split the
                            // path in

// Colors
#define RESET "\e[m"
#define GREEN "\e[32m"
#define YELLOW "\e[33m"
#define MAGENTA "\e[35m"
#define CYAN "\e[36m"

// Global variables
Vector3D<double> point_kalman;

/**
 * Callback method to get the points from ROS
 */
void callBack(const geometry_msgs::PointStampedConstPtr &point_ros) {
  point_kalman[0] = point_ros->point.x;
  point_kalman[1] = point_ros->point.y;
  point_kalman[2] = point_ros->point.z;
}

/**
 * Sends the desired Q to the real robot
 * @param Q_desired
 * @param client_setJointConfig
 * @param srv_setJointConfig
 * @param debugging
 */
void sendRobotToQ(rw::math::Q &Q_desired,
                  ros::ServiceClient &client_setJointConfig,
                  path_planning::setJointConfig &srv_setJointConfig,
                  bool &debugging) {
  // Copy the joint values in to the service message
  for (unsigned char joint = 0; joint < 7; joint++) {
    srv_setJointConfig.request.commands[joint] = 1;
    srv_setJointConfig.request.positions[joint] = Q_desired[joint] * Rad2Deg;
  }
  // Publish it!
  if (client_setJointConfig.call(srv_setJointConfig)) {
    if (debugging) cout << "	" << Q_desired << endl;

    /*
     * Maybe problems with sending too many Q? This is your place!
     */
    Timer waitUntilRobot;
    waitUntilRobot.resetAndResume();
    while (waitUntilRobot.getTimeMs() <= 100)
      ;  // Do nothing
  } else if (debugging)
    ROS_ERROR("Robot not found");
}

/**
 *
 * @param argc
 * @param argv
 * @return
 */
int main(int argc, char **argv) {
  /*
   * ROS Stuff
   */
  ROS_INFO("Path Planning Started!");
  ros::init(argc, argv, "path_planning_node");
  ros::NodeHandle nh;
  ros::Subscriber point_sub;

  // Parameters
  bool debugging = true;
  int frame_rate;
  double spline_path_dt = 3;  // DONT PUT 1 OR LESS
  double step_dt = 1;

  nh.setParam(PARAM_DEBUGGING, true);
  nh.setParam(PARAM_SPLINE_PATH_DT, spline_path_dt);
  nh.setParam(PARAM_STEP_DT, step_dt);
  nh.getParam(PARAM_FRAME_RATE, frame_rate);  // Obtained from the server

  // Get the new predicted point and store it
  point_sub = nh.subscribe(SUBSCRIBER, 1, callBack);

  // Services
  ros::ServiceClient client_getJointConfig =
      nh.serviceClient<path_planning::getJointConfig>("pa10/getJointConfig");
  ros::ServiceClient client_setJointConfig =
      nh.serviceClient<path_planning::setJointConfig>("pa10/setJointsConfig");
  path_planning::getJointConfig srv_getJointConfig;
  path_planning::setJointConfig srv_setJointConfig;

  /*
   * RobWork
   */
  // Load the workcell and the device
  WorkCell::Ptr wc = WorkCellLoader::Factory::load(WORKCELL_PATH);
  Device::Ptr device = wc->findDevice(ROBOT_NAME);

  // Generate randomness for the RRT
  Math::seed();

  // Obtain the current state and position of the real robot
  State state = wc->getDefaultState();
  Q Q_home = Q(7, 0, -0.65, 0, 1.8, 0, 0.42, 0);
  Q Q_current(7, 0, 0, 0, 0, 0, 0, 0);
  Timer pathNotFoundTimer;
  int pathNotFoundTimeLimit = 10;

  CollisionDetector detector(
      wc, ProximityStrategyFactory::makeDefaultCollisionStrategy());
  PlannerConstraint constraint =
      PlannerConstraint::make(&detector, device, state);
  QSampler::Ptr sampler = QSampler::makeConstrained(
      QSampler::makeUniform(device), constraint.getQConstraintPtr());
  QMetric::Ptr metric = MetricFactory::makeEuclidean<Q>();
  double extend = 0.1;  // Distance to search the next point
  QToQPlanner::Ptr planner = RRTPlanner::makeQToQPlanner(
      constraint, sampler, metric, extend, RRTPlanner::RRTConnect);

  // Going home
  sendRobotToQ(Q_home, client_setJointConfig, srv_setJointConfig, debugging);
  device->setQ(Q_home, state);

  /*
   * Loop!
   */
  while (ros::ok()) {
    // Check if debugging
    nh.getParam(PARAM_DEBUGGING, debugging);
    // Refresh frequency
    nh.getParam(PARAM_FRAME_RATE, frame_rate);
    ros::Rate loop_rate(frame_rate);
    // Get the path not found time limit
    nh.getParam(PARAM_PATH_NOT_FOUND_TIME_LIMIT, pathNotFoundTimeLimit);
    // And the interpolation options
    nh.getParam(PARAM_SPLINE_PATH_DT, spline_path_dt);
    nh.getParam(PARAM_STEP_DT, step_dt);

    // Generate randomness
    Math::seed();

    // Obtain from the kalman point, the point where we want to put the camera.
    // This point for now will be that which is at 0.5.m from the object and is
    // contained
    //	in (1) a plane that with the kalman point is parallel to the ground and
    //(2) also
    //	in the plane generated by the robot's base axis and the kalman point.

    /*
     * Obtains the Desired Q
     */
    // Translate the kalman point from the camera to the robot's base
    if (debugging)
      cout << endl
           << GREEN "New Point from: " RESET << endl;
    if (debugging) cout << GREEN "	Camera: " RESET << point_kalman << endl;
    point_kalman =
        (device->baseTframe(wc->findFrame("Camera"), state)) * point_kalman;
    if (debugging) cout << GREEN "	Base: " RESET << point_kalman << endl;
    // Calculate in polar coordinates so it will simplify the calculations
    // P(Radius, angle, height)
    double radius = sqrt(pow(point_kalman[0], 2) + pow(point_kalman[1], 2));
    double angle = atan2(point_kalman[1], point_kalman[0]);
    double height = point_kalman[2];
    Vector3D<double> point_kalman_polar(radius, angle, height);
    if (debugging)
      cout << GREEN "	and in Polar: " RESET << point_kalman_polar << endl;
    // The camera position will be 0.5 meters from the kalman point in the
    // radial direction
    Vector3D<double> camera_position_polar =
        point_kalman_polar - Vector3D<double>(0.3, 0, 0);
    Vector3D<double> camera_position(
        camera_position_polar[0] *
            cos(camera_position_polar[1]),  // r*cos(angle)
        camera_position_polar[0] *
            sin(camera_position_polar[1]),  // r*sin(angle)
        camera_position_polar[2]            // height
        );
    // The rotation has to be that which is oriented to the point.
    RPY<double> cameraRotation = RPY<>(-1.571 + angle, 0, -1.571);
    // So the transformation matrix is created with the translation and the
    // rotation matrices
    Transform3D<double> cameraTransfomationMatrix =
        Transform3D<double>(camera_position, cameraRotation.toRotation3D());
    if (debugging)
      cout << GREEN "	Position for Camera: " RESET
           << cameraTransfomationMatrix.P() << endl;
    if (debugging)
      cout << GREEN "	Rotation for Camera: " RESET
           << RPY<double>(cameraTransfomationMatrix.R()) << endl;

    // Now we can calculate the Q with inverse kinematics
    rw::invkin::JacobianIKSolver solver(device, state);
    solver.setCheckJointLimits(1);  // Activate the joint limits checking
    vector<Q> qVectorSolutionIK;  // It returns a vector with all the solutions
    qVectorSolutionIK = solver.solve(cameraTransfomationMatrix, state);

    /*
     * Obtains the Path. RRT + Natural Cubic Spline
     */
    // QPath to store the path
    QPath pathToDesiredQ;
    // We need an smart pointer for the Interpolation. I don't know why
    QPath::Ptr pathToDesiredQPtr;
    pathToDesiredQPtr = &pathToDesiredQ;

    // If there is a target Q
    if (!qVectorSolutionIK.empty()) {
      // Clear the vector of q's
      pathToDesiredQ.clear();
      // Get current position of the real robot
      client_getJointConfig.call(srv_getJointConfig);
      for (unsigned char joint = 0; joint < device->getDOF(); joint++)
        Q_current[joint] =
            srv_getJointConfig.response.positions[joint] * Deg2Rad;
      device->setQ(Q_current, state);

      // Calculate the path from the current Q to the desired Q and show the
      // calculation time
      if (debugging)
        cout << YELLOW "    Planning from " << Q_current << " to "
             << qVectorSolutionIK[0] << ":" RESET << endl;
      planner->query(Q_current, qVectorSolutionIK[0], pathToDesiredQ);
      if (debugging)
        cout << CYAN "	Solution found with " << qVectorSolutionIK.size()
             << " Q's" << endl;

      // For the solution, we interpolate all the Q to get an smoother path
      if (pathToDesiredQ.size() >= 2) {
        InterpolatorTrajectory<Q>::Ptr pathInterpolated =
            CubicSplineFactory::makeNaturalSpline(pathToDesiredQPtr,
                                                  spline_path_dt);
        vector<Q> qVector = pathInterpolated->getPath((double)(step_dt), false);
        for (auto q : qVector) {
          sendRobotToQ(q, client_setJointConfig, srv_setJointConfig, debugging);
        }
        pathNotFoundTimer.resetAndResume();
      } else {
        if (debugging)
          cout << MAGENTA
              "Not solution found. Reset in: " << pathNotFoundTimer.getTimeSec()
               << "/" << pathNotFoundTimeLimit << RESET << endl
               << endl;
        // If we don't find a solution over the time limit, we go back to home
        if (pathNotFoundTimer.getTimeSec() >= pathNotFoundTimeLimit) {
          sendRobotToQ(Q_home, client_setJointConfig, srv_setJointConfig,
                       debugging);
          pathNotFoundTimer.reset();
          if (debugging) ROS_WARN("Robot going home");
        }
      }
    } else {
      if (debugging)
        cout << MAGENTA
            "Not solution found. Reset in: " << pathNotFoundTimer.getTimeSec()
             << "/" << pathNotFoundTimeLimit << RESET << endl
             << endl;
      // If we don't find a solution over the time limit, we go back to home
      if (pathNotFoundTimer.getTimeSec() >= pathNotFoundTimeLimit) {
        sendRobotToQ(Q_home, client_setJointConfig, srv_setJointConfig,
                     debugging);
        device->setQ(Q_home, state);
        pathNotFoundTimer.reset();
        if (debugging) ROS_WARN("Robot going home");
      }
    }

    loop_rate.sleep();
    ros::spinOnce();
  }
}
