// Copyright (c) 2023 Franka Robotics GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_example_controllers/cartesian_pose_example_controller.h>

#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

#include <controller_interface/controller_base.h>
#include <franka_hw/franka_cartesian_command_interface.h>
#include <hardware_interface/hardware_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/GripperCommandAction.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
//#include <std_msgs/Float64MultiArray.h>  // For target position message

namespace franka_example_controllers {

  using GripperClient = actionlib::SimpleActionClient<control_msgs::GripperCommandAction>;
  std::unique_ptr<GripperClient> gripper_client_;

bool CartesianPoseExampleController::init(hardware_interface::RobotHW* robot_hardware,
                                          ros::NodeHandle& node_handle) {
  cartesian_pose_interface_ = robot_hardware->get<franka_hw::FrankaPoseCartesianInterface>();
  if (cartesian_pose_interface_ == nullptr) {
    ROS_ERROR(
        "CartesianPoseExampleController: Could not get Cartesian Pose "
        "interface from hardware");
    return false;
  }

  std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR("CartesianPoseExampleController: Could not get parameter arm_id");
    return false;
  }

  try {
    cartesian_pose_handle_ = std::make_unique<franka_hw::FrankaCartesianPoseHandle>(
        cartesian_pose_interface_->getHandle(arm_id + "_robot"));
  } catch (const hardware_interface::HardwareInterfaceException& e) {
    ROS_ERROR_STREAM(
        "CartesianPoseExampleController: Exception getting Cartesian handle: " << e.what());
    return false;
  }

  auto state_interface = robot_hardware->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR("CartesianPoseExampleController: Could not get state interface from hardware");
    return false;
  }

  try {
    auto state_handle = state_interface->getHandle(arm_id + "_robot");

    std::array<double, 7> q_start{{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
    for (size_t i = 0; i < q_start.size(); i++) {
      if (std::abs(state_handle.getRobotState().q_d[i] - q_start[i]) > 0.1) {
        ROS_ERROR_STREAM(
            "CartesianPoseExampleController: Robot is not in the expected starting position for "
            "running this example. Run `roslaunch franka_example_controllers move_to_start.launch "
            "robot_ip:=<robot-ip> load_gripper:=<has-attached-gripper>` first.");
        return false;
      }
    }
  } catch (const hardware_interface::HardwareInterfaceException& e) {
    ROS_ERROR_STREAM(
        "CartesianPoseExampleController: Exception getting state handle: " << e.what());
    return false;
  }

  return true;
}

enum TaskState{
  FINDINGTARGET,
  FINDINGTARGET_2_MOVE,
  MOVE,
  DROP
};
TaskState curr_taskState;

Eigen::Matrix3d R_start; // initial rotation matrix
Eigen::Matrix3d R_target;
Eigen::Matrix3d Rx_180;
Eigen::Matrix3d Ry_180;


Eigen::Matrix4d Base_T_End,End_T_Camera,Camera_T_Box,Base_T_Box,Base_T_Camera;
double targetPosition[3]; // x y z
double move_start_time;

ros::NodeHandle nh_;
//boost::shared_ptr<const std_msgs::Float64MultiArray> msg;
ros::Subscriber box_pose_sub_;
bool box_pose_receive_ = false;
std::array<double, 16> cur_pose;

void CartesianPoseExampleController::starting(const ros::Time& /* time */) {
  initial_pose_ = cartesian_pose_handle_->getRobotState().O_T_EE_d;
  // intial_pose x = 0.306324 y = -0.001665 z = 0.420514
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
    R_start(i, j) = initial_pose_[i + j*4];  // column-major

  elapsed_time_ = ros::Duration(0.0);
  curr_taskState = FINDINGTARGET;

  // End_T_Camera << -0.707, -0.707, 0, 0.0175,
  //                  0.707, -0.707, 0, -0.11258,
  //                      0,      0, 1,  0.05436,
  //                      0,      0, 0,        1;

  End_T_Camera(0,0) = -0.707;
  End_T_Camera(1,0) = 0.707;
  End_T_Camera(2,0) = 0;
  End_T_Camera(3,0) = 0;

  End_T_Camera(0,1) = -0.707;
  End_T_Camera(1,1) = -0.707;
  End_T_Camera(2,1) = 0;
  End_T_Camera(3,1) = 0;

  End_T_Camera(0,2) = 0;
  End_T_Camera(1,2) = 0;
  End_T_Camera(2,2) = 1;
  End_T_Camera(3,2) = 0;

  End_T_Camera(0,3) = -0.0672216;
  End_T_Camera(1,3) = -0.0919666;
  End_T_Camera(2,3) = 0.05436;
  End_T_Camera(3,3) = 1;
  
  for (int i = 0; i < 4; ++i)
    for (int j = 0; j < 4; ++j)
      Base_T_End(i,j) = initial_pose_[i + j*4];  // column-major

  nh_ = ros::NodeHandle("~");
  box_pose_sub_ = nh_.subscribe("/Box_pose",1,&CartesianPoseExampleController::matrixCallback,this);
  // gripper_client_ = std::make_unique<GripperClient>("/gripper_action",true);
  // ROS_INFO("Waiting for gripper action server...");
  // grip••••••••••••per_client_->waitForServer();
  // ROS_INFO("Gripper action server connected");

  // control_msgs::GripperCommandGoal goal;
  // goal.command.position = 0.0;  // Close the gripper
  // goal.command.max_effort = 10.0;
  // gripper_client_->sendGoal(goal);
  // gripper_client_->waitForResult(ros::Duration(5.0));
}

void CartesianPoseExampleController::matrixCallback(const std_msgs::Float64MultiArray::ConstPtr& msg){

  if(msg->data.size() != 16) return;

  for (int i = 0; i < 4; ++i)
    for (int j = 0; j < 4; ++j)
      Camera_T_Box(i,j) = msg->data[j + i*4];  // row-major
  
  printf("[%.4f %.4f %.4f %.4f\n %.4f %.4f %.4f %.4f\n %.4f %.4f %.4f %.4f]\n",
    Camera_T_Box(0,0),Camera_T_Box(0,1),Camera_T_Box(0,2),Camera_T_Box(0,3),
    Camera_T_Box(1,0),Camera_T_Box(1,1),Camera_T_Box(1,2),Camera_T_Box(1,3),
    Camera_T_Box(2,0),Camera_T_Box(2,1),Camera_T_Box(2,2),Camera_T_Box(2,3));
  box_pose_receive_ = true;
  ROS_INFO("Received target matrix");
}

void CartesianPoseExampleController::update(const ros::Time& /* time */,
                                            const ros::Duration& period) {

  ros::Time current_time = ros::Time::now();
  elapsed_time_ += period;
  std::array<double, 16> new_pose =initial_pose_;

  switch (curr_taskState)
  {
    case FINDINGTARGET:
      ROS_INFO("Waiting for 4x4 matrix ......");
      if(box_pose_receive_ == true)
        curr_taskState = FINDINGTARGET_2_MOVE;
      break;
  
    case FINDINGTARGET_2_MOVE:
      Rx_180 = Eigen::AngleAxis<double>(M_PI, Eigen::Vector3d::UnitX()).toRotationMatrix();
      Ry_180 = Eigen::AngleAxis<double>(M_PI, Eigen::Vector3d::UnitY()).toRotationMatrix();
      Base_T_Box = Base_T_End * End_T_Camera * Camera_T_Box;

      Base_T_Camera = Base_T_End * End_T_Camera;

      R_target = Base_T_Box.block<3,3>(0,0);
      R_target = Rx_180 * R_target;
      //R_target = Ry_180 * R_target;

      targetPosition[0] = Base_T_Box(0,3); // x
      targetPosition[1] = Base_T_Box(1,3);   // y
      //targetPosition[2] = Base_T_Box(2,3) - 0.11; // z
      targetPosition[2] = Base_T_Box(2,3);

      move_start_time = elapsed_time_.toSec();


      cur_pose = cartesian_pose_handle_->getRobotState().O_T_EE;
      printf("box_x = %.4f box_y = %.4f box_z = %.4f\n",targetPosition[0] , targetPosition[1], targetPosition[2]);
      printf("cur_x = %.4f cur_y = %.4f cur_z = %.4f\n",cur_pose[12] , cur_pose[13], cur_pose[14]);
      printf("Base to Box matrix\n[%.4f %.4f %.4f %.4f\n %.4f %.4f %.4f %.4f\n %.4f %.4f %.4f %.4f]\n",
        Base_T_Box(0,0),Base_T_Box(0,1),Base_T_Box(0,2),Base_T_Box(0,3),
        Base_T_Box(1,0),Base_T_Box(1,1),Base_T_Box(1,2),Base_T_Box(1,3),
        Base_T_Box(2,0),Base_T_Box(2,1),Base_T_Box(2,2),Base_T_Box(2,3));
      printf("Base to Camera matrix\n[%.4f %.4f %.4f %.4f\n %.4f %.4f %.4f %.4f\n %.4f %.4f %.4f %.4f]\n",
        Base_T_Camera(0,0),Base_T_Camera(0,1),Base_T_Camera(0,2),Base_T_Camera(0,3),
        Base_T_Camera(1,0),Base_T_Camera(1,1),Base_T_Camera(1,2),Base_T_Camera(1,3),
        Base_T_Camera(2,0),Base_T_Camera(2,1),Base_T_Camera(2,2),Base_T_Camera(2,3));
      printf("Current End-effector Rotation matrix\n[%.4f %.4f %.4f %.4f\n %.4f %.4f %.4f %.4f\n %.4f %.4f %.4f %.4f\n %.4f %.4f %.4f %.4f]\n",
        cur_pose[0],cur_pose[4],cur_pose[8],cur_pose[12],
        cur_pose[1],cur_pose[5],cur_pose[9],cur_pose[13],
        cur_pose[2],cur_pose[6],cur_pose[10],cur_pose[14],
        cur_pose[3],cur_pose[7],cur_pose[11],cur_pose[15]);
      printf("Camera to Box matrix\n[%.4f %.4f %.4f %.4f\n %.4f %.4f %.4f %.4f\n %.4f %.4f %.4f %.4f\n %.4f %.4f %.4f %.4f]\n",
        Camera_T_Box(0,0),Camera_T_Box(0,1),Camera_T_Box(0,2),Camera_T_Box(0,3),
        Camera_T_Box(1,0),Camera_T_Box(1,1),Camera_T_Box(1,2),Camera_T_Box(1,3),
        Camera_T_Box(2,0),Camera_T_Box(2,1),Camera_T_Box(2,2),Camera_T_Box(2,3),
        Camera_T_Box(3,0),Camera_T_Box(3,1),Camera_T_Box(3,2),Camera_T_Box(3,3));
      curr_taskState = MOVE;
      break;

    case MOVE:
      std::array<double, 16> cur_pose = cartesian_pose_handle_->getRobotState().O_T_EE;

      double pstart[3] = {new_pose[12],new_pose[13],new_pose[14]};

      double T = 30.0;  // total motion time
      double t = elapsed_time_.toSec();
      t -= move_start_time;
      if (t > T) t = T;

      double s = 0.5 * (1 - std::cos(M_PI * t / T));

      Eigen::Quaterniond q_start(R_start);
      Eigen::Quaterniond q_target(R_target);
      Eigen::Quaterniond q_interp = q_start.slerp(s, q_target);
      Eigen::Matrix3d R_interp = q_interp.toRotationMatrix();
      for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
          new_pose[i + j*4] = R_interp(i, j);  // column-major

      new_pose[12] = (1 - s) * pstart[0] + s * targetPosition[0];  // X
      new_pose[13] = (1 - s) * pstart[1] + s * targetPosition[1];  // Y
      new_pose[14] = (1 - s) * pstart[2] + s * targetPosition[2];  // Z

      // Debug info
      printf("[%d] t = %.2f \n tar_x = %.4f tar_y = %.4f tar_z = %.4f\n",current_time.sec, t, new_pose[12] , new_pose[13], new_pose[14] );
      printf("cur_x = %.4f cur_y = %.4f cur_z = %.4f\n",cur_pose[12],cur_pose[13],cur_pose[14]);
      printf("[%.4f %.4f %.4f\n %.4f %.4f %.4f\n %.4f %.4f %.4f]\n",
          new_pose[0],new_pose[4],new_pose[8],
          new_pose[1],new_pose[5],new_pose[9],
          new_pose[2],new_pose[6],new_pose[10]);
      break;
  }

  cartesian_pose_handle_->setCommand(new_pose);  
}

}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::CartesianPoseExampleController,
                       controller_interface::ControllerBase)
