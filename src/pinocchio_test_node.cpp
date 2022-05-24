
#include <pinocchio/parsers/urdf.hpp>

#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
//

#include "pinocchio/parsers/sample-models.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/rnea.hpp"

#include <iostream>

#ifndef PINOCCHIO_MODEL_DIR
  #define PINOCCHIO_MODEL_DIR "path_to_the_model_dir"
#endif

//#include "ros/ros.h"
//#include "std_msgs/String.h"

int main(int argc, char **argv)
{
//using namespace pinocchio;
////ros::init(argc, argv, "pinocchio_test_node");
////ros::NodeHandle nh;
//
////ros::Publisher chatter_pub = nh.advertise<std_msgs::String>("chatter", 1000);
//
////ros::Rate loop_rate(10);
//
////using namespace pinocchio;
//
//// You should change here to set up your own URDF file or just pass it as an argument of this example.
//const std::string urdf_filename = std::string("/home/ola/catkin_test_ws/src/pinocchio_test_pkgs/models/example-robot-data/robots/ur_description/urdf/ur5_robot.urdf");
//
//// Load the urdf model
//struct pinocchio::ModelTpl<double> model;
//pinocchio::urdf::buildModel(urdf_filename,model);
//std::cout << "model name: " << model.name << std::endl;
//
//// Create data required by the algorithms
//Data data(model);
//
//// Sample a random configuration
//Eigen::VectorXd q = randomConfiguration(model);
//std::cout << "q: " << q.transpose() << std::endl;
//
//// Perform the forward kinematics over the kinematic tree
//forwardKinematics(model,data,q);
//
//// Print out the placement of each joint of the kinematic tree
//for(JointIndex joint_id = 0; joint_id < (JointIndex)model.njoints; ++joint_id)
//  std::cout << std::setw(24) << std::left
//            << model.names[joint_id] << ": "
//            << std::fixed << std::setprecision(2)
//            << data.oMi[joint_id].translation().transpose()
//            << std::endl;
//
//
////while (ros::ok())
////{
////  std_msgs::String msg;
////  msg.data = "hello world";
//
////  chatter_pub.publish(msg);

  //  ros::spinOnce();

  //  loop_rate.sleep();
 // }
  pinocchio::Model model;
  //pinocchio::buildModels::manipulator(model);
  //const std::string urdf_filename = std::string("/home/ola/catkin_test_ws/src/pinocchio_test_pkgs/models/example-robot-data/robots/ur_description/urdf/ur5_robot.urdf");
  const std::string urdf_filename = std::string("/home/ola/Desktop/3link/urdf/3link.urdf");

  pinocchio::urdf::buildModel(urdf_filename,model);
  pinocchio::Data data(model);

  Eigen::VectorXd q = pinocchio::neutral(model);
  Eigen::VectorXd v = Eigen::VectorXd::Zero(model.nv);
  Eigen::VectorXd a = Eigen::VectorXd::Zero(model.nv);

  const Eigen::VectorXd & tau = pinocchio::rnea(model,data,q,v,a);
  std::cout << "tau = " << tau.transpose() << std::endl;

  return 0;
}
