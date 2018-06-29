#include "dyros_canary_controller/dyros_canary_model.h"



namespace dyros_canary_controller
{

// https://stackoverflow.com/questions/8016780/undefined-reference-to-static-constexpr-char
constexpr const char* DyrosCanaryModel::EE_NAME[2];
constexpr const size_t DyrosCanaryModel::TOTAL_DOF;
constexpr const size_t DyrosCanaryModel::MOBILE_BASE_DOF;
constexpr const size_t DyrosCanaryModel::ARM_DOF;
constexpr const size_t DyrosCanaryModel::HAND_DOF;

// These should be replaced by YAML or URDF or something
const std::string DyrosCanaryModel::JOINT_NAME[DyrosCanaryModel::TOTAL_DOF] = {
  "front_right_wheel", "front_left_wheel", "rear_right_wheel", "rear_left_wheel",
  "panda_left_joint1", "panda_left_joint2", "panda_left_joint3", "panda_left_joint4", "panda_left_joint5", "panda_left_joint6", "panda_left_joint7",
  "panda_right_joint1", "panda_right_joint2", "panda_right_joint3", "panda_right_joint4", "panda_right_joint5", "panda_right_joint6", "panda_right_joint7",
  "panda_left_finger_joint1","panda_left_finger_joint2",
  "panda_right_finger_joint1","panda_right_finger_joint2"};

DyrosCanaryModel::DyrosCanaryModel() :
  joint_start_index_{0, 7}
{
  A_temp_.resize(TOTAL_DOF, TOTAL_DOF);
  q_.setZero();

  std::string desc_package_path = ros::package::getPath("dyros_canary_description");
  std::string urdf_path = desc_package_path + "/robots/dyros_canary.urdf";

  ROS_INFO("Loading DYROS Canary description from = %s",urdf_path.c_str());
  //RigidBodyDynamics::Addons::URDFReadFromFile(urdf_path.c_str(), &model_, false, false);
  ROS_INFO("Successfully loaded.");
  ROS_INFO("Total DoF = %d", model_.dof_count);
  ROS_INFO("Total DoF = %d", model_.q_size);
  //model_.mJoints[0].)
  if(model_.dof_count != TOTAL_DOF)
  {
    ROS_WARN("The DoF in the model file and the code do not match.");
    ROS_WARN("Model file = %d, Code = %d", model_.dof_count, (int)TOTAL_DOF);
  }

  /*
  for (size_t i=0; i<2; i++)
  {
    end_effector_id_[i] = model_.GetBodyId(EE_NAME[i]);
    ROS_INFO("%s: id - %d",EE_NAME[i], end_effector_id_[i]);
    std::cout << model_.mBodies[end_effector_id_[i]].mCenterOfMass << std::endl;
  }
  */

  for (size_t i=0; i<TOTAL_DOF; i++)
  {
    joint_name_map_[JOINT_NAME[i]] = i;
  }
}


void DyrosCanaryModel::test()
{
  updateKinematics(Eigen::Vector22d::Zero());
  std::cout << "arm_jacobian_" << std::endl;
  std::cout << arm_jacobian_[0] << std::endl << std::endl;
  std::cout << arm_jacobian_[1] << std::endl;
  std::cout << "currnet_transform_" << std::endl;
  std::cout << currnet_transform_[0].translation() << std::endl << std::endl;
  std::cout << currnet_transform_[1].translation() << std::endl << std::endl;
  std::cout << currnet_transform_[2].translation() << std::endl << std::endl;
  std::cout << currnet_transform_[3].translation() << std::endl << std::endl;
}

void DyrosCanaryModel::updateKinematics(const Eigen::VectorXd& q)
{
  RigidBodyDynamics::UpdateKinematicsCustom(model_, &q, NULL, NULL);
  RigidBodyDynamics::CompositeRigidBodyAlgorithm(model_, q_, A_temp_, true);
  q_ = q;

  for(unsigned int i=0; i<2; i++)
  {
    getTransformEndEffector((EndEffector)i, &currnet_transform_[i]);
    getJacobianMatrix7DoF((EndEffector)i, &arm_jacobian_[i]);
  }
}



void DyrosCanaryModel::getTransformEndEffector // must call updateKinematics before calling this function
(EndEffector ee, Eigen::Isometry3d* transform_matrix)
{
  Eigen::Vector3d gghg = RigidBodyDynamics::CalcBodyToBaseCoordinates(model_, q_,end_effector_id_[ee], base_position_, false);
  transform_matrix->translation() = RigidBodyDynamics::CalcBodyToBaseCoordinates
      (model_, q_,end_effector_id_[ee], base_position_, false);
  transform_matrix->linear() = RigidBodyDynamics::CalcBodyWorldOrientation(
        model_, q_, end_effector_id_[ee], false).transpose();
}

void DyrosCanaryModel::getTransformEndEffector // must call updateKinematics before calling this function
(EndEffector ee, Eigen::Vector3d* position, Eigen::Matrix3d* rotation)
{
  *position = RigidBodyDynamics::CalcBodyToBaseCoordinates
      (model_, q_,end_effector_id_[ee], base_position_, false);
  *rotation = RigidBodyDynamics::CalcBodyWorldOrientation(
        model_, q_, end_effector_id_[ee], false).transpose();
}

void DyrosCanaryModel::getTransformEndEffector
(EndEffector ee, const Eigen::VectorXd& q, bool update_kinematics,
 Eigen::Vector3d* position, Eigen::Matrix3d* rotation)
{
  VectorQd q_new;
  q_new = q_;
  q_new.segment<7>(joint_start_index_[ee]) = q;
  if (update_kinematics)
  {
    q_ = q_new;
  }
  *position = RigidBodyDynamics::CalcBodyToBaseCoordinates
      (model_,q_new,end_effector_id_[ee], base_position_, update_kinematics);
  *rotation = RigidBodyDynamics::CalcBodyWorldOrientation(
        model_, q_new, end_effector_id_[ee], update_kinematics).transpose();
  // RigidBodyDynamics::Calcpo
  // model_.mBodies[0].mCenterOfMass
}


void DyrosCanaryModel::getJacobianMatrix7DoF
(EndEffector ee, Eigen::Matrix<double, 6, 7> *jacobian)
{
  Eigen::MatrixXd full_jacobian(6,TOTAL_DOF);
  full_jacobian.setZero();
  RigidBodyDynamics::CalcPointJacobian6D(model_, q_, end_effector_id_[ee],
                                         Eigen::Vector3d::Zero(), full_jacobian, false);

  jacobian->block<3, 7>(0, 0) = full_jacobian.block<3, 7>(3, joint_start_index_[ee]);
  jacobian->block<3, 7>(3, 0) = full_jacobian.block<3, 7>(0, joint_start_index_[ee]);
}
}
