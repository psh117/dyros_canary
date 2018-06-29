#ifndef DYROS_CANARY_MODEL_H
#define DYROS_CANARY_MODEL_H

#include <string>
#include <map>

#include <ros/ros.h>
#include <ros/package.h>
#include <rbdl/rbdl.h>
#include <rbdl/addons/urdfreader/urdfreader.h>
#include "math_type_define.h"


namespace dyros_canary_controller
{

class DyrosCanaryModel
{
public:
  DyrosCanaryModel();

  enum EndEffector : unsigned int {
    EE_LEFT_HAND, EE_RIGHT_HAND};

  static constexpr size_t TOTAL_DOF = 22;
  static constexpr size_t MOBILE_BASE_DOF = 4;
  static constexpr size_t ARM_DOF = 7;
  static constexpr size_t HAND_DOF = 2;

  static const std::string JOINT_NAME[TOTAL_DOF];

  static constexpr const char* EE_NAME[2] =
      {"panda_left_hand", "panda_right_hand" };

  unsigned int end_effector_id_[2];
  const unsigned int joint_start_index_[2];

  void test();

  std::map<std::string, size_t> joint_name_map_;
  inline size_t getIndex(const std::string& joint_name) const
  {
    return joint_name_map_.at(joint_name);
  }
  // Calc Jacobian, Transformation
  void updateKinematics(const Eigen::VectorXd &q);
  void updateSensorData(const Eigen::Vector6d &r_ft, const Eigen::Vector6d &l_ft);

  void getTransformEndEffector(EndEffector ee, Eigen::Isometry3d* transform_matrix);
  void getTransformEndEffector(EndEffector ee, Eigen::Vector3d* position, Eigen::Matrix3d* rotation);

  void getTransformEndEffector(EndEffector ee, const Eigen::VectorXd& q, bool update_kinematics,
                                  Eigen::Vector3d* position, Eigen::Matrix3d* rotation);

  void getJacobianMatrix7DoF(EndEffector ee, Eigen::Matrix<double, 6, 7> *jacobian);


  const Eigen::Isometry3d& getCurrentTrasmfrom(EndEffector ee) { return currnet_transform_[ee]; }
  const Eigen::Matrix<double, 6, 7>& getArmJacobian(EndEffector ee) { return arm_jacobian_[ee-2]; }

private:
  RigidBodyDynamics::Model model_;

  Eigen::Matrix<double, DyrosCanaryModel::TOTAL_DOF, 1> q_;
  Eigen::Vector3d base_position_;

  Eigen::Isometry3d currnet_transform_[4];

  Eigen::Matrix<double, 6, 7> arm_jacobian_[2];
  Eigen::MatrixXd A_temp_;


};

typedef Eigen::Matrix<double, DyrosCanaryModel::TOTAL_DOF, 1> VectorQd;


}
#endif // DYROS_JET_MODEL_H
