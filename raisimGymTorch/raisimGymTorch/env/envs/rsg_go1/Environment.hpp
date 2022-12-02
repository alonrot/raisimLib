//----------------------------//
// This file is part of RaiSim//
// Copyright 2020, RaiSim Tech//
//----------------------------//

#pragma once

#include <stdlib.h>
#include <set>
#include "../../RaisimGymEnv.hpp"

namespace raisim {

class ENVIRONMENT : public RaisimGymEnv {

 public:

  explicit ENVIRONMENT(const std::string& resourceDir, const Yaml::Node& cfg, bool visualizable) :
      RaisimGymEnv(resourceDir, cfg), visualizable_(visualizable), normDist_(0, 1) {

    /// create world
    world_ = std::make_unique<raisim::World>();

    /// add objects
    // http://wiki.ros.org/urdf/XML/joint
    // go1 = world_->addArticulatedSystem("/Users/alonrot/work/code_projects_WIP/unitree_pybullet_modif/data/go1_description/urdf/go1.urdf");
    go1 = world_->addArticulatedSystem("/Users/alonrot/work/code_projects_WIP/unitree_mujoco/data/go1/urdf/go1.urdf");
    go1->setName("go1");
    go1->setControlMode(raisim::ControlMode::PD_PLUS_FEEDFORWARD_TORQUE);
    world_->addGround();

    // amarco: not using this; left here for reference
    // size_t id1 = go1->getFrameIdxByName("FR_hip_joint");
    // std::cout << "id1: " + std::to_string(id1) + "\n";
    
    /// get robot data
    gcDim_ = go1->getGeneralizedCoordinateDim();
    gvDim_ = go1->getDOF();
    nJoints_ = gvDim_ - 6; // amarco: I guess we're removing the DOFs of the base here?
    std::cout << "nJoints_: " + std::to_string(nJoints_) + "\n";

    /// initialize containers
    gc_.setZero(gcDim_); gc_init_.setZero(gcDim_);
    gv_.setZero(gvDim_); gv_init_.setZero(gvDim_);
    pTarget_.setZero(gcDim_); vTarget_.setZero(gvDim_); pTarget12_.setZero(nJoints_);

    /// this is nominal configuration of go1
    // [com [Y(green), X(red), Z(blue)], quaternion [1,i,j,k]], joint_positions [hip, thigh, knee]x[FR, FL, RR, RL] ] // amarco: the [FR, FL, RR, RL] order is assumed from the visualizer; double check with the URDF and also using getFrameIdxByName() from raisim/mac/include/raisim/object/ArticulatedSystem/ArticulatedSystem.hpp
    // gc_init_ << 0.0, -1.5, 0.54, 1.0, 0.0, 0.0, 0.0, 0.01, 0.4, -1.3, -0.01, 0.4, -1.3, 0.01, -0.4, 1.3, -0.01, -0.4, 1.3; // go1
    // gc_init_ << 0.1454, -1.4882, 0.3408, 1.0, 0.0, 0.0, 0.0, 0.0273, 0.3974, -1.3512, -0.0371, 0.4368, -1.3878, 0.0218, -0.4503, 1.3713, -0.0141, -0.3949, 1.347; // go1 - exactly touching the ground, for learning
    


    // gc_init_ << 0.1454, -1.4882, 0.3408, 1.0, 0.0, 0.0, 0.0, 0.0273, 0.3974, -1.3512, -0.0371, 0.4368, -1.3878, 0.0218, -0.4503, 1.3713, -0.0141, -0.3949, 1.347; // go1
    // gc_init_ << 0.1454, -1.4882, 0.3408, 1.0, 0.0, 0.0, 0.0, 0.0273, 0.50, -1.3512, -0.0371, 0.4368, -1.3878, 0.0218, -0.4503, 1.3713, -0.0141, -0.3949, 1.347; // go1




    // CORRECT posture FROM PYTHON:
    gc_init_ << 0.0, -1.5245, 0.3435, 1.0, 0.0, 0.0, 1.0, 0.0136, 0.7304, -1.4505, -0.0118, 0.7317, -1.4437, 0.0105, 0.6590, -1.3903, -0.0102, 0.6563, -1.3944;
    // go1_nominal_joint_config = np.array([0.0, -1.5245, 0.3435, 1.0, 0.0, 0.0, 1.0, # (com [Y(green), X(red), Z(blue)], quaternion [1,i,j,k]],...
    //                                     0.0136, 0.7304, -1.4505,  # FR [hip lateral, hip, knee]
    //                                     -0.0118, 0.7317, -1.4437, # FL [hip lateral, hip, knee]
    //                                     0.0105, 0.6590, -1.3903, # RR [hip lateral, hip, knee]
    //                                     -0.0102, 0.6563, -1.3944]) # RL [hip lateral, hip, knee]


    // // INCORRECT posture:
    // gc_init_ << 0.1454, -1.4882, 0.3408, 
    // 1.0, 0.0, 0.0, 0.0, 
    // 0.0273, 0.3974, -1.3512, // FR
    // -0.0371, 0.4368, -1.3878, 
    // 0.0218, -0.4503, 1.3713, 
    // -0.0141, -0.3949, 1.347; // go1


    /// set pd gains
    Eigen::VectorXd jointPgain(gvDim_), jointDgain(gvDim_);
    jointPgain.setZero(); jointPgain.tail(nJoints_).setConstant(50.0);
    jointDgain.setZero(); jointDgain.tail(nJoints_).setConstant(0.2);
    go1->setPdGains(jointPgain, jointDgain);
    go1->setGeneralizedForce(Eigen::VectorXd::Zero(gvDim_));

    /// MUST BE DONE FOR ALL ENVIRONMENTS
    // body height (1) + body orientation (3) + joint angles (12) + body linear velocty (3) + body angular velocity (3) + joint velocity (12) = 34
    obDim_ = 34;
    std::cout << "obDim_: " + std::to_string(obDim_) + "\n";
    actionDim_ = nJoints_; actionMean_.setZero(actionDim_); actionStd_.setZero(actionDim_);
    obDouble_.setZero(obDim_);

    // float action_std = cfg["policy"]["action_std"].As<float>();

    // std::cout << "simdt: " + std::to_string(cfg_["simulation_dt"].template As<double>()) + "\n";

    // // amarco: reading from the cfg file works, but the values are empty or zero
    // double action_std;
    // if(&cfg["policy"]["action_std"]){
    //   action_std = cfg["policy"]["action_std"].template As<double>();
    // }
    // else{
    //   action_std = -1.0;
    // }
    // std::cout << "action_std: " + std::to_string(action_std) + "\n";

    // std::string trial_str;
    // trial_str = cfg["trial"].template As<std::string>();
    // std::cout << "trial_str: " + trial_str + "\n";


    // gggggggggggggg
    // amarco: The above cfg["policy"]["action_std"].As<float>(); doesn't work, so we can't really expose action_std

    /// action scaling
    actionMean_ = gc_init_.tail(nJoints_);


    double action_std;
    action_std = 0.01;

    // actionStd_.setConstant(0.3); // Original
    // actionStd_.setConstant(1.0); // amarco
    // actionStd_.setConstant(0.1); // amarco
    // actionStd_.setConstant(0.05); // amarco
    actionStd_.setConstant(action_std); // amarco

    /// Reward coefficients
    // rewards_.initializeFromConfigurationFile(cfg["reward"]); // amarco: original
    rewards_.initializeFromConfigurationFile(cfg["reward"]);

    /// indices of links that should not make contact with ground
    /// amarco: with anymal, this is the shank
    footIndices_.insert(go1->getBodyIdx("FL_calf")); // go1
    footIndices_.insert(go1->getBodyIdx("FR_calf")); // go1
    footIndices_.insert(go1->getBodyIdx("RL_calf")); // go1
    footIndices_.insert(go1->getBodyIdx("RR_calf")); // go1

    /// visualize if it is the first environment
    if (visualizable_) {
      server_ = std::make_unique<raisim::RaisimServer>(world_.get());
      server_->launchServer();
      server_->focusOn(go1);
    }
  }

  void init() final { }

  void reset() final {
    go1->setState(gc_init_, gv_init_);

    // // amarco: possibility for making the robot wait a bit before moving:
    // for(int i=0; i< 100; i++){
    //   this->step(Eigen::VectorZeros(...))
    // }

    updateObservation();

  }

  // // amarco: added for calling it inside step, and exposing it to python
  // inline void setPdTarget_from_commanded_action(const Eigen::Ref<EigenVec>& action) {

  //   /// action scaling
  //   pTarget12_ = action.cast<double>();
  //   pTarget12_ = pTarget12_.cwiseProduct(actionStd_); // amarco: element-wise product; https://eigen.tuxfamily.org/dox/group__TutorialArrayClass.html
  //   pTarget12_ += actionMean_;
  //   pTarget_.tail(nJoints_) = pTarget12_;

  //   go1->setPdTarget(pTarget_, vTarget_);

  // }

  // // amarco: added for calling it inside step, and exposing it to python
  // inline void step_integrate_once(void){

  //   if(server_) server_->lockVisualizationServerMutex();
  //   world_->integrate();
  //   if(server_) server_->unlockVisualizationServerMutex();

  // }

  // // amarco: added for calling it inside step, and exposing it to python
  // inline void get_latest_rewards(void){
  //   rewards_.record("torque", go1->getGeneralizedForce().squaredNorm());
  //   rewards_.record("forwardVel", std::min(4.0, bodyLinearVel_[0])); // Saturate velocity

  //   return rewards_.sum();
  // }


  // // amarco: modified step function
  // float step(const Eigen::Ref<EigenVec>& action) final {

  //   setPdTarget_from_commanded_action(action);

  //   for(int i=0; i< integration_steps_; i++){
  //     step_integrate_once();
  //   }

  //   updateObservation();

  //   return get_latest_rewards();
  // }


  // amarco: original step function (replaced by the above function; it does exactly the same)
  float step(const Eigen::Ref<EigenVec>& action) final {

    /// action scaling
    pTarget12_ = action.cast<double>();
    pTarget12_ = pTarget12_.cwiseProduct(actionStd_); // amarco: element-wise product; https://eigen.tuxfamily.org/dox/group__TutorialArrayClass.html
    pTarget12_ += actionMean_;
    pTarget_.tail(nJoints_) = pTarget12_;

    go1->setPdTarget(pTarget_, vTarget_);

    // amarco NOTE: Even though it's inefficient to recompute this here as opposed to
    // computing it only once in the header, we can't do it in the header
    // because control_dt_ and simulation_dt_ are defined as some default values and
    // they will suffer changes once the VectorizedEnvironment functions are called
    integration_steps_ = int(control_dt_ / simulation_dt_ + 1e-10);
    
    // std::cout << "integration_steps_: " + std::to_string(integration_steps_) + "\n";
    // std::cout << "control_dt_: " + std::to_string(control_dt_) + "\n";
    // std::cout << "simulation_dt_: " + std::to_string(simulation_dt_) + "\n";

    for(int i=0; i< integration_steps_; i++){
      if(server_) server_->lockVisualizationServerMutex();
      world_->integrate();
      if(server_) server_->unlockVisualizationServerMutex();
    }

    updateObservation();

    rewards_.record("torque", go1->getGeneralizedForce().squaredNorm());
    // rewards_.record("forwardVel", std::min(4.0, bodyLinearVel_[0])); // amarco: original
    rewards_.record("forwardVel", std::min(2.0, bodyLinearVel_[0])); // amarco: original

    rewards_.record("lateralVel", std::min(2.0, bodyLinearVel_[0])); // amarco: original

    // rewards_.record("com_y", pow(gc_[1],2.0)); // amarco: added; pow(base,exponent), https://cplusplus.com/reference/cmath/pow/

    return rewards_.sum();
  }

  void updateObservation() {
    go1->getState(gc_, gv_);
    raisim::Vec<4> quat;
    raisim::Mat<3,3> rot;
    quat[0] = gc_[3]; quat[1] = gc_[4]; quat[2] = gc_[5]; quat[3] = gc_[6];
    raisim::quatToRotMat(quat, rot);
    bodyLinearVel_ = rot.e().transpose() * gv_.segment(0, 3);
    bodyAngularVel_ = rot.e().transpose() * gv_.segment(3, 3);

    obDouble_ << gc_[2], /// body height
        rot.e().row(2).transpose(), /// body orientation
        gc_.tail(12), /// joint angles
        bodyLinearVel_, bodyAngularVel_, /// body linear&angular velocity
        gv_.tail(12); /// joint velocity
  }

  void observe(Eigen::Ref<EigenVec> ob) final {
    /// convert it to float
    ob = obDouble_.cast<float>();
  }

  bool isTerminalState(float& terminalReward) final {
    terminalReward = float(terminalRewardCoeff_);

    /// if the contact body is not feet
    for(auto& contact: go1->getContacts())
      if(footIndices_.find(contact.getlocalBodyIndex()) == footIndices_.end())
        return true;

    terminalReward = 0.f;
    return false;
  }

  void curriculumUpdate() { };

  // int get_nr_of_times_integration_is_called_inside_step_for_the_same_applied_action() {
  //   return integration_steps_
  // }

 private:
  int gcDim_, gvDim_, nJoints_;
  bool visualizable_ = false;
  raisim::ArticulatedSystem* go1;
  Eigen::VectorXd gc_init_, gv_init_, gc_, gv_, pTarget_, pTarget12_, vTarget_;
  double terminalRewardCoeff_ = -10.;
  Eigen::VectorXd actionMean_, actionStd_, obDouble_;
  Eigen::Vector3d bodyLinearVel_, bodyAngularVel_;
  std::set<size_t> footIndices_;

  // amarco added
  int integration_steps_;

  /// these variables are not in use. They are placed to show you how to create a random number sampler.
  std::normal_distribution<double> normDist_;
  thread_local static std::mt19937 gen_;
};
thread_local std::mt19937 raisim::ENVIRONMENT::gen_;

}

