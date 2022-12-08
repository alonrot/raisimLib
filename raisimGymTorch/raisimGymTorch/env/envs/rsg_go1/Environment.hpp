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
    nJoints_ = gvDim_ - 6; // amarco: I guess we're removing the DOFs of the base here? How are the joint velocities ordered here?
    // std::cout << "nJoints_: " + std::to_string(nJoints_) + "\n"; // 12
    // std::cout << "gcDim_: " + std::to_string(gcDim_) + "\n"; // 19
    // std::cout << "gvDim_: " + std::to_string(gvDim_) + "\n"; // 18

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

    // Joint limits:
    this->pos_joint_lim_upper_.setZero(nJoints_);
    this->pos_joint_lim_lower.setZero(nJoints_);
    this->pos_joint_lim_upper_ << 1.047, 2.966, -0.837, 1.047, 2.966, -0.837, 1.047, 2.966, -0.837, 1.047, 2.966, -0.837;
    this->pos_joint_lim_lower << -1.047, -0.663, -2.721, -1.047, -0.663, -2.721, -1.047, -0.663, -2.721, -1.047, -0.663, -2.721;


    // Eigen::VectorXd gc_tmp; gc_tmp.setZero(19);
    // gc_tmp << 0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0, 16.0, 17.0, 18.0;
    // std::cout << "gc_tmp(Eigen::seqN(7,4,3)): " << gc_tmp(Eigen::seqN(7,4,3)) << "\n";


    /// set pd gains
    Eigen::VectorXd jointPgain(gvDim_), jointDgain(gvDim_);
    jointPgain.setZero(); jointPgain.tail(nJoints_).setConstant(40.0);
    jointDgain.setZero(); jointDgain.tail(nJoints_).setConstant(2.0);
    go1->setPdGains(jointPgain, jointDgain);
    go1->setGeneralizedForce(Eigen::VectorXd::Zero(gvDim_));

    /// MUST BE DONE FOR ALL ENVIRONMENTS
    // body height (1) + body orientation (3) + joint angles (12) + body linear velocty (3) + body angular velocity (3) + joint velocity (12) = 34
    obDim_ = 34;
    std::cout << "obDim_: " + std::to_string(obDim_) + "\n";
    // actionDim_ = nJoints_;
    // actionDim_ = 4;
    actionDim_ = 6;
    actionMean_.setZero(nJoints_);
    actionStd_.setZero(nJoints_);
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


    
    // amarco: The above cfg["policy"]["action_std"].As<float>(); doesn't work, so we can't really expose action_std

    /// action scaling
    actionMean_ = gc_init_.tail(nJoints_);


    double action_std;
    // action_std = 0.01; // before changing reward function
    action_std = 0.75;

    // actionStd_.setConstant(0.3); // Original
    // actionStd_.setConstant(1.0); // amarco
    // actionStd_.setConstant(0.1); // amarco
    // actionStd_.setConstant(0.05); // amarco
    actionStd_.setConstant(action_std); // amarco

    // amarco:
    // this->seq_vel_hips = Eigen::seq(7+2,4,3); // [body_pos(3), body_ori(4), joint_pos(12)]
    // seq_vel_hips = Eigen::seq(7+2,4,3); // [...?]

    this->action_last.setZero(actionDim_);
    this->action_curr.setZero(actionDim_);
    this->action_clipped.setZero(actionDim_);

    // amarco: action clipping
    // Actions are desired positions, measured in radians.
    // <go1_const.h>
    // constexpr double go1_Hip_max   = 1.047;    // unit:radian ( = 60   degree)
    // constexpr double go1_Hip_min   = -1.047;   // unit:radian ( = -60  degree)
    // constexpr double go1_Thigh_max = 2.966;    // unit:radian ( = 170  degree)
    // constexpr double go1_Thigh_min = -0.663;   // unit:radian ( = -38  degree)
    // constexpr double go1_Calf_max  = -0.837;   // unit:radian ( = -48  degree)
    // constexpr double go1_Calf_min  = -2.721;   // unit:radian ( = -156 degree)
    this->action_lim = 5.0;



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

    trying_out_some_stuff();

  }

  void init() final { }

  void reset() final {
    go1->setState(gc_init_, gv_init_);

    // std::cout << "We are resetting!\n";
    // std::cout << "Initial position:\n";
    // std::cout << gc_init_.tail(12);

    // std::cout << "Initial velocity:\n";
    // std::cout << gv_init_.tail(12);

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



inline void trying_out_some_stuff(void){

  /// Let's check all contact impulses acting on "LF_SHANK"
  auto footIndex = go1->getBodyIdx("FR_calf");
  // std::cout << "footIndex: " << footIndex << "\n";

  /// for all contacts on the robot, check ...
  for(auto& contact: go1->getContacts()) {
    
    if (contact.skip())
      continue; /// if the contact is internal, one contact point is set to 'skip'
    
    if ( footIndex == contact.getlocalBodyIndex() ) {
    
      // std::cout << "Contact impulse in the contact frame, size: " << contact.getImpulse().e().size() << std::endl;
      // std::cout << "Contact impulse in the contact frame: " << contact.getImpulse().e() << std::endl;
      /// the impulse is acting from objectB to objectA. You can check if this object is objectA or B by
    
      std::cout << "is ObjectA: " << contact.isObjectA() << std::endl;
      // std::cout << "Contact frame: \n" << contact.getContactFrame().e().transpose() << std::endl;
      /// contact frame is transposed.
    
      // std::cout << "Contact impulse in the world frame: " << contact.getContactFrame().e().transpose() * contact.getImpulse().e() << std::endl;
      // std::cout << "Contact Normal in the world frame: " << contact.getNormal().e().transpose() << std::endl;
      // std::cout << "Contact position in the world frame: " << contact.getPosition().e().transpose() << std::endl;
      std::cout << "It collides with: " << world_->getObject(contact.getPairObjectIndex()) << std::endl;
    
      // if (contact.getPairContactIndexInPairObject() != raisim::BodyType::STATIC) {
      //   /// Static objects do not have Contacts store. So you must check if the pair object is static
      //   /// This saves computation in raisim
      //   world_->getObject(contact.getPairObjectIndex())->getContacts(); /// You can use the same methods on the pair object
      // }
    
    
    }
  }

}











  // amarco: added for calling it inside step, and exposing it to python
  inline float get_latest_rewards(void){

    // Reward velocity tracking:
    float sigma_tracking = 0.25;
    Eigen::Vector2d vel_body_lin_xy_des;
    vel_body_lin_xy_des << 0.5, 0.0; // 2022-12-07-14-04-48 ~2900 epochs; action_std = 0.5 -> pretty stupid policy, just dragging along the floor ...
    
    // Past trials
    // vel_body_lin_xy_des << 1.0, 0.0; // 2022-12-07-09-57-32 ~1000 epochs, walks forward, but ridoculously small steps; action_std = 0.2;
    // vel_body_lin_xy_des << 0.25, 0.0; // 2022-12-07-10-40-31 ~1500 epochs action_std = 0.5, walks forward but dragging feet and, not nice
    // vel_body_lin_xy_des << 0.0, 1.0; // 2022-12-07-09-28-02 -> ~1000 epochs, was walking laterally; action_std = 0.2;

    // 2022-12-07-20-06-57 -> Better rewqrd funcrion, walks vibrating...

    // 2022-12-08-08-38-21 -> Better rewqrd funcrion, walks vibrating...

    // 2022-12-08-09-25-39 -> Better rewqrd funcrion, walks vibrating...

    // 2022-12-08-10-48-31 -> Same as above; the robot sort of walks, but sliding the feet

    // Reward linear body velocity:
    float error_vel_lin_tracking = (vel_body_lin_xy_des-bodyLinearVel_.head(2)).squaredNorm();
    rewards_.record("vel_body_tracking_error", exp(-error_vel_lin_tracking/sigma_tracking));

    // Reward tracking angular velocity:
    float error_vel_ang_tracking = pow(0.0-bodyAngularVel_[2],2.0);
    rewards_.record("vel_ang_tracking_error", exp(-error_vel_ang_tracking/sigma_tracking));

    // Reward tracking robot height:
    float error_height_tracking = pow(0.3435-gc_[2],2.0);
    rewards_.record("height_body_tracking_error", exp(-error_height_tracking/sigma_tracking));

    // Penalize torque:
    rewards_.record("torque", go1->getGeneralizedForce().squaredNorm());

    // Penalize vertical velocity:
    rewards_.record("vel_body_lin_z", pow(bodyLinearVel_[2],2.0));

    // Penalize roll and pitch:
    rewards_.record("vel_ang_xy", (bodyAngularVel_.head(2)).squaredNorm() );

    // amarco:

    // How to avoid the "sliding mode" in which the robot moves but legs barely move?

    // Penalize hip joints positions, they shouldn't be moving too much for walking on a flat floor
    Eigen::Vector4d pos_hips_des; pos_hips_des << 0.0136, -0.0118, 0.0105, -0.0102;
    rewards_.record("pos_hips_des", (pos_hips_des-gc_(Eigen::seqN(7,4,3))).squaredNorm());

    // Eigen::VectorXd gc_tmp; gc_tmp << 0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18;
    // std::cout << "gc_tmp(Eigen::seqN(7,4,3)): " << gc_tmp(Eigen::seqN(7,4,3)) << "\n";


    // Penalize hip joints velocities: What are the indices for the hips?
    // Eigen::Vector4d vel_hips_des; vel_hips_des << 0.0, 0.0, 0.0;
    // rewards_.record("vel_hips_des", (vel_hips_des-gv_(this->seq_hips)).squaredNorm());

    // Penalize joint positions that are out of limits:
    double pos_joint_off_limits_val = -(gc_.tail(12)-this->pos_joint_lim_lower).cwiseMin(0.0).sum();
    pos_joint_off_limits_val += (gc_.tail(12)-this->pos_joint_lim_upper_).cwiseMax(0.0).sum();
    rewards_.record("pos_joint_off_limits", pos_joint_off_limits_val);
    // cwiseMin(action_lim).cwiseMax(-action_lim);

    // Penalize jumps in consecutive actions:
    rewards_.record("action_rate", (this->action_curr - this->action_last).squaredNorm());
    for(int ii; ii < this->action_last.size(); ii++){
      this->action_last[ii] = this->action_curr[ii];
    }

    // TODO: Reduce the action space by simply coupling the FR RL legs together. Reduce by half.



    return rewards_.sum();
  }


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

    trying_out_some_stuff();

    // std::cout << "action.size(): " << action.size() << "\n";

    // std::cout << "Before:\n";
    // std::cout << "action: " << action.transpose().format(this->clean_format) << "\n";
    // std::cout << "action curr: " << this->action_curr.transpose().format(this->clean_format) << "\n";
    // std::cout << "action clipped: " << this->action_clipped.transpose().format(this->clean_format) << "\n";

    // We need to modify the action. Copy it:
    for(int ii; ii<this->action_curr.size();ii++){
      this->action_curr[ii] = action[ii];
      this->action_clipped[ii] = action[ii];
    }

    // Clip the action:
    this->action_clipped.cwiseMin(this->action_lim).cwiseMax(-this->action_lim);
    // std::cout << "After:\n";
    // std::cout << "action: " << action.transpose().format(this->clean_format) << "\n";
    // std::cout << "action curr: " << this->action_curr.transpose().format(this->clean_format) << "\n";
    // std::cout << "action clipped: " << this->action_clipped.transpose().format(this->clean_format) << "\n";

    // Parse into desired position:


    // Couple:
    Eigen::VectorXd action_des;
    action_des.setZero(this->nJoints_);
    action_des[0] = this->action_clipped[0];
    action_des[1] = this->action_clipped[1];
    action_des[2] = this->action_clipped[2];

    action_des[9] = -this->action_clipped[0];
    action_des[10] = this->action_clipped[1];
    action_des[11] = this->action_clipped[2];

    action_des[3] = -this->action_clipped[3];
    action_des[4] = this->action_clipped[4];
    action_des[5] = this->action_clipped[5];

    action_des[6] = this->action_clipped[3];
    action_des[7] = this->action_clipped[4];
    action_des[8] = this->action_clipped[5];


    // // Couple:
    // Eigen::VectorXd action_des;
    // action_des.setZero(this->nJoints_);
    // // 0.0136, -0.0118, 0.0105, -0.0102;
    // action_des[0] = 0.0; // By setting zero here, we'll be applying just the initial position as desired joint position
    // action_des[1] = this->action_clipped[0];
    // action_des[2] = this->action_clipped[1];

    // action_des[9] = 0.0;
    // action_des[10] = this->action_clipped[0];
    // action_des[11] = this->action_clipped[1];

    // action_des[3] = 0.0;
    // action_des[4] = this->action_clipped[2];
    // action_des[5] = this->action_clipped[3];

    // action_des[6] = 0.0;
    // action_des[7] = this->action_clipped[2];
    // action_des[8] = this->action_clipped[3];



    
    /// action scaling
    // pTarget12_ = action.cast<double>(); // original
    // pTarget12_ = this->action_clipped.cast<double>(); // amarco
    pTarget12_ = action_des.cast<double>(); // amarco
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

    // integration_steps_ = 1;
    for(int i=0; i< integration_steps_; i++){
      if(server_) server_->lockVisualizationServerMutex();
      world_->integrate();
      if(server_) server_->unlockVisualizationServerMutex();
    }

    updateObservation();

    return get_latest_rewards();
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
  Eigen::VectorXd pos_joint_lim_upper_;
  Eigen::VectorXd pos_joint_lim_lower;
  Eigen::VectorXd action_last;
  Eigen::VectorXd action_curr;
  Eigen::VectorXd action_clipped;
  // auto seq_vel_hips;
  double action_lim;
  Eigen::IOFormat clean_format;

  /// these variables are not in use. They are placed to show you how to create a random number sampler.
  std::normal_distribution<double> normDist_;
  thread_local static std::mt19937 gen_;
};
thread_local std::mt19937 raisim::ENVIRONMENT::gen_;

}

