//----------------------------//
// This file is part of RaiSim//
// Copyright 2020, RaiSim Tech//
//----------------------------//

#pragma once

#include <stdlib.h>
#include <set>
#include "../../RaisimGymEnv.hpp"

// amarco:
#include <math.h>
#include <cmath>
#include <cfloat>

enum action_parsing_types : int {

  STANDARD,
  COUPLED,
  COUPLED_NO_LAT_HIP,
  COUPLED_CPG

};

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


    // // float dummy_val1 = cfg.dummy.template As<float>();
    // RSFATAL_IF(cfg["dummy"].IsNone(), "Node dummy doesn't exist");
    // float dummy_val = cfg["dummy"].template As<float>();
    // std::cout << "dummy_val: " << dummy_val << "\n";

    float P_gains_val = 0.0;
    read_value_from_cfg_file(cfg,"P_gains_val",P_gains_val);
    float D_gains_val = 0.0;
    read_value_from_cfg_file(cfg,"D_gains_val",D_gains_val);
    float action_std = 0.0;
    read_value_from_cfg_file(cfg,"action_std",action_std);
    read_value_from_cfg_file(cfg,"action_lim",this->action_lim);
    
    this->external_force.setZero(3);
    double force_impulse_val = 0.0;
    read_value_from_cfg_file(cfg,"force_impulse_val",force_impulse_val);
    // this->external_force[0] = force_impulse_val;
    this->external_force[1] = force_impulse_val;
    std::cout << "external_force: " << this->external_force.transpose().format(this->clean_format) << "\n";

    read_value_from_cfg_file(cfg,"Nsteps_force_impulse",this->Nsteps_force_impulse);
    read_value_from_cfg_file(cfg,"height_body_desired",this->height_body_desired);

    read_value_from_cfg_file(cfg,"action_parametrization",this->action_parametrization);
    
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



    // // NOTE: The getSimulationTimeStep() will return a default value, different from the user-defined desired value, specified in the config file. Such value will be assigned later on by upper classes, after step() is called for the first time
    // std::cout << "[initialization] this->getControlTimeStep(): " << this->getControlTimeStep() << "\n";
    // std::cout << "[initialization] this->getSimulationTimeStep(): " << this->getSimulationTimeStep() << "\n";


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
    // jointPgain.setZero(); jointPgain.tail(nJoints_).setConstant(40.0);
    // jointDgain.setZero(); jointDgain.tail(nJoints_).setConstant(2.0);
    jointPgain.setZero(); jointPgain.tail(nJoints_).setConstant(P_gains_val);
    jointDgain.setZero(); jointDgain.tail(nJoints_).setConstant(D_gains_val);
    go1->setPdGains(jointPgain, jointDgain);
    go1->setGeneralizedForce(Eigen::VectorXd::Zero(gvDim_));


    // Select action parsing:
    // this->action_parametrization = COUPLED;
    // this->action_parametrization = COUPLED_NO_LAT_HIP;
    // this->action_parametrization = COUPLED_CPG;


    if(this->action_parametrization == STANDARD){
      this->actionDim_ = nJoints_;
      std::cout << "action_parametrization: STANDARD | dim_action: " << this->actionDim_ << "\n";
    }

    if(this->action_parametrization == COUPLED){
      this->actionDim_ = 6;
      std::cout << "action_parametrization: COUPLED | dim_action: " << this->actionDim_ << "\n";
    }

    if(this->action_parametrization == COUPLED_NO_LAT_HIP){
      this->actionDim_ = 4;
      std::cout << "action_parametrization: COUPLED_NO_LAT_HIP | dim_action: " << this->actionDim_ << "\n";
    }

    if(this->action_parametrization == COUPLED_CPG){
      this->actionDim_ = 4;
      std::cout << "action_parametrization: COUPLED_CPG | dim_action: " << this->actionDim_ << "\n";
    }



    /// MUST BE DONE FOR ALL ENVIRONMENTS
    // body height (1) + body orientation (3) + joint angles (12) + body linear velocty (3) + body angular velocity (3) + joint velocity (12) = 34
    obDim_ = 34;
    std::cout << "obDim_: " + std::to_string(obDim_) + "\n";


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


    // double action_std;
    // action_std = 0.01; // before changing reward function
    // action_std = 0.75;
    // action_std = 1.0;

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
    this->first_time = true;





    // amarco: action clipping
    // Actions are desired positions, measured in radians.
    // <go1_const.h>
    // constexpr double go1_Hip_max   = 1.047;    // unit:radian ( = 60   degree)
    // constexpr double go1_Hip_min   = -1.047;   // unit:radian ( = -60  degree)
    // constexpr double go1_Thigh_max = 2.966;    // unit:radian ( = 170  degree)
    // constexpr double go1_Thigh_min = -0.663;   // unit:radian ( = -38  degree)
    // constexpr double go1_Calf_max  = -0.837;   // unit:radian ( = -48  degree)
    // constexpr double go1_Calf_min  = -2.721;   // unit:radian ( = -156 degree)
    // this->action_lim = 10.0;
    // this->action_lim = action_lim;

    this->time_counter = 0.0;
    this->generalized_force.setZero(18);


    this->joint_vel_curr.setZero(this->nJoints_);
    this->joint_vel_last.setZero(this->nJoints_);
    this->joint_acc_curr.setZero(this->nJoints_);
    this->first_time = true;



    /// Reward coefficients
    // rewards_.initializeFromConfigurationFile(cfg["reward"]); // amarco: original
    rewards_.initializeFromConfigurationFile(cfg["reward"]);

    /// indices of links that should not make contact with ground
    /// amarco: with anymal, this is the shank
    footIndices_.insert(go1->getBodyIdx("FL_calf")); // go1
    footIndices_.insert(go1->getBodyIdx("FR_calf")); // go1
    footIndices_.insert(go1->getBodyIdx("RL_calf")); // go1
    footIndices_.insert(go1->getBodyIdx("RR_calf")); // go1

    for(auto ii=footIndices_.begin(); ii!=footIndices_.end(); ii++){
      std::cout << "footIndices_: " << *ii << "\n";
    }

    this->footIndices_mine = {go1->getBodyIdx("FR_calf"),   // 3
                              go1->getBodyIdx("FL_calf"),   // 6
                              go1->getBodyIdx("RR_calf"),   // 9
                              go1->getBodyIdx("RL_calf")};  // 12

    for(int ii=0; ii<this->footIndices_mine.size();ii++){
      std::cout << "this->footIndices_mine[" << ii << "] = " << this->footIndices_mine[ii] << "\n";
    }

    this->foot_in_contact_curr = {0, 0, 0, 0};
    this->foot_in_contact_last = {0, 0, 0, 0};

    this->time_good_contacts = 0.0;
    this->time_bad_contacts = 0.0;

    this->counter_force = 0;

    this->apply_impulse = true;

    this->yaw_cum = 0.0;




    detect_contact();

    // auto aux1 = footIndices_.find(12);
    // std::cout << "aux_exists: " << *aux1 << "\n";


    // auto aux2 = footIndices_.find(4);
    // std::cout << "aux_doesnt_exists: " << *aux2 << "\n";


    // std::cout << footIndices_ << "\n";

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

    // std::cout << "We are resetting!\n";
    // std::cout << "Initial position:\n";
    // std::cout << gc_init_.tail(12);

    // std::cout << "Initial velocity:\n";
    // std::cout << gv_init_.tail(12);

    // // amarco: possibility for making the robot wait a bit before moving:
    // for(int i=0; i< 100; i++){
    //   this->step(Eigen::VectorZeros(...))
    // }

    this->time_counter = 0.0;
    this->time_good_contacts = 0.0;
    this->time_bad_contacts = 0.0;
    this->yaw_cum = 0.0;

    this->joint_vel_curr.setZero(this->nJoints_);
    this->joint_vel_last.setZero(this->nJoints_);
    this->joint_acc_curr.setZero(this->nJoints_);

    this->apply_impulse = true;
    this->counter_force = 0;


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


template<typename T>
inline void read_value_from_cfg_file(const Yaml::Node& cfg, const std::string & name, T & val){

    // float dummy_val1 = cfg.dummy.template As<float>();
    RSFATAL_IF(cfg[name].IsNone(), "Node " << name << " doesn't exist");
    val = cfg[name].template As<T>();
    std::cout << name << ": " << val << "\n";

    return;
}

inline int detect_contact(void){

  // Initialize:
  for(int ii=0; ii < this->foot_in_contact_curr.size(); ii++){
    this->foot_in_contact_curr[ii] = 0;
  }

  /// for all contacts on the robot, check ...
  for(auto& contact: go1->getContacts()) {
    
    // TODO: What do the mean by "contact is internal"? Contact of the joints with themselves or with other joints?
    if (contact.skip())
      continue; /// if the contact is internal, one contact point is set to 'skip'
    
    for(int ii=0; ii < this->footIndices_mine.size(); ii++){

      if(this->footIndices_mine[ii] == contact.getlocalBodyIndex()){
        this->foot_in_contact_curr[ii] = 1;
      }

    }

  }


  int sum_contact = 0;
  // std::cout << "this->foot_in_contact_curr:\n";
  for(int ii=0; ii < this->foot_in_contact_curr.size(); ii++){
    // std::cout << this->foot_in_contact_curr[ii] << ", ";
    sum_contact += this->foot_in_contact_curr[ii];
  }
  // std::cout << "\n";

  // std::cout << "sum_contact: " << sum_contact << "\n";

  return sum_contact;

}


inline void update_time_in_the_air(void){

  int sum_contact = this->detect_contact(); // update this->foot_in_contact_curr

  // Make sure that in the last time step the feel where also in the air. If not, return without increasing this->time_good_contacts
  bool same_contacts = true;
  for(int ii=0;ii<this->foot_in_contact_curr.size();ii++){
    if(this->foot_in_contact_curr[ii] != this->foot_in_contact_last[ii]){ same_contacts = false; }
  }

  // Always update this->foot_in_contact_last
  for(int ii=0;ii<this->foot_in_contact_last.size();ii++){
    this->foot_in_contact_last[ii] = this->foot_in_contact_curr[ii];
  }

  if(!same_contacts){
    this->time_bad_contacts += this->getControlTimeStep();
    return;
  }

  // Only add to this->time_good_contacts if the FR leg is in the air and the FL one is touching the ground, or viceversa
  if(this->foot_in_contact_curr[0] != this->foot_in_contact_curr[1]){
    this->time_good_contacts += this->getControlTimeStep();
  }
  else{
    this->time_bad_contacts += this->getControlTimeStep();
  }

  return;

}





  // amarco: added for calling it inside step, and exposing it to python
  inline float get_latest_rewards(void){

    // Reward velocity tracking:
    float sigma_tracking = 0.1;
    Eigen::Vector2d vel_body_lin_xy_des;
    // vel_body_lin_xy_des << 0.75, 0.0; // 2022-12-07-14-04-48 ~2900 epochs; action_std = 0.5 -> pretty stupid policy, just dragging along the floor ...
    vel_body_lin_xy_des << 0.25, 0.0; // 2022-12-07-14-04-48 ~2900 epochs; action_std = 0.5 -> pretty stupid policy, just dragging along the floor ...
    
    // Past trials
    // vel_body_lin_xy_des << 1.0, 0.0; // 2022-12-07-09-57-32 ~1000 epochs, walks forward, but ridoculously small steps; action_std = 0.2;
    // vel_body_lin_xy_des << 0.25, 0.0; // 2022-12-07-10-40-31 ~1500 epochs action_std = 0.5, walks forward but dragging feet and, not nice
    // vel_body_lin_xy_des << 0.0, 1.0; // 2022-12-07-09-28-02 -> ~1000 epochs, was walking laterally; action_std = 0.2;

    // 2022-12-07-20-06-57 -> Better rewqrd funcrion, walks vibrating...

    // 2022-12-08-08-38-21 -> Better rewqrd funcrion, walks vibrating...

    // 2022-12-08-09-25-39 -> Better rewqrd funcrion, walks vibrating...

    // 2022-12-08-10-48-31 -> Same as above; the robot sort of walks, but sliding the feet

    // 2022-12-08-18-17-06 -> It falls down... after 700 (maybe continue training, not sure)

    // 2022-12-08-22-58-21 -> ~1200, retrained from 2022-12-08-18-17-06, so in total ~1900 -> Same as above...

    // 2022-12-09-08-58-04 -> ~1000, same thing, a bit better

    // 2022-12-09-09-32-06 -> ~2000, a bit better but still dragging the feet

    // 2022-12-09-12-26-50 -> ~1200, using CPG, dragging totally

    // 2022-12-11-03-43-43 -> ~1400, after really changing a lot the reward function; still kind of dragging ...

    // 2022-12-11-04-51-14 -> ~1300, FINALLY!!! 

    // 2022-12-11-11-54-13 -> ~10,000 iters -> weird behaviors toward the end

    // 2022-12-11-19-18-59 -> ~500

    // 2022-12-12-09-31-44 -> ~500 iters -> not moving at all; using CPG

    // 2022-12-12-10-19-37 -> ~500 iters -> not moving at all; using CPG

    // 2022-12-12-10-59-50 -> ~1000 iters, running in place, promising, but actions too high, maybe retrain

    // 2022-12-12-11-20-08 -> ~650, stays still....

    // 2022-12-12-12-09-49 -> ~550, weird jump but kind of promising

    // 2022-12-12-12-40-56 ~650, jumps and short task

    // 2022-12-12-13-24-27 ~700 still kind of jumpy and short, testing ...

    // 2022-12-12-14-08-29 ~550, stepping very fast in place...

    // 2022-12-12-14-40-02 ~200, stepping in place, promising, not walking forward yet, this was COUPLED_NO_LAT_HIP

    // 2022-12-12-15-14-24, ~500, stepping in place, maybe missing a push?

    // 2022-12-12-15-58-27, ~350, now added the push, mich more promising

    // 2022-12-12-16-54-52, ~600 (+350), retraining of the above (2022-12-12-15-58-27); very promising. 
    // (Environment.hpp was changed after this expermient; see below)

    // 2022-12-12-18-21-55: ~1350, I changed the code to a slightly larger impulse and changed the forward velocity to 0.25

    // 2022-12-12-19-33-35: ~1450, walks!! in circles though... -> Using COUPLED
    //                      ~1750, I stopped it here. Walks in circles, but a bit worse than the previous policies

    // 2022-12-12-22-29-34, ~1350, stepping in place

    // 2022-12-12-23-36-07, ~900, doesn't move!

    // 2022-12-13-00-44-57, ~1350, stepping in place, not good... seems as if the force didnt' have any effect


    // Reward linear body velocity:
    float error_vel_lin_tracking = (vel_body_lin_xy_des-bodyLinearVel_.head(2)).squaredNorm();
    // rewards_.record("vel_body_tracking_error", exp(-error_vel_lin_tracking/sigma_tracking));
    rewards_.record("vel_body_tracking_error", error_vel_lin_tracking);

    // Reward tracking angular velocity:
    float error_vel_ang_tracking = pow(0.0-bodyAngularVel_[2],2.0);
    rewards_.record("vel_ang_tracking_error", exp(-error_vel_ang_tracking/sigma_tracking));

    // Reward tracking robot height:
    float error_height_tracking = pow(0.32-gc_[2],2.0);
    rewards_.record("height_body_tracking_error", exp(-error_height_tracking/sigma_tracking));

    // // Reward tracking yaw:
    // float body_ang_pos_yaw = rot.e().row(2)[2];
    // Eigen::Vector3d body_ang_pos_yaw = this->obDouble_(Eigen::seqN(3,1,3));
    // std::cout << "body_ang_pos_yaw: " << body_ang_pos_yaw.transpose().format(this->clean_format) << "\n";
    // float error_height_tracking = pow(0.25-gc_[2],2.0);
    // rewards_.record("height_body_tracking_error", exp(-error_height_tracking/sigma_tracking));

    // Penalize torque:
    float torque = go1->getGeneralizedForce().squaredNorm(); // This gives the feedforward torque applied to all joints. The first 6 elements are zero.
    // go1->getGeneralizedForce() -> This returns the generalized externally-applied feed-forward torque (in this case zero, as set in go1->setGeneralizedForce(Eigen::VectorXd::Zero(gvDim_));) PLUS the torque resulting from the PD controller
    rewards_.record("torque", torque);

    // Penalize vertical velocity:
    float vel_body_lin_z = pow(bodyLinearVel_[2],2.0);
    rewards_.record("vel_body_lin_z", vel_body_lin_z);

    // Penalize roll and pitch:
    float vel_ang_xy = (bodyAngularVel_.head(2)).squaredNorm();
    rewards_.record("vel_ang_xy", vel_ang_xy);

    // amarco:

    // How to avoid the "sliding mode" in which the robot moves but legs barely move?

    // Penalize hip joints positions, they shouldn't be moving too much for walking on a flat floor
    Eigen::Vector4d pos_hips_des; pos_hips_des << 0.0136, -0.0118, 0.0105, -0.0102;
    float pos_hips_des_err = (pos_hips_des-gc_(Eigen::seqN(7,4,3))).squaredNorm();
    rewards_.record("pos_hips_des", pos_hips_des_err);

    // Eigen::VectorXd gc_tmp; gc_tmp << 0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18;
    // std::cout << "gc_tmp(Eigen::seqN(7,4,3)): " << gc_tmp(Eigen::seqN(7,4,3)) << "\n";


    // Penalize hip joints velocities: What are the indices for the hips?
    // Eigen::Vector4d vel_hips_des; vel_hips_des << 0.0, 0.0, 0.0;
    // rewards_.record("vel_hips_des", (vel_hips_des-gv_(this->seq_hips)).squaredNorm());

    // Penalize joint positions that are out of limits:
    float pos_joint_off_limits_val = -(gc_.tail(this->nJoints_)-this->pos_joint_lim_lower).cwiseMin(0.0).sum();
    pos_joint_off_limits_val += (gc_.tail(this->nJoints_)-this->pos_joint_lim_upper_).cwiseMax(0.0).sum();
    rewards_.record("pos_joint_off_limits", pos_joint_off_limits_val);
    // cwiseMin(action_lim).cwiseMax(-action_lim);

    // Penalize jumps in consecutive actions:
    float action_rate = (this->action_curr - this->action_last).squaredNorm();
    rewards_.record("action_rate", action_rate);
    for(int ii; ii < this->action_last.size(); ii++){
      this->action_last[ii] = this->action_curr[ii];
    }

    // Reward when the FR is in contact but the FL is in the air, or viceversa:

    this->update_time_in_the_air();
    // std::cout << "this->time_good_contacts: " << this->time_good_contacts << "\n";
    rewards_.record("time_good_contacts", this->time_good_contacts);
    rewards_.record("time_bad_contacts", this->time_bad_contacts);


    if(this->action_parametrization == STANDARD || this->action_parametrization == COUPLED){
      // In these two cases, the lateral hip joint is part of the action space
      this->update_yaw();
      rewards_.record("heading_error", pow(0.0-this->yaw_cum,2.0));
    }



    // std::cout << "[in rewards] this->getControlTimeStep(): " << this->getControlTimeStep() << "\n";
    // std::cout << "[in rewards] this->getSimulationTimeStep(): " << this->getSimulationTimeStep() << "\n";



    // Penalize vibration:
    if(this->first_time){
      this->joint_vel_curr = gv_.tail(this->nJoints_);
      this->joint_acc_curr.setZero(this->nJoints_); // Not needed; it's already zero, but left here for clarity
      this->first_time = false;
    }
    else{
      this->joint_acc_curr = (this->joint_vel_curr - this->joint_vel_last) / this->getControlTimeStep();
    }
    for(int ii; ii < this->joint_vel_last.size();ii++){
      this->joint_vel_last[ii] = this->joint_vel_curr[ii];
    }
    float joint_acc_curr = this->joint_acc_curr.squaredNorm();
    rewards_.record("joint_acc_curr", joint_acc_curr);


    // std::cout << "error_vel_lin_tracking: " << error_vel_lin_tracking << "\n";
    // std::cout << "error_vel_ang_tracking: " << error_vel_ang_tracking << "\n";
    // std::cout << "error_height_tracking: " << error_height_tracking << "\n";
    // std::cout << "torque: " << torque << "\n";
    // std::cout << "vel_body_lin_z: " << vel_body_lin_z << "\n";
    // std::cout << "vel_ang_xy: " << vel_ang_xy << "\n";
    // std::cout << "pos_hips_des_err: " << pos_hips_des_err << "\n";
    // std::cout << "pos_joint_off_limits_val: " << pos_joint_off_limits_val << "\n";
    // std::cout << "action_rate: " << action_rate << "\n";
    // std::cout << "sum_contact: " << sum_contact << "\n";
    // std::cout << "time_in_the_air: " << time_in_the_air << "\n";
    // std::cout << "joint_acc_curr: " << joint_acc_curr << "\n";


    // print_if_bad("error_vel_lin_tracking", error_vel_lin_tracking);
    // print_if_bad("error_vel_ang_tracking", error_vel_ang_tracking);
    // print_if_bad("error_height_tracking", error_height_tracking);
    // bool is_bad = print_if_bad("torque", torque);

    // if(is_bad){
    //   this->generalized_force = go1->getGeneralizedForce();
    //   std::cout << "generalized_force: " << this->generalized_force << "\n";
    //   std::cout << "this->pTarget12_: " << this->pTarget12_.transpose().format(this->clean_format) << "\n";

    //   std::cout << "action curr: " << this->action_curr.transpose().format(this->clean_format) << "\n";

    //   // Eigen::VectorXd pgain(this->gvDim_);
    //   // Eigen::VectorXd dgain(this->gvDim_);
    //   // go1->getPdGains(pgain,dgain);
    //   // std::cout << "go1->kp_" << pgain << "\n";
    //   // std::cout << "go1->kd_" << dgain << "\n";


    //   // std::cout << "go1->uref_" << go1->uref_ << "\n";
    //   // std::cout << "go1->qref_" << go1->qref_ << "\n";
    //   // std::cout << "go1->diag_w" << go1->diag_w << "\n";
    //   // std::cout << "go1->diag_w_dt_" << go1->diag_w_dt_ << "\n";
    //   // std::cout << "go1->posErr_" << go1->posErr_ << "\n";
    //   // std::cout << "go1->uErr_" << go1->uErr_ << "\n";

    //   // if(std::isnan(this->pTarget12_) || std::isinf(this->pTarget12_)){

    //   // }


    //   std::cout << "rewards_.sum(): " << rewards_.sum() << "\n";

    // }
    // print_if_bad("vel_body_lin_z", vel_body_lin_z);
    // print_if_bad("vel_ang_xy", vel_ang_xy);
    // print_if_bad("pos_hips_des_err", pos_hips_des_err);
    // print_if_bad("pos_joint_off_limits_val", pos_joint_off_limits_val);
    // print_if_bad("action_rate", action_rate);
    // print_if_bad("sum_contact", sum_contact);
    // print_if_bad("time_in_the_air", time_in_the_air);
    // print_if_bad("joint_acc_curr", joint_acc_curr);


    float reward_total = rewards_.sum();

    // NOTE: rewards_.record() checks for NaNs, but it doesn't check for infs
    // RSFATAL_IF(std::isinf(reward_total), "reward_total is inf. Stopping ...");

    return reward_total;
  }


  inline void update_yaw(void){
    // NOTE: Ideally, this comes from IMUs
    this->yaw_cum += this->bodyAngularVel_[1] * this->control_dt_;
    // std::cout << "this->yaw_cum: " << this->yaw_cum << "\n";
  }



  template<typename T>
  inline bool print_if_bad(const std::string & var_name, const T & var){

    bool is_bad = false;
    if(std::isnan(var)){
      RSFATAL_IF(std::isnan(var), "reward_total is NaN. Stopping ...");
      std::cout << "Is NaN: " << var_name << ": " << var << "\n";
      is_bad = true;
    }

    if(std::isinf(var)){
      RSFATAL_IF(std::isinf(var), "reward_total is inf. Stopping ...");
      std::cout << "Is Inf: " << var_name << ": " << var << "\n\n";
      is_bad = true;
    }

    return is_bad;
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




  inline void parse_action_standard(Eigen::Ref<Eigen::VectorXd> action_des){
    assert(this->action_clipped.size() == 12);
    action_des = this->action_clipped;
    return;
  }


  inline void parse_action_coupled(Eigen::Ref<Eigen::VectorXd> action_des){

    assert(this->action_clipped.size() == 6);

    // std::cout << "[inside, before] action_des: " << action_des.transpose().format(this->clean_format) << "\n";

    // Couple:
    // Eigen::VectorXd action_des;
    // action_des.setZero(this->nJoints_);
    action_des[0] = this->action_clipped[0] / (2.*this->action_lim) * 0.1;
    action_des[1] = this->action_clipped[1];
    action_des[2] = this->action_clipped[2];

    action_des[9] = -this->action_clipped[0] / (2.*this->action_lim) * 0.1;
    action_des[10] = this->action_clipped[1];
    action_des[11] = this->action_clipped[2];

    action_des[3] = -this->action_clipped[3] / (2.*this->action_lim) * 0.1;
    action_des[4] = this->action_clipped[4];
    action_des[5] = this->action_clipped[5];

    action_des[6] = this->action_clipped[3] / (2.*this->action_lim) * 0.1;
    action_des[7] = this->action_clipped[4];
    action_des[8] = this->action_clipped[5];

    // std::cout << "[inside, after] action_des: " << action_des.transpose().format(this->clean_format) << "\n";
    // std::cout << "this->action_clipped: " << this->action_clipped.transpose().format(this->clean_format) << "\n";

    return;
  }

  inline void parse_action_coupled_no_lat_hip(Eigen::Ref<Eigen::VectorXd> action_des){

    // std::cout << "[inside, before] action_des: " << action_des.transpose().format(this->clean_format) << "\n";

    assert(this->action_clipped.size() == 4);

    // Couple:
    // Eigen::VectorXd action_des;
    // action_des.setZero(this->nJoints_);
    // 0.0136, -0.0118, 0.0105, -0.0102;
    action_des[0] = 0.0; // By setting zero here, we'll be applying just the initial position as desired joint position
    action_des[1] = this->action_clipped[0];
    action_des[2] = this->action_clipped[1];

    action_des[9] = 0.0;
    action_des[10] = this->action_clipped[0];
    action_des[11] = this->action_clipped[1];

    action_des[3] = 0.0;
    action_des[4] = this->action_clipped[2];
    action_des[5] = this->action_clipped[3];

    action_des[6] = 0.0;
    action_des[7] = this->action_clipped[2];
    action_des[8] = this->action_clipped[3];

    // std::cout << "[inside, after] action_des: " << action_des.transpose().format(this->clean_format) << "\n";
    // std::cout << "this->action_clipped: " << this->action_clipped.transpose().format(this->clean_format) << "\n";

    return;
  }

  inline void parse_action_cpg(Eigen::Ref<Eigen::VectorXd> action_des){

    assert(this->action_clipped.size() == 4);

    double freq_FR_1, ampl_FR_1, freq_FR_2, ampl_FR_2;

    // freq_FR_1 = abs(this->action_clipped[0]) / (this->action_lim) * 5.0; // max freq: 5 Hz
    // freq_FR_2 = abs(this->action_clipped[2])  / (this->action_lim) * 5.0; // max freq: 5 Hz
    // ampl_FR_1 = this->action_clipped[1] / (2.*this->action_lim) * 2.*M_PI;
    // ampl_FR_2 = this->action_clipped[3] / (2.*this->action_lim) * 2.*M_PI;


    freq_FR_1 = abs(this->action_clipped[0]);
    freq_FR_2 = abs(this->action_clipped[2]);
    ampl_FR_1 = this->action_clipped[1];
    ampl_FR_2 = this->action_clipped[3];

    // std::cout << "freq_FR_1: " << freq_FR_1 << " Hz\n";
    // std::cout << "ampl_FR_1: " << ampl_FR_1 << " rad\n";
    // std::cout << "freq_FR_2: " << freq_FR_2 << " Hz\n";
    // std::cout << "ampl_FR_2: " << ampl_FR_2 << " rad\n";

    action_des[0] = 0.0; // By setting zero here, we'll be applying just the initial position as desired joint position
    action_des[1] = ampl_FR_1*sin(freq_FR_1*2.*M_PI*this->time_counter);
    action_des[2] = ampl_FR_2*sin(freq_FR_2*2.*M_PI*this->time_counter);
    action_des[9] = 0.0;
    action_des[10] = ampl_FR_1*sin(freq_FR_1*2.*M_PI*this->time_counter);
    action_des[11] = ampl_FR_2*sin(freq_FR_2*2.*M_PI*this->time_counter);

    action_des[3] = 0.0;
    action_des[4] = ampl_FR_1*sin(freq_FR_1*2.*M_PI*this->time_counter + M_PI);
    action_des[5] = ampl_FR_2*sin(freq_FR_2*2.*M_PI*this->time_counter + M_PI);
    action_des[6] = 0.0;
    action_des[7] = ampl_FR_1*sin(freq_FR_1*2.*M_PI*this->time_counter + M_PI);
    action_des[8] = ampl_FR_2*sin(freq_FR_2*2.*M_PI*this->time_counter + M_PI);

    this->time_counter += this->getControlTimeStep();

    // std::cout << "[in parse_action_cpg] this->getControlTimeStep(): " << this->getControlTimeStep() << "\n";
    // std::cout << "[in parse_action_cpg] this->getSimulationTimeStep(): " << this->getSimulationTimeStep() << "\n";


    return;
  }


  // amarco: original step function (replaced by the above function; it does exactly the same)
  float step(const Eigen::Ref<EigenVec>& action) final {


    // Set external force:
    // We need a velocity of 0.5 m/s, which implies that the required acceleration is 0.5 / 0.002 = 250. (0.5 / 0.01 = 50; force of 560)
    // The robot's mass is about ~11.2kg. So, let's apply a force of 2800.0;
    // Gentle push during this->Nsteps_force_impulse steps
    if(this->counter_force < this->Nsteps_force_impulse){
      this->counter_force += 1;
      // std::cout << "increasing force counter; this->counter_force = " << this->counter_force << "\n";
    }
    else if(this->apply_impulse){ this->apply_impulse = false; }




    // if(this->first_time_force == true){

    //   go1->setExternalForce(go1->getBodyIdx("base"),this->external_force); // https://raisim.com/sections/ArticulatedSystem.html?highlight=external%20force#_CPPv4N6raisim17ArticulatedSystem16setExternalForceE6size_tRK3VecIXL3EEE

    //   // The external force is applied for a single time step only. You have to apply the force for every time step if you want persistent force
    //   if(this->counter_force > this->Nsteps_force_impulse){
    //     this->first_time_force = false;
    //   }

    //   this->counter_force += 1;
    // }

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

    // std::cout << "action: " << action.transpose().format(this->clean_format) << "\n";

    // Clip the action:
    // std::cout << "[before] this->action_clipped: " << this->action_clipped.transpose().format(this->clean_format) << "\n";
    this->action_clipped.cwiseMin(this->action_lim).cwiseMax(-this->action_lim);
    // std::cout << "[after] this->action_clipped: " << this->action_clipped.transpose().format(this->clean_format) << "\n";
    // std::cout << "After:\n";
    // std::cout << "action curr: " << this->action_curr.transpose().format(this->clean_format) << "\n";
    // std::cout << "action clipped: " << this->action_clipped.transpose().format(this->clean_format) << "\n";


    Eigen::VectorXd action_des;
    action_des.setZero(this->nJoints_);
    // std::cout << "[before] action_des: " << action_des.transpose().format(this->clean_format) << "\n";

    if(this->action_parametrization == STANDARD){
      this->parse_action_standard(action_des);
    }

    if(this->action_parametrization == COUPLED){
      this->parse_action_coupled(action_des);
    }

    if(this->action_parametrization == COUPLED_NO_LAT_HIP){
      this->parse_action_coupled_no_lat_hip(action_des);
    }

    if(this->action_parametrization == COUPLED_CPG){
      this->parse_action_cpg(action_des);
    }


    // this->parse_action(action_des); // Uses this->action_clipped
    // this->parse_action_cpg(action_des); // Uses this->action_clipped
    // std::cout << "[after] action_des: " << action_des.transpose().format(this->clean_format) << "\n";
    

    /// action scaling
    // pTarget12_ = action.cast<double>(); // original
    // pTarget12_ = this->action_clipped.cast<double>(); // amarco
    pTarget12_ = action_des.cast<double>(); // amarco
    // std::cout << "pTarget12_: " << pTarget12_.transpose().format(this->clean_format) << "\n";
    pTarget12_ = pTarget12_.cwiseProduct(actionStd_); // amarco: element-wise product; https://eigen.tuxfamily.org/dox/group__TutorialArrayClass.html
    // std::cout << "pTarget12_: " << pTarget12_.transpose().format(this->clean_format) << "\n";
    pTarget12_ += actionMean_;
    // std::cout << "pTarget12_: " << pTarget12_.transpose().format(this->clean_format) << "\n";
    pTarget_.tail(nJoints_) = pTarget12_;
    // std::cout << "pTarget_: " << pTarget_.transpose().format(this->clean_format) << "\n";

    // std::cout << "pTarget_: " << pTarget_.transpose().format(this->clean_format) << "\n";
    // std::cout << "vTarget_: " << vTarget_.transpose().format(this->clean_format) << "\n";

    go1->setPdTarget(pTarget_, vTarget_);
    // amarco NOTE: Even though it's inefficient to recompute this here as opposed to
    // computing it only once in the header, we can't do it in the header
    // because control_dt_ and simulation_dt_ are defined as some default values and
    // they will suffer changes once the VectorizedEnvironment functions are called
    integration_steps_ = int(control_dt_ / simulation_dt_ + 1e-10);
    // integration_steps_ = 1;
    
    // std::cout << "integration_steps_: " + std::to_string(integration_steps_) + "\n";
    // std::cout << "control_dt_: " + std::to_string(control_dt_) + "\n";
    // std::cout << "simulation_dt_: " + std::to_string(simulation_dt_) + "\n";

    for(int i=0; i< integration_steps_; i++){
      if(server_) server_->lockVisualizationServerMutex();
      if(this->apply_impulse){ go1->setExternalForce(go1->getBodyIdx("base"),this->external_force); } // https://raisim.com/sections/ArticulatedSystem.html?highlight=external%20force#_CPPv4N6raisim17ArticulatedSystem16setExternalForceE6size_tRK3VecIXL3EEE
      world_->integrate(); // https://raisim.com/sections/WorldSystem.html?highlight=integrate#_CPPv4N6raisim5World9integrateEv
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
  std::vector<int> foot_in_contact_curr;
  std::vector<int> foot_in_contact_last;
  std::vector<size_t> footIndices_mine;
  double time_good_contacts;
  double time_bad_contacts;
  double time_counter;
  Eigen::VectorXd joint_vel_curr;
  Eigen::VectorXd joint_acc_curr;
  Eigen::VectorXd joint_vel_last;
  bool first_time;
  VecDyn generalized_force;
  int action_parametrization;
  int counter_force;
  int Nsteps_force_impulse;
  float height_body_desired;
  Eigen::VectorXd external_force;
  double yaw_cum;
  bool apply_impulse;


  /// these variables are not in use. They are placed to show you how to create a random number sampler.
  std::normal_distribution<double> normDist_;
  thread_local static std::mt19937 gen_;
};
thread_local std::mt19937 raisim::ENVIRONMENT::gen_;

}

