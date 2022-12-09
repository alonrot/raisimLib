// This file is part of RaiSim. You must obtain a valid license from RaiSim Tech
// Inc. prior to usage.

#include "raisim/RaisimServer.hpp"
#include "raisim/World.hpp"
#if WIN32
#include <timeapi.h>
#endif

void trying_out_some_stuff(raisim::ArticulatedSystem * aliengo, raisim::World & world_){

  /// Let's check all contact impulses acting on "LF_SHANK"
  auto footIndex_FR_calf = aliengo->getBodyIdx("FR_calf");
  auto footIndex_FL_calf = aliengo->getBodyIdx("FL_calf");
  auto footIndex_RR_calf = aliengo->getBodyIdx("RR_calf");
  auto footIndex_RL_calf = aliengo->getBodyIdx("RL_calf");
  // std::cout << "footIndex_FL_calf: " << footIndex_FL_calf << "\n";

  std::cout << "footIndex_FR_calf: " << footIndex_FR_calf << "\n";
  std::cout << "footIndex_FL_calf: " << footIndex_FL_calf << "\n";
  std::cout << "footIndex_RR_calf: " << footIndex_RR_calf << "\n";
  std::cout << "footIndex_RL_calf: " << footIndex_RL_calf << "\n";

  /// for all contacts on the robot, check ...
  for(auto& contact: aliengo->getContacts()) {
    
    if (contact.skip())
      continue; /// if the contact is internal, one contact point is set to 'skip'

    // Print indices:
    std::cout << "contact.getlocalBodyIndex(): " << contact.getlocalBodyIndex() << "\n";

    // std::cout << "aliengo->getContacts().size(): " << aliengo->getContacts().size() << "\n";
    
    if ( footIndex_FL_calf == contact.getlocalBodyIndex() ) {

      std::cout << "Contact made!\n";
    
      // std::cout << "Contact impulse in the contact frame, size: " << contact.getImpulse().e().size() << std::endl;
      // std::cout << "Contact impulse in the contact frame: " << contact.getImpulse().e() << std::endl;
      /// the impulse is acting from objectB to objectA. You can check if this object is objectA or B by
    
      // std::cout << "is ObjectA: " << contact.isObjectA() << std::endl;
      // std::cout << "Contact frame: \n" << contact.getContactFrame().e().transpose() << std::endl;
      /// contact frame is transposed.
    
      // std::cout << "Contact impulse in the world frame: " << contact.getContactFrame().e().transpose() * contact.getImpulse().e() << std::endl;
      // std::cout << "Contact Normal in the world frame: " << contact.getNormal().e().transpose() << std::endl;
      // std::cout << "Contact position in the world frame: " << contact.getPosition().e().transpose() << std::endl;
      // std::cout << "It collides with: " << world_.getObject(contact.getPairObjectIndex()) << std::endl;
    
      // if (contact.getPairContactIndexInPairObject() != raisim::BodyType::STATIC) {
      //   /// Static objects do not have Contacts store. So you must check if the pair object is static
      //   /// This saves computation in raisim
      //   world_.getObject(contact.getPairObjectIndex())->getContacts(); /// You can use the same methods on the pair object
      // }
    
    
    }
  }

}

int detect_contact(raisim::ArticulatedSystem * aliengo){

  std::vector<int> foot_in_contact = {0, 0, 0, 0};
  std::vector<int> footIndices_mine = {3, 6, 9, 12};


  // Initialize:
  for(int ii=0; ii < foot_in_contact.size(); ii++){
    foot_in_contact[ii] = 0;
  }

  /// for all contacts on the robot, check ...
  for(auto& contact: aliengo->getContacts()) {
    
    if (contact.skip())
      continue; /// if the contact is internal, one contact point is set to 'skip'
    
    for(int ii=0; ii < footIndices_mine.size(); ii++){

      if(footIndices_mine[ii] == contact.getlocalBodyIndex()){
        foot_in_contact[ii] = 1;
      }

    }

  }


  int sum_contact = 0;
  std::cout << "foot_in_contact:\n";
  for(int ii=0; ii < foot_in_contact.size(); ii++){
    std::cout << foot_in_contact[ii] << ", ";
    sum_contact += foot_in_contact[ii];
  }
  std::cout << "\n";

  std::cout << "sum_contact: " << sum_contact << "\n";

  double time_in_the_air = 0.0;
  if(foot_in_contact[0] != foot_in_contact[1]){
    time_in_the_air = 0.002;
  }

  std::cout << "time_in_the_air: " << time_in_the_air << "\n";


  return sum_contact;

}




int main(int argc, char* argv[]) {
  auto binaryPath = raisim::Path::setFromArgv(argv[0]);
  raisim::World::setActivationKey(binaryPath.getDirectory() + "\\rsc\\activation.raisim");

  /// create raisim world
  raisim::World world;
  world.setTimeStep(0.001);

  /// create objects
  auto ground = world.addGround();
  ground->setAppearance("steel");
  auto aliengo = world.addArticulatedSystem(binaryPath.getDirectory() + "\\rsc\\aliengo\\aliengo.urdf");

  /// aliengo joint PD controller
  Eigen::VectorXd jointNominalConfig(aliengo->getGeneralizedCoordinateDim()), jointVelocityTarget(aliengo->getDOF());
  // jointNominalConfig << 0, 0, 0.54, 1.0, 0.0, 0.0, 0.0, 0.03, 0.4, -0.8, -0.03, 0.4, -0.8, 0.03, -0.4, 0.8, -0.03, -0.4, 0.8;
  jointNominalConfig << 0, 0, 0.54, 1.0, 0.0, 0.0, 0.0, 0.03, 0.4, -1.8, -0.03, 0.4, -0.8, 0.03, -0.4, 0.8, -0.03, -0.4, 0.8;
  jointVelocityTarget.setZero();

  Eigen::VectorXd jointPgain(aliengo->getDOF()), jointDgain(aliengo->getDOF());
  jointPgain.tail(12).setConstant(100.0);
  jointDgain.tail(12).setConstant(1.0);

  aliengo->setGeneralizedCoordinate(jointNominalConfig);
  aliengo->setGeneralizedForce(Eigen::VectorXd::Zero(aliengo->getDOF()));
  aliengo->setPdGains(jointPgain, jointDgain);
  aliengo->setPdTarget(jointNominalConfig, jointVelocityTarget);
  aliengo->setName("aliengo");

  /// launch raisim server
  raisim::RaisimServer server(&world);
  server.focusOn(aliengo);
  server.launchServer();

  for (int i=0; i<2000000; i++) {
    raisim::MSLEEP(50);
    server.integrateWorldThreadSafe();

    // trying_out_some_stuff(aliengo,world);
    detect_contact(aliengo);

  }

  std::cout<<"total mass "<<aliengo->getCompositeMass()[0]<<std::endl;

  server.killServer();
}
