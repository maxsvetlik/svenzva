#include <kdl_parser/kdl_parser.hpp>
#include <ros/ros.h>
#include <ros/package.h>

#include <kdl/frames.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainidsolver_recursive_newton_euler.hpp>


int main(){

   KDL::Tree my_tree;
   std::string path = ros::package::getPath("svenzva_description");
   std::string full_path = path + "/robots/svenzva_arm.urdf";
   ROS_INFO("Loading model from %s", full_path.c_str());
   kdl_parser::treeFromFile(full_path, my_tree);
   if (!kdl_parser::treeFromFile(full_path, my_tree)){
      ROS_ERROR("Failed to construct kdl tree");
      return false;
   }
   KDL::Chain chain;
   my_tree.getChain("base_link", "link_6", chain);

   int mNumJnts = 6;
   // Get some joint pos, vel, acc values
    KDL::JntArray jnt_q(mNumJnts);
    KDL::JntArray jnt_qd(mNumJnts);
    KDL::JntArray jnt_qdd(mNumJnts);
    KDL::JntArray jnt_taugc(mNumJnts);
    KDL::Wrenches jnt_wrenches;
    for (unsigned int i = 0; i < mNumJnts; i++) {
      jnt_q(i) = 0.0;
      jnt_qd(i) = 0.0;
      jnt_qdd(i) = 0.0;
      jnt_wrenches.push_back(KDL::Wrench());
    }
    jnt_q(1) = 1.570;

    // Kinematics 
    KDL::ChainFkSolverPos_recursive fkSolver = KDL::ChainFkSolverPos_recursive(chain);
    KDL::Frame fkKDL;
    fkSolver.JntToCart(jnt_q, fkKDL);

    // Compute Dynamics 
    KDL::Vector gravity(0.0, 0.0, -9.81);
    KDL::ChainIdSolver_RNE gcSolver = KDL::ChainIdSolver_RNE(chain, gravity);
    int ret = gcSolver.CartToJnt(jnt_q, jnt_qd, jnt_qdd, jnt_wrenches,jnt_taugc);
    if (ret < 0) ROS_ERROR("KDL: inverse dynamics ERROR");

    for( int i = 0; i < mNumJnts; i++)
        ROS_INFO("Joint %d got %f", i, jnt_taugc(i));


}
