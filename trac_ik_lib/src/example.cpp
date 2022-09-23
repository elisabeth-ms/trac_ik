#include <trac_ik/trac_ik.hpp>
#include <kdl/chain.hpp>
 #include <kdl/jntarray.hpp>
// #include <kdl/chainfksolverpos_recursive.hpp>
// #include <kdl/chainiksolvervel_pinv.hpp>
// #include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/utilities/utility.h>
#include <iostream>

#define NJoints 8
static KDL::Chain makeTeoTrunkAndRightArmKinematicsFromDH()
{
    const KDL::Joint rotZ(KDL::Joint::RotZ);
    const KDL::Joint fixed(KDL::Joint::None); // Rigid Connection
    KDL::Chain chain;
    chain.addSegment(KDL::Segment(rotZ, KDL::Frame::DH(0.0, -KDL::PI / 2, 0.1932, 0.0)));
    chain.addSegment(KDL::Segment(rotZ, KDL::Frame::DH(0.305, 0.0, -0.34692, -KDL::PI / 2)));
    chain.addSegment(KDL::Segment(rotZ, KDL::Frame::DH(0, -KDL::PI / 2, 0, 0)));
    chain.addSegment(KDL::Segment(rotZ, KDL::Frame::DH(0, -KDL::PI / 2, 0, -KDL::PI / 2)));
    chain.addSegment(KDL::Segment(rotZ, KDL::Frame::DH(0, -KDL::PI / 2, -0.32901, -KDL::PI / 2)));
    chain.addSegment(KDL::Segment(rotZ, KDL::Frame::DH(0, KDL::PI / 2, 0, 0)));
    chain.addSegment(KDL::Segment(rotZ, KDL::Frame::DH(0, -KDL::PI / 2, -0.215, 0)));
    chain.addSegment(KDL::Segment(rotZ, KDL::Frame::DH(-0.09, 0, 0, -KDL::PI / 2)));
    chain.addSegment(KDL::Segment(fixed, KDL::Frame::DH(0, KDL::PI / 2, 0, -KDL::PI / 2)));
    chain.addSegment(KDL::Segment(fixed, KDL::Frame::DH(0, 0, 0.0975, 0)));


    return chain;
}

void makeQLimitsTeoTrunkAndRightArmKinematics(KDL::JntArray &qmin, KDL::JntArray &qmax)
{
    qmin.resize(NJoints);
    qmax.resize(NJoints);
    qmin(0) = -31.0* KDL::deg2rad;
    qmin(1) = -10.1* KDL::deg2rad;
    qmin(2) = -98.1* KDL::deg2rad;
    qmin(3) = -75.5* KDL::deg2rad;
    qmin(4) = -80.1* KDL::deg2rad;
    qmin(5) = -99.6* KDL::deg2rad;
    qmin(6) = -80.4* KDL::deg2rad;
    qmin(7) = -115.1* KDL::deg2rad;
    qmax(0) = 31.0* KDL::deg2rad;
    qmax(1) = 25.5* KDL::deg2rad;
    qmax(2) = 106.0* KDL::deg2rad;
    qmax(3) = 22.4* KDL::deg2rad;
    qmax(4) = 57.0* KDL::deg2rad;
    qmax(5) = 98.4* KDL::deg2rad;
    qmax(6) = 99.6* KDL::deg2rad;
    qmax(7) = 44.7* KDL::deg2rad;
}


int main(int argc, char** argv){

    KDL::JntArray qmin(NJoints), qmax(NJoints);
    makeQLimitsTeoTrunkAndRightArmKinematics(qmin, qmax);
    KDL::Chain chain = makeTeoTrunkAndRightArmKinematicsFromDH();
    KDL::ChainFkSolverPos_recursive  fksolver(chain);

    KDL::Frame desired_end_effector_pose;
    KDL::JntArray qTest(NJoints);
    for(unsigned int j=0; j<NJoints; j++)
        qTest(j) = (qmin(j)+qmax(j))/2.0;
    fksolver.JntToCart(qTest, desired_end_effector_pose);


    TRAC_IK::TRAC_IK ik_solver(chain, qmin, qmax, 0.005, 1e-5,TRAC_IK::Speed); 

    KDL::JntArray joint_seed(NJoints);
    KDL::JntArray return_joints(NJoints);
    int rc = ik_solver.CartToJnt(joint_seed, desired_end_effector_pose, return_joints);
    std::cout<<return_joints(0)<<" "<<return_joints(1)<<" "<<return_joints(2)<<" "<<return_joints(3)<<" "<<return_joints(4)<<" "<<return_joints(5)<<" "<<return_joints(6)<<" "<<return_joints(7)<<std::endl;
    std::cout<<rc<<std::endl;
}