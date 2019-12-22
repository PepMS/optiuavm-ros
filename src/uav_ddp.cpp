
#include "uav_ddp.hpp"

#include <pinocchio/parsers/urdf.hpp>

UavDDPNode::UavDDPNode()
{
    pinocchio::urdf::buildModel(EXAMPLE_ROBOT_DATA_MODEL_DIR "/hector_description/robots/quadrotor_base.urdf", pinocchio::JointModelFreeFlyer(), _model);
    
    
}

UavDDPNode::~UavDDPNode()
{

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, ros::this_node::getName());

    return 0;
}