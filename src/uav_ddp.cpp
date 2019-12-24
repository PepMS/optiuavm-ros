
#include "uav_ddp.hpp"

UavDDPNode::UavDDPNode()
{
    // DDP solver related
    uav_params_ = boost::make_shared<crocoddyl::UavUamParams>(0.13, 0.22, 6.6e-5, 1e-6, 'x', 5, 0.1);  
    pinocchio::urdf::buildModel(EXAMPLE_ROBOT_DATA_MODEL_DIR "/hector_description/robots/quadrotor_base.urdf", pinocchio::JointModelFreeFlyer(), uav_model_);
    
    Eigen::VectorXd x0;
    x0 = Eigen::VectorXd::Zero(uav_model_.nv*2 + 1);
    x0(6) = 1.0;

    bl_frameid_ = uav_model_.getFrameId("base_link");
    
    Eigen::Vector3d tpos;
    tpos << 0,0,1;
    Eigen::Quaterniond tquat(1,0,0,0); 
    int knots = 70;
    Eigen::Vector3d zero_vel = Eigen::Vector3d::Zero();
    crocoddyl::WayPoint wp(knots, tpos, tquat, zero_vel, zero_vel);

    nav_problem_ = boost::make_shared<crocoddyl::SimpleUavUamGotoProblem>(uav_model_, uav_params_);
    ddp_problem_ = nav_problem_->createProblem(x0, wp, 2e-2, bl_frameid_);

    // Publishers and subscribers
    sb_pose_ = nh_.subscribe("mavros/local_position/pose", 1, &UavDDPNode::callbackPose, this);
    
}

UavDDPNode::~UavDDPNode(){}

void UavDDPNode::callbackPose(const geometry_msgs::PoseStamped::ConstPtr& msg_pose)
{
    
}

void UavDDPNode::callbackTwist(const geometry_msgs::TwistStamped::ConstPtr& msg_twist)
{
    
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, ros::this_node::getName());
    UavDDPNode croco_node;
    ros::Rate loop_rate(100);


    while (ros::ok())
    {
        crocoddyl::SolverFDDP fddp(croco_node.ddp_problem_);

        ros::spinOnce();

        loop_rate.sleep();
    }
    return 0;
}