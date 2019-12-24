
#include "uav_ddp.hpp"

UavDDPNode::UavDDPNode()
{
    // DDP solver related
    uav_params_ = boost::make_shared<crocoddyl::UavUamParams>(0.13, 0.22, 6.6e-5, 1e-6, 'x', 5, 0.1);  
    pinocchio::urdf::buildModel(EXAMPLE_ROBOT_DATA_MODEL_DIR "/hector_description/robots/quadrotor_base.urdf", pinocchio::JointModelFreeFlyer(), uav_model_);
    
    
    x0_ = Eigen::VectorXd::Zero(uav_model_.nv*2 + 1);
    bl_frameid_ = uav_model_.getFrameId("base_link");
    dt_ = 2e-2;
    
    Eigen::Vector3d tpos;
    tpos << 0,0,1;
    Eigen::Quaterniond tquat(1,0,0,0);
    Eigen::Vector3d zero_vel = Eigen::Vector3d::Zero();
    wp_ = boost::make_shared<crocoddyl::WayPoint>(100, tpos, tquat, zero_vel, zero_vel);

    nav_problem_ = boost::make_shared<crocoddyl::SimpleUavUamGotoProblem>(uav_model_, uav_params_);

    // Publishers and subscribers
    sb_pose_ = nh_.subscribe("mavros/local_position/pose", 1, &UavDDPNode::callbackPose, this);
    sb_twist_ = nh_.subscribe("mavros/local_position/velocity_body", 1, &UavDDPNode::callbackTwist, this);
    
}

UavDDPNode::~UavDDPNode(){}

void UavDDPNode::updateDDPProblem()
{
    ddp_problem_ = nav_problem_->createProblem(x0_, *wp_.get(), dt_, bl_frameid_);
}


void UavDDPNode::callbackPose(const geometry_msgs::PoseStamped::ConstPtr& msg_pose)
{
    geometry_msgs::Pose pose = msg_pose->pose; 
    Eigen::VectorXd x0_pose(7);
    
    x0_pose << pose.position.x, pose.position.y, pose.position.z, pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w;
    
    x0_.head(7) = x0_pose; 

    // std::cout << "New position:" << std::endl << x0_ << std::endl;
}

void UavDDPNode::callbackTwist(const geometry_msgs::TwistStamped::ConstPtr& msg_twist)
{
    geometry_msgs::Twist twist = msg_twist->twist;
    Eigen::VectorXd x0_twist(6);

    x0_twist << twist.linear.x, twist.linear.y, twist.linear.z, twist.angular.x, twist.angular.y, twist.angular.z; 

    x0_.tail(6) = x0_twist;

    // std::cout << "New position:" << std::endl << x0_ << std::endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, ros::this_node::getName());
    UavDDPNode croco_node;
    ros::Rate loop_rate(100);


    while (ros::ok())
    {
        
        croco_node.updateDDPProblem();
        crocoddyl::SolverFDDP(croco_node.ddp_problem_);
        
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}