
#include "uav_ddp.hpp"

UavDDPNode::UavDDPNode()
{
    // Initialize specific classes needed in the DDP solver
    initializeDDP();
    
    // Publishers and subscribers
    sb_pose_ = nh_.subscribe("/mavros/local_position/pose", 1, &UavDDPNode::callbackPose, this);
    sb_twist_ = nh_.subscribe("/mavros/local_position/velocity_body", 1, &UavDDPNode::callbackTwist, this);
    pub_policy_ = nh_.advertise<uav_oc_msgs::UAVOptCtlPolicy>("/optctl/actuator_control", 10);    
}

UavDDPNode::~UavDDPNode(){}

void UavDDPNode::initializeDDP()
{
    // Load URDF Model
    pinocchio::urdf::buildModel(EXAMPLE_ROBOT_DATA_MODEL_DIR "/iris_description/robots/iris_simple.urdf",  pinocchio::JointModelFreeFlyer(), uav_model_);

    // Load UAV params
    uav_params_ = boost::make_shared<crocoddyl::UavUamParams>();
    fillUavParams();

    // Frame we want in the cost function
    bl_frameid_ = uav_model_.getFrameId("iris__base_link");

    // x0_:Set an initial pose (should be read from topic)
    x0_ = Eigen::VectorXd::Zero(uav_model_.nv*2 + 1);
    x0_(6) = 1.0;
    
    // This may be problematic as tpos and tquat are passed by reference so when the constructor ends, it may destruct both variables.
    Eigen::Vector3d tpos;
    tpos << 0,0,1;
    Eigen::Quaterniond tquat(1,0,0,0);
    Eigen::Vector3d zero_vel = Eigen::Vector3d::Zero();
    wp_ = boost::make_shared<crocoddyl::WayPoint>(50, tpos, tquat, zero_vel, zero_vel);

    // Navigation problem
    nav_problem_ = boost::make_shared<crocoddyl::SimpleUavUamGotoProblem>(uav_model_, uav_params_);

    // Shooting problem
    ddp_problem_ = nav_problem_->createProblem(x0_, *wp_.get(), 2e-2, bl_frameid_);

    // Solver callbacks
    fddp_cbs_.push_back(boost::make_shared<crocoddyl::CallbackVerbose>());

    // Setting the solver
    fddp_ = boost::make_shared<crocoddyl::SolverFDDP>(ddp_problem_);      
    // fddp_->setCallbacks(fddp_cbs_);
    fddp_->solve();
    fddp_->solve(fddp_->get_xs(), fddp_->get_us());
}

void UavDDPNode::fillUavParams()
{
    if (! nh_.hasParam("multirotor"))
    {
        ROS_ERROR("Fill UAV params: multirotor parameter not found. Please, load the YAML file with the multirotor description.");
    }     
    
    if (!nh_.getParam("multirotor/cf", uav_params_->cf_))
        ROS_ERROR("Fill UAV params: please, specify the thrust coefficient for the propellers (cf).");
    
    if (!nh_.getParam("multirotor/cm", uav_params_->cm_))
        ROS_ERROR("Fill UAV params: please, specify the torque coefficient for the propellers (cm).");
    
    if (!nh_.getParam("multirotor/max_thrust", uav_params_->max_thrust_))
        ROS_ERROR("Fill UAV params: please, specify the max lift force (max_thrust) that a rotor can produce.");
    
    if (!nh_.getParam("multirotor/min_thrust", uav_params_->min_thrust_))
        ROS_ERROR("Fill UAV params: please, specify the min lift force (max_thrust) that a rotor can produce.");

    XmlRpc::XmlRpcValue rotors;
    if (!nh_.getParam("multirotor/rotors", rotors))
        ROS_ERROR("Fill UAV params: please, specify the position of the rotors.");
    
    if (rotors.size() < 4)
        ROS_ERROR("Please, specify more than 3 rotors. This node is intended for planar multirotors.");

    uav_params_->n_rotors_ = rotors.size();
    Eigen::MatrixXd S = Eigen::MatrixXd::Zero(6, rotors.size());

    for (int32_t i = 0; i < rotors.size(); ++i)
    {
        XmlRpc::XmlRpcValue rotor = rotors[i];
        double x = rotor["x"]; 
        double y = rotor["y"]; 
        double z = rotor["z"]; 

        S(2, i) = 1.0; // Thrust
        S(3, i) = y;   // Mx 
        S(4, i) = -x;  // My 
        S(5, i) = z*uav_params_->cm_/uav_params_->cf_; // Mz
    }
    uav_params_->tau_f_ = S;
}

void UavDDPNode::callbackPose(const geometry_msgs::PoseStamped::ConstPtr& msg_pose)
{
    geometry_msgs::Pose pose = msg_pose->pose; 
    Eigen::Quaterniond quat0(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
    quat0.normalize();
    
    x0_.head(3) << pose.position.x, pose.position.y, pose.position.z;
    x0_.segment(3,4) << quat0.x(), quat0.y(), quat0.z(), quat0.w();
}

void UavDDPNode::callbackTwist(const geometry_msgs::TwistStamped::ConstPtr& msg_twist)
{
    geometry_msgs::Twist twist = msg_twist->twist;
    Eigen::VectorXd x0_twist(6);

    x0_twist << twist.linear.x, twist.linear.y, twist.linear.z, twist.angular.x, twist.angular.y, twist.angular.z; 

    x0_.tail(6) = x0_twist;
}

void UavDDPNode::publishControls()
{
    uav_oc_msgs::UAVOptCtlPolicy policy_msg;
    
    // Header
    policy_msg.header.stamp = ros::Time::now();

    // U desired for the current node
    policy_msg.u_desired = std::vector<float>(uav_params_->n_rotors_);
    policy_msg.u_desired[0] = nav_problem_->actuation_->get_generalizedTorque(fddp_->get_us()[0])[3];
    policy_msg.u_desired[1] = nav_problem_->actuation_->get_generalizedTorque(fddp_->get_us()[0])[4];
    policy_msg.u_desired[2] = nav_problem_->actuation_->get_generalizedTorque(fddp_->get_us()[0])[5];
    policy_msg.u_desired[3] = nav_problem_->actuation_->get_generalizedTorque(fddp_->get_us()[0])[2];    
    
    // State desired for the current node
    int ndx = fddp_->get_xs()[0].size();
    policy_msg.x_desired = std::vector<float>(ndx);
    for (int ii = 0; ii < ndx; ++ii)
    {
        policy_msg.x_desired[ii] = fddp_->get_xs()[0][ii];
    }

    policy_msg.ffterm.mx = fddp_->get_k()[0][0];
    policy_msg.ffterm.my = fddp_->get_k()[0][1];
    policy_msg.ffterm.mz = fddp_->get_k()[0][2];
    policy_msg.ffterm.th = fddp_->get_k()[0][3];

    pub_policy_.publish(policy_msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, ros::this_node::getName());
    UavDDPNode croco_node;
    ros::Rate loop_rate(500);


    while (ros::ok())
    {    
        croco_node.ddp_problem_->set_x0(croco_node.x0_);

        croco_node.fddp_->solve(croco_node.fddp_->get_xs(), croco_node.fddp_->get_us());

        croco_node.publishControls();

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}