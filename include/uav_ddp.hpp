/**************************
 *  Libraries includes    *
 **************************/
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/math/quaternion.hpp>
#include <example-robot-data/path.hpp>
#include <crocoddyl/multibody/actuations/uav-uam.hpp>
#include <crocoddyl/multibody/utils/uavuam-goto.hpp>
#include <crocoddyl/core/solvers/fddp.hpp>

/**************************
 *      STD includes      *
 **************************/
#include <iostream>
/**************************
 *      ROS includes      *
 **************************/
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/ActuatorControl.h>
#include <mavros_msgs/HilActuatorControls.h>
#include <uav_oc_msgs/UAVOptCtlPolicy.h>

class UavDDPNode 
{
    public: //attributes

    // ROS related
    ros::NodeHandle nh_;
    ros::Subscriber sb_pose_;
    ros::Subscriber sb_twist_;
    ros::Publisher pub_policy_;

    // Crocoddyl related
    int bl_frameid_;
    bool dt_;
    pinocchio::Model uav_model_;
    boost::shared_ptr<crocoddyl::UavUamParams> uav_params_;
    boost::shared_ptr<crocoddyl::WayPoint> wp_;
    boost::shared_ptr<crocoddyl::SimpleUavUamGotoProblem> nav_problem_;
    boost::shared_ptr<crocoddyl::ShootingProblem> ddp_problem_;

    Eigen::VectorXd x0_;
    std::vector<Eigen::VectorXd> x_traj_;
    std::vector<Eigen::VectorXd> u_traj_;
    

    public: //methods
    UavDDPNode();
    ~UavDDPNode();
    void updateDDPProblem();

    void callbackPose(const geometry_msgs::PoseStamped::ConstPtr& msg_pose);
    void callbackTwist(const geometry_msgs::TwistStamped::ConstPtr& msg_twist);
    void publishControls();
    
};