/**************************
 *  Libraries includes    *
 **************************/
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/math/quaternion.hpp>
#include <example-robot-data/path.hpp>
#include <crocoddyl/multibody/actuations/uav-uam.hpp>
#include <crocoddyl/multibody/utils/uavuam-goto.hpp>
#include <crocoddyl/core/solvers/fddp.hpp>
#include <crocoddyl/core/utils/callbacks.hpp>


/**************************
 *      STD includes      *
 **************************/
#include <iostream>
#include <mutex>
/**************************
 *      ROS includes      *
 **************************/
#include <ros/ros.h>
#include <ros/subscribe_options.h>
#include <ros/callback_queue.h>
#include <ros/spinner.h>
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
    
    // Multithrad
    // ros::CallbackQueue sb_pose_queue_;
    // ros::SubscribeOptions sb_pose_opt_;
    // ros::CallbackQueue sb_twist_queue_;
    // ros::SubscribeOptions sb_twist_opt_;

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
    boost::shared_ptr<crocoddyl::SolverFDDP> fddp_;
    // Solver Callbacks
    std::vector<boost::shared_ptr<crocoddyl::CallbackAbstract> > fddp_cbs_;

    // std::mutex mut_pose0_;
    // std::mutex mut_twist0_;
    Eigen::VectorXd x0_;
    Eigen::Quaterniond q0_;
    std::vector<Eigen::VectorXd> x_traj_;
    std::vector<Eigen::VectorXd> u_traj_;
    uav_oc_msgs::UAVOptCtlPolicy policy_msg_;


    public: //methods
    UavDDPNode();
    ~UavDDPNode();
    void fillUavParams();
    void initializeDDP();   

    void callbackPose(const geometry_msgs::PoseStamped::ConstPtr& msg_pose);
    void callbackTwist(const geometry_msgs::TwistStamped::ConstPtr& msg_twist);
    void publishControls();
    
};