/**************************
 *  Libraries includes    *
 **************************/
#include <iostream>
#include <pinocchio/parsers/urdf.hpp>
// #include <pinocchio/algorithm/joint-configuration.hpp>
#include <example-robot-data/path.hpp>

/**************************
 *      STD includes      *
 **************************/

/**************************
 *      ROS includes      *
 **************************/
#include <ros/ros.h>





class UavDDPNode 
{
    public: //attributes

    ros::NodeHandle _nh;

    pinocchio::Model _model;


    public: //methods
    
    UavDDPNode();

    ~UavDDPNode();
};