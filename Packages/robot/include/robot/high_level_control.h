/** 
 * @file high_level_control.h
 * @brief This file defines the HighLevelControl clas
 *
 * @author Atabak Hafeez [atabakhafeez]
 * @author Maria Ficiu [MariaFiciu]
 * @author Rubin Deliallisi [rdeliallisi]
 * @author Siddharth Shukla [thunderboltsid]
 * @bug No known bugs.
 */

#ifndef HIGH_LEVEL_CONTROL_H
#define HIGH_LEVEL_CONTROL_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

/**
 * @brief Breaks down abstract movement commands into smaller ones, that can be
 * executed by the Low Level Control
 *
 * Usage:
 *     HighLevelControl high_level_control;
 *     high_level_control.FollowWallMove();
 *
 * @author Rubin Deliallisi (rdeliallisi)
 * @date March 2016
 */

class HighLevelControl {

private:

    // TODO(rdeliallisi): To be put into Low Level Control
    /**
     * @brief Ros node to which the class is attached
    */
    ros::NodeHandle node_;

    // TODO(rdeliallisi): To be put into Low Level Control
    /**
     * @brief Used to send messages to the actual robot
     */
    ros::Publisher cmd_vel_pub_;

    // TODO(rdeliallisi): To be put into Object Detection
    /**
     * @brief Used to get the data from the laser range finder
     */
    ros::Subscriber laser_sub_;

    /**
     * @brief The type of turn the robot is currently in when turning.
     */
    int turn_type_;

    /**
     * @brief Minimum proximity distance that the robot can have from a wall
     */
    double security_distance_;

    /**
     * @brief Maximum distance the robot can be away from the anchor wall
     */
    double wall_follow_distance_;

    /**
     * @brief Linear velocity for a single robot movement
     */
    double linear_velocity_;

    /**
     * @brief Angular velocity for a single robot movement
     */
    double angular_velocity_;

    /**
     * @brief Can the robot continue walking in a straight line or not
     */
    bool can_continue_;

    /**
     * @brief Is the robot close enough to the anchor wall
     */
    bool is_close_to_wall_;

    /**
     * @brief Is the robot anchored to a wall or not
     */
    bool is_following_wall_;

    /**
     * @brief When the robot is in controled random walk this variable specifies
     * if it is turning or not
     */
    bool is_turning_;

    // TODO(rdeliallisi): To be put into Object Detection
    /**
     * @brief Gets the data from the laser range finder, examines them and
     * updates the relevant class variables
     * @param msg Raw data comming from the laser range finder
     */
    void LaserCallback(const sensor_msgs::LaserScan::ConstPtr& msg);

    //TODO(rdeliallisi): To be put into Low Level Control
    /**
     * @brief Send the movement command to the robot using ROS nodes and topics
     * @param linear_speed  Linear Speed that the robot should move with
     * @param angular_speed Angular Speed that the robot should rotate with
     */
    void Move(double linear_speed, double angular_speed);

public:
    HighLevelControl();

    /**
     * @brief Change the minimum distance the robot can be close to the wall to
     * @param security_distance New security distance of the robot
     */
    void set_security_distance(double security_distance);

    /**
     * @brief Change the angular velocity of the robot
     * @param angular_velocity New angular velocity of the robot
     */
    void set_angular_velocity(double angular_velocity);

    /**
     * @brief Change the angular velocity of the robot
     * @param linear_velocity New angular velocity of the robot
     */
    void set_linear_velocity(double linear_velocity);


    /**
     * @brief Override method for testing purposes
     * @param turn_type Decides the type of the wall the robot is following.
     * 1 for right wall, -1 for left wall
     */
    void set_turn_type(int turn_type);

    /**
     * @brief Checks if the robot can continue waking forward or not
     * @return Returns the value of can_continue_
     */
    bool can_continue() {
        return can_continue_;
    }

    /**
     * @brief Checks if the robot is close enough to the wall it is anchored to 
     * @return Returns the value of is_close_to_wall_
     */
    bool is_close_to_wall() {
        return is_close_to_wall_;
    }

    /**
     * @brief Checks if the robot is anchored and following a specific wall
     * @return Returns the value of is_following_wall_
     */
    bool is_following_wall() {
        return is_following_wall_;
    }

    /**
     * @brief Checks if the robot is in the middle of a turning process
     * @return Returns the value of is_turning_
     */
    bool is_turning() {
        return is_turning_;
    }

    /**
     * @brief Moves the robot so that it always follows a wall
     */
    void WallFollowMove();

    /**
     * @brief Moves the robot randomly using the specified linear and
     * angular velocity. The robot will chose a direction to turn and
     * turn in that direction until it completes the turn.
     */
    void ControlledRandomMove();

    /**
     * @brief Turns the robot a random amount of degrees in a random direction 
     * each time an obstacle is encountered.
     */
    void TotalRandomMove();
};

#endif