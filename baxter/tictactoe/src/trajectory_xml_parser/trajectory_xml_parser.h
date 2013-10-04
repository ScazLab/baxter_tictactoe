#ifndef TRAJECTORY_XML_PARSER_H
#define TRAJECTORY_XML_PARSER_H

/**
 * \brief Operations with trajectories and files
 * \author Alvaro Castro Gonzalez, acgonzal@ing.uc3m.es
 *
 * Class for working with trajectories and files.
 * The trajectory xml files follow the next structure:
 * <trajectory id="grasp" time_to_start="0">
 *      <point time_from_start="3">
 *          <joint name="right_s0" position="-1.5667843457" velocity="0.0" acceleration="0.0"/>
 *          <joint ... />
 *      </point>
 * </trajectory>
 * <trajectory id="put@1x1" ... > ... </trajectory>
 *
 * The time_to_start property of the trajectory elements represents the seconds elapsed before the trajectory starts; 0 means that it starts inmediatelly.
 * The id property of the trajectory elements must be unique for each trajectory. This is not checked in the code.
 */

#include <vector>
#include <qt4/Qt/QtCore>
// ROS includes
#include <ros/ros.h>
#include <ros/duration.h>
// Baxter includes
#include <control_msgs/FollowJointTrajectoryAction.h>

namespace ttt
{

class trajectory_xml_parser
{
private:
    /**
     * Read the joint names from a xml file where trajectories are stored.
     * It is assumed that all trajectories in the xml file use the same joint names, and the same number of joints for all points.
     * The joint names are extracted from the first point from the first trajectory.
     * \param filename a string containing the xml file name
     * \param joint_names a vector with the names of all joint
     * \return false if an error happens; true otherwise. In case of error the correcteness of the operation is not guarranted.
     **/
    static bool read_joint_names_from_file(std::string filename, std::vector<std::string>& joint_names);

public:

    /**
     * Write a trajectory to a xml file.
     * \param traj trajectory to be written
     * \param filename name or path for the xml file
     * \param trajectory_id trajectory id/name. It must be unique but this is not checked in the code.
     * \return false if an error happens; true otherwise. In case of error the correcteness of the operation is not guarranted.
     **/
    static bool write_to_file(trajectory_msgs::JointTrajectory traj, std::string filename, std::string trajectory_id);

    /**
     * Write several trajectories to a xml file.
     * \param trajs vector of trajectories to be written
     * \param filename name or path for the xml file
     * \param trajectory_ids vector of trajectory ids/names. Ids/names must be unique but this is not checked in the code.
     * \return false if an error happens; true otherwise. In case of error the correcteness of the operation is not guarranted.
     **/
    static bool write_to_file(std::vector<trajectory_msgs::JointTrajectory> trajs, std::string filename, std::vector<std::string> trajectory_ids);

    /**
     * Read all the trajectories from a xml file.
     * \param filename name or path for the xml file
     * \param trajs this is an output parameter. It is a vector where all the read trajectories are loaded. Previous trajectories are deleted.
     * \param traj_ids this is an output parameter. It is a vector where the read trajectory ids/names are loaded. Ids/names must be unique but this is not checked in the code. Previous ids are deleted.
     * \return false if an error happens; true otherwise. In case of error the correcteness of the operation is not guarranted.
     **/
    static bool read_from_file(std::string filename, std::vector<trajectory_msgs::JointTrajectory>& trajs, std::vector<std::string>& traj_ids);

    /**
     * This function returns a vector of points for a trajectory which are read from a so-called raw file.
     * The raw file has to be created by the echo of topis related to the state of the joints. For example, a raw file can be created executing:
     *      $>rostopic echo /robot/limb/left/joint_states -n 1 >> rawfile
     * The function will just consider the lines related to the position of the joints. That is lines following the next format:
     *      position: [double, ..., double]
     * where each double represents the position of a joint.
     * In case of error during the execution of the function, the function stops the execution.
     * \param filename name or path for the raw file
     * \param n_joints number of joints consider for the points. All the points will be related to the same joints.
     * \param time_gap the number of seconds to reach the next point
     * \return vector of points which can be assigned to a trajectory.
     **/
    static std::vector<trajectory_msgs::JointTrajectoryPoint> read_points_from_raw_file(std::string filename, int n_joints, double time_gap);
};

}
#endif // TRAJECTORY_XML_PARSER_H
