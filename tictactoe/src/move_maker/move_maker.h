#ifndef MOVE_MAKER_H
#define MOVE_MAKER_H

#include <ros/ros.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

#include <tictactoe/PlaceTokenAction.h>
#include <actionlib/server/simple_action_server.h>

#include "src/trajectory_player/trajectory_player.h"
#include "src/trajectory_xml_parser/trajectory_xml_parser.h"
#include "src/utils/my_ros_utils.h"
#include "ttt_moves_trajectories.h"

#include <tictactoe/SetTrajectoryType.h>

#define INTER_TRAJ_GAP 0.5 // 0.5s

namespace ttt
{

typedef actionlib::SimpleActionServer<tictactoe::PlaceTokenAction> Server;

class TTT_Trajectory
{
public:
    TTT_Trajectory(trajectory_msgs::JointTrajectory t, std::string id):trajectory(t),name(id){}
    TTT_Trajectory():name("noname"){}
    trajectory_msgs::JointTrajectory trajectory;
    std::string name;

    TTT_Trajectory& operator=(TTT_Trajectory const & t)
    {
        this->name=t.name;
        this->trajectory=t.trajectory;
        return *this;
    }

    inline std::string get_ttt_trajectory_description()
    {
        std::stringstream description;
        description << "TTT Trajectory called " << this->name << " has " << this->trajectory.points.size() << " points";
        return description.str();
    }
};

enum Trajectory_Type { PLAIN, GRASP, RELEASE};

class Move_Maker
{
    std::vector<TTT_Trajectory> _trajectory_repository;
    size_t _ttt_traj_n;

    ros::NodeHandle _n;

    std::string _prefix_traj; //! This variable is used to identified between smooth and mechanistic trajectories. It will be put ahead of the trajectory names. Possible values are "mech_" or "".

    bool set_movement_type(tictactoe::SetTrajectoryType::Request& req, tictactoe::SetTrajectoryType::Response& res);
    ros::ServiceServer _srv_set_traj_type;

    Trajectory_Player* _traj_player;    

    Server _place_token;
    void execute_place_token(const tictactoe::PlaceTokenGoalConstPtr& goal);
    tictactoe::PlaceTokenFeedback _place_token_feedback;
    tictactoe::PlaceTokenResult _place_token_result;

    bool get_ttt_trajectory(std::string traj_name, TTT_Trajectory& traj);

    bool is_preempted();

    std::string* get_trajectories_to_cell(std::string cell_id);

    void execute_single_trajectory(std::string traj_id, Trajectory_Type mode);

    inline void set_mechanistic_trajectories()
    {
        _prefix_traj="mech_";
    }

    inline void set_smooth_trajectories()
    {
        _prefix_traj="";
    }

public:
    Move_Maker(const char * trajectory_file, const char * service);

    bool make_a_move(std::vector<std::string> traj_names, std::vector<Trajectory_Type> modes);

    void print_trajectory_repository_details();

};

}

#endif // MOVE_MAKER_H
