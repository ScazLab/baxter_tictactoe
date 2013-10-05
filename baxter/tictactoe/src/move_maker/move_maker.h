#ifndef MOVE_MAKER_H
#define MOVE_MAKER_H

#include <control_msgs/FollowJointTrajectoryAction.h>
#include "trajectory_player.h"
#include "trajectory_xml_parser.h"

namespace ttt
{

class TTT_Trajectory
{
public:
    TTT_Trajectory(trajectory_msgs::JointTrajectory t, std::string id):trajectory(t),name(id){}
    TTT_Trajectory();
    trajectory_msgs::JointTrajectory trajectory;
    std::string name;

    TTT_Trajectory& operator=(TTT_Trajectory const & t)
    {
        this->name=t.name;
        this->trajectory=t.trajectory;
        return *this;
    }
};

enum Trajectory_Type { PLAIN, GRASP, RELEASE};

class Move_Maker
{
    std::vector<TTT_Trajectory> _trajectory_repository;
    size_t _ttt_traj_n;

    Trajectory_Player* _traj_player;

    bool get_ttt_trajectory(std::string traj_name, TTT_Trajectory& traj);

public:
    Move_Maker(const char * trajectory_file, const char * service);

    bool make_a_move(std::vector<std::string> traj_names, std::vector<Trajectory_Type> types);
};

}

#endif // MOVE_MAKER_H
