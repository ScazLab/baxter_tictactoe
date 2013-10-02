#include <ros/ros.h>
#include <ros/duration.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> Client;

trajectory_msgs::JointTrajectory get_1go_trajectory(std::vector<std::string> joint_names);
trajectory_msgs::JointTrajectory get_1back_trajectory(std::vector<std::string> joint_names);
trajectory_msgs::JointTrajectory get_2go_trajectory(std::vector<std::string> joint_names);
trajectory_msgs::JointTrajectory get_2back_trajectory(std::vector<std::string> joint_names);

int main(int argc, char **argv)
{

    ROS_INFO_STREAM("Two go-back trajectories in the left arm");
    ros::Duration long_timeout (10.0);

    ros::init(argc, argv, "two_trajectories_client");
    Client client("/sdk/robot/limb/left/follow_joint_trajectory", true); // true -> don't need to call ros::spin()
    ROS_INFO("Waiting for connecting with the server...");
    ROS_ASSERT_MSG(client.waitForServer(long_timeout),"Timeout. Server not available.");
    ROS_INFO("CONNECTED");


    unsigned int n_joints = 7;
    std::vector<std::string> joint_manes(n_joints);
    joint_manes[0]="left_e0";
    joint_manes[1]="left_e1";
    joint_manes[2]="left_s0";
    joint_manes[3]="left_s1";
    joint_manes[4]="left_w0";
    joint_manes[5]="left_w1";
    joint_manes[6]="left_w2";

    // 1go trajectory
    trajectory_msgs::JointTrajectoryPoint point;
    point.positions.resize(n_joints,0.0);
    point.velocities.resize(n_joints,0.0);
    point.accelerations.resize(n_joints,0.0);    
    trajectory_msgs::JointTrajectory traj_home;
    traj_home.joint_names=joint_manes;
//    traj_home.header.stamp = ros::Time(0); // start inmediately
    traj_home.points.resize(1,point);
    traj_home.points[0].positions[0] = -1.9684808438;
    traj_home.points[0].positions[1] = 2.46510712332;
    traj_home.points[0].positions[2] = 1.57961671452;
    traj_home.points[0].positions[3] = 0.277267027094;
    traj_home.points[0].positions[4] = 1.49601476168;
    traj_home.points[0].positions[5] = 1.97768472852;
    traj_home.points[0].positions[6] = -0.0636602026245;
    traj_home.points[0].time_from_start = ros::Duration(10.0);
    // Action goal
    control_msgs::FollowJointTrajectoryGoal goal_home;
    goal_home.trajectory = traj_home;
    ROS_INFO_STREAM("Press ENTER to start to trajectory to home");
    std::cin.get();
    goal_home.trajectory.header.stamp = ros::Time::now(); // start now
    client.sendGoal(goal_home);
    ROS_ASSERT_MSG(client.waitForResult(),"Home not reached.");
    ROS_INFO("Home reached. Ready for next trajectory");
    ROS_INFO_STREAM("action client state is " << client.getState().toString());

    trajectory_msgs::JointTrajectory t1go   = get_1go_trajectory(joint_manes);
    trajectory_msgs::JointTrajectory t1back = get_1back_trajectory(joint_manes);
    trajectory_msgs::JointTrajectory t2go   = get_2go_trajectory(joint_manes);
    trajectory_msgs::JointTrajectory t2back = get_2back_trajectory(joint_manes);

    std::string option="1";
    do{
//        ROS_INFO("Choose the point to go: [1,2]");
//        std::cin >> option;
        control_msgs::FollowJointTrajectoryGoal goal;
        if(option=="1")
        {
            goal.trajectory = t1go;
            goal.trajectory.header.stamp = ros::Time::now(); // start now
            client.sendGoal(goal);
            ROS_ASSERT_MSG(client.waitForResult(),"Goal not reached.");
            ROS_INFO("Point 1 reached!");
            ros::Duration(1).sleep(); // let's wait 1 second
            goal.trajectory = t1back;
            goal.trajectory.header.stamp = ros::Time::now(); // start now
            client.sendGoal(goal);
            ROS_ASSERT_MSG(client.waitForResult(),"Goal not reached.");
            ROS_INFO("Back point 1 reached!");
            option="2";
        }
        else if(option=="2")
        {
            goal.trajectory = t2go;
            goal.trajectory.header.stamp = ros::Time::now(); // start now
            client.sendGoal(goal);
            ROS_ASSERT_MSG(client.waitForResult(),"Goal not reached.");
            ROS_INFO("Point 2 reached!");
            ros::Duration(1).sleep(); // let's wait 1 second
            goal.trajectory = t2back;
            goal.trajectory.header.stamp = ros::Time::now(); // start now
            client.sendGoal(goal);
            ROS_ASSERT_MSG(client.waitForResult(),"Goal not reached.");
            ROS_INFO("Back point 2 reached!");
            option="1";
        }
        else break;

    }
    while(option=="1" || option=="2");


    ros::shutdown();

}

trajectory_msgs::JointTrajectory get_1go_trajectory(std::vector<std::string> joint_names)
{
    size_t n_joints = joint_names.size();

    trajectory_msgs::JointTrajectoryPoint point;
    point.positions.resize(n_joints,0.0);
    point.velocities.resize(n_joints,0.0);
    point.accelerations.resize(n_joints,0.0);

    trajectory_msgs::JointTrajectory t;
    t.joint_names=joint_names;
    t.points.resize(7,point);  // This trajectory has 7 points

//name: ['e0', 'e1', 's0', 's1', 'w0', 'w1', 'w2']
//position: [-1.9680973486083986, 2.465490618511963, 1.5811506953063965, 0.2653786760009766, 1.4963982568725587, 1.9788352141113283, -0.06289321223144531]
    t.points[0].positions[0] = -1.9680973486083986;
    t.points[0].positions[1] = 2.465490618511963;
    t.points[0].positions[2] = 1.5811506953063965;
    t.points[0].positions[3] = 0.2653786760009766;
    t.points[0].positions[4] = 1.4963982568725587;
    t.points[0].positions[5] = 1.9788352141113283;
    t.points[0].positions[6] = -0.06289321223144531;
    t.points[0].time_from_start = ros::Duration(1);

//name: ['e0', 'e1', 's0', 's1', 'w0', 'w1', 'w2']
//position: [-2.02332065690918, 2.2054808752624515, 0.7424467004882813, 0.22626216595458987, 1.5416506900634768, 2.0470973590942383, -0.4015194707702637]
    t.points[1].positions[0] = -2.02332065690918;
    t.points[1].positions[1] =  2.2054808752624515;
    t.points[1].positions[2] = 0.7424467004882813;
    t.points[1].positions[3] = 0.22626216595458987;
    t.points[1].positions[4] = 1.5416506900634768;
    t.points[1].positions[5] = 2.0470973590942383;
    t.points[1].positions[6] = -0.4015194707702637;
    t.points[1].time_from_start = ros::Duration(2);

//name: ['e0', 'e1', 's0', 's1', 'w0', 'w1', 'w2']
//position: [-1.991490555596924, 1.857650732006836, 0.18561167512207033, 0.2676796471801758, 1.8031944140991212, 1.9853546324523927, -0.6247136751525879]
    t.points[2].positions[0] = -1.991490555596924;
    t.points[2].positions[1] = 1.857650732006836;
    t.points[2].positions[2] = 0.18561167512207033;
    t.points[2].positions[3] = 0.2676796471801758;
    t.points[2].positions[4] = 1.8031944140991212;
    t.points[2].positions[5] = 1.9853546324523927;
    t.points[2].positions[6] = -0.6247136751525879;
    t.points[2].time_from_start = ros::Duration(3);

//name: ['e0', 'e1', 's0', 's1', 'w0', 'w1', 'w2']
//position: [-1.9232284106140138, 1.3755972699645997, -0.23738352665405274, 0.29797576770629886, 1.9780682237182619, 1.7775002359313965, -0.7838641817138673]
    t.points[3].positions[0] = -1.9232284106140138;
    t.points[3].positions[1] = 1.3755972699645997;
    t.points[3].positions[2] = -0.23738352665405274;
    t.points[3].positions[3] = 0.29797576770629886;
    t.points[3].positions[4] = 1.9780682237182619;
    t.points[3].positions[5] = 1.7775002359313965;
    t.points[3].positions[6] = -0.7838641817138673;
    t.points[3].time_from_start = ros::Duration(4);

//name: ['e0', 'e1', 's0', 's1', 'w0', 'w1', 'w2']
//position: [-2.0398109503601076, 0.5319078375915528, -0.6504078533203126, 0.1967330358215332, 2.0996362010192873, 1.5922720560058594, -1.1129030603393555]
    t.points[4].positions[0] = -2.0398109503601076;
    t.points[4].positions[1] = 0.5319078375915528;
    t.points[4].positions[2] = -0.6504078533203126;
    t.points[4].positions[3] = 0.1967330358215332;
    t.points[4].positions[4] = 2.0996362010192873;
    t.points[4].positions[5] = 1.5922720560058594;
    t.points[4].positions[6] = -1.1129030603393555;
    t.points[4].time_from_start = ros::Duration(5);

//name: ['e0', 'e1', 's0', 's1', 'w0', 'w1', 'w2']
//position: [-2.0605196909729004, 0.6807039738464356, -0.5825292035339356, 0.2949078061340332, 2.1456556246032714, 1.5830681712890626, -1.336480759918213]
    t.points[5].positions[0] = -2.0605196909729004;
    t.points[5].positions[1] = 0.6807039738464356;
    t.points[5].positions[2] = -0.5825292035339356;
    t.points[5].positions[3] = 0.2949078061340332;
    t.points[5].positions[4] = 2.1456556246032714;
    t.points[5].positions[5] = 1.5830681712890626;
    t.points[5].positions[6] = -1.336480759918213;
    t.points[5].time_from_start = ros::Duration(6);

//name: ['e0', 'e1', 's0', 's1', 'w0', 'w1', 'w2']
//position: [-2.060903186169434, 0.6799369834533692, -0.5836796891235352, 0.2937573205444336, 2.144888634210205, 1.5792332193237306, -1.3368642551147463]
    t.points[6].positions[0] = -2.060903186169434;
    t.points[6].positions[1] = 0.6799369834533692;
    t.points[6].positions[2] = -0.5836796891235352;
    t.points[6].positions[3] = 0.2937573205444336;
    t.points[6].positions[4] = 2.144888634210205;
    t.points[6].positions[5] = 1.5792332193237306;
    t.points[6].positions[6] = -1.3368642551147463;
    t.points[6].time_from_start = ros::Duration(7);

    return t;
}


trajectory_msgs::JointTrajectory get_1back_trajectory(std::vector<std::string> joint_names)
{
    size_t n_joints = joint_names.size();

    trajectory_msgs::JointTrajectoryPoint point;
    point.positions.resize(n_joints,0.0);
    point.velocities.resize(n_joints,0.0);
    point.accelerations.resize(n_joints,0.0);

    trajectory_msgs::JointTrajectory t;
    t.joint_names=joint_names;
    t.points.resize(5,point);  // This trajectory has 5 points

//name: ['e0', 'e1', 's0', 's1', 'w0', 'w1', 'w2']
//position: [-2.0601361957763675, 0.6799369834533692, -0.5829126987304688, 0.2949078061340332, 2.1445051390136722, 1.5796167145202638, -1.3368642551147463]
    t.points[0].positions[0] = -2.0601361957763675;
    t.points[0].positions[1] = 0.6799369834533692;
    t.points[0].positions[2] = -0.5829126987304688;
    t.points[0].positions[3] = 0.2949078061340332;
    t.points[0].positions[4] = 2.1445051390136722;
    t.points[0].positions[5] = 1.5796167145202638;
    t.points[0].positions[6] = -1.3368642551147463;
    t.points[0].time_from_start = ros::Duration(1);

//name: ['e0', 'e1', 's0', 's1', 'w0', 'w1', 'w2']
//position: [-2.1299323215454105, 1.0415729537841798, -0.48090297645263674, 0.3313398498046875, 2.2422964141296386, 1.6927477974975587, -1.6827769223876954]
    t.points[1].positions[0] = -2.1299323215454105;
    t.points[1].positions[1] = 1.0415729537841798;
    t.points[1].positions[2] = -0.48090297645263674;
    t.points[1].positions[3] = 0.3313398498046875;
    t.points[1].positions[4] = 2.2422964141296386;
    t.points[1].positions[5] = 1.6927477974975587;
    t.points[1].positions[6] = -1.6827769223876954;
    t.points[1].time_from_start = ros::Duration(2);

//name: ['e0', 'e1', 's0', 's1', 'w0', 'w1', 'w2']
//position: [-1.9849711372558596, 1.7759662551452637, 0.060592241052246094, 0.3014272244750977, 1.869155587902832, 1.979985699700928, -1.6808594464050295]
    t.points[2].positions[0] = -1.9849711372558596;
    t.points[2].positions[1] = 1.7759662551452637;
    t.points[2].positions[2] = 0.060592241052246094;
    t.points[2].positions[3] = 0.3014272244750977;
    t.points[2].positions[4] = 1.869155587902832;
    t.points[2].positions[5] = 1.979985699700928;
    t.points[2].positions[6] = -1.6808594464050295;
    t.points[2].time_from_start = ros::Duration(3);

//name: ['e0', 'e1', 's0', 's1', 'w0', 'w1', 'w2']
//position: [-2.004529392279053, 2.416786728552246, 0.7052476664245606, 0.3570340279724121, 1.5684953538208009, 2.093116782678223, -1.6352235180175783]
    t.points[3].positions[0] = -2.004529392279053;
    t.points[3].positions[1] = 2.416786728552246;
    t.points[3].positions[2] = 0.7052476664245606;
    t.points[3].positions[3] = 0.3570340279724121;
    t.points[3].positions[4] = 1.5684953538208009;
    t.points[3].positions[5] = 2.093116782678223;
    t.points[3].positions[6] = -1.6352235180175783;
    t.points[3].time_from_start = ros::Duration(4);

//name: ['e0', 'e1', 's0', 's1', 'w0', 'w1', 'w2']
//position: [-2.044796387915039, 2.6085343268188477, 1.7088545957519532, 0.4406359808166504, 1.5447186516357423, 2.09158280189209, -0.13230584280395508]
    t.points[4].positions[0] = -2.044796387915039;
    t.points[4].positions[1] = 2.6085343268188477;
    t.points[4].positions[2] = 1.7088545957519532;
    t.points[4].positions[3] = 0.4406359808166504;
    t.points[4].positions[4] = 1.5447186516357423;
    t.points[4].positions[5] = 2.09158280189209;
    t.points[4].positions[6] = -0.13230584280395508;
    t.points[4].time_from_start = ros::Duration(5);

    return t;
}

trajectory_msgs::JointTrajectory get_2go_trajectory(std::vector<std::string> joint_names)
{
    size_t n_joints = joint_names.size();

    trajectory_msgs::JointTrajectoryPoint point;
    point.positions.resize(n_joints,0.0);
    point.velocities.resize(n_joints,0.0);
    point.accelerations.resize(n_joints,0.0);

    trajectory_msgs::JointTrajectory t;
    t.joint_names=joint_names;
    t.points.resize(6,point);  // This trajectory has 6 points

//name: ['e0', 'e1', 's0', 's1', 'w0', 'w1', 'w2']
//position: [-1.9677138534118654, 2.46510712331543, 1.5788497241271973, 0.2765000367004395, 1.4956312664794922, 1.978451718914795, -0.06327670742797852]
    t.points[0].positions[0] = -1.9677138534118654;
    t.points[0].positions[1] = 2.46510712331543;
    t.points[0].positions[2] = 1.5788497241271973;
    t.points[0].positions[3] = 0.2765000367004395;
    t.points[0].positions[4] = 1.4956312664794922;
    t.points[0].positions[5] = 1.978451718914795;
    t.points[0].positions[6] = -0.06327670742797852;
    t.points[0].time_from_start = ros::Duration(1);

//name: ['e0', 'e1', 's0', 's1', 'w0', 'w1', 'w2']
//position: [-1.932815790527344, 2.074325518048096, 0.6477233869445801, 0.21283983407592774, 1.6233351669250489, 1.9397187040649415, -0.604004934539795]
    t.points[1].positions[0] = -1.932815790527344;
    t.points[1].positions[1] = 2.074325518048096;
    t.points[1].positions[2] = 0.6477233869445801;
    t.points[1].positions[3] = 0.21283983407592774;
    t.points[1].positions[4] = 1.6233351669250489;
    t.points[1].positions[5] = 1.9397187040649415;
    t.points[1].positions[6] = -0.604004934539795;
    t.points[1].time_from_start = ros::Duration(2);

//name: ['e0', 'e1', 's0', 's1', 'w0', 'w1', 'w2']
//position: [-1.7855536350585939, 1.7042526533935547, 0.03298058690185547, 0.15378157380981447, 1.7571749905151368, 1.7475876106018067, -1.1209564594665529]
    t.points[2].positions[0] = -1.7855536350585939;
    t.points[2].positions[1] = 1.7042526533935547;
    t.points[2].positions[2] = 0.03298058690185547;
    t.points[2].positions[3] = 0.15378157380981447;
    t.points[2].positions[4] = 1.7571749905151368;
    t.points[2].positions[5] = 1.7475876106018067;
    t.points[2].positions[6] = -1.1209564594665529;
    t.points[2].time_from_start = ros::Duration(3);

//name: ['e0', 'e1', 's0', 's1', 'w0', 'w1', 'w2']
//position: [-1.9742332717529298, 0.8034224367370606, -0.709849608782959, 0.20401944455566406, 2.0302235704467773, 1.6850778935668946, -1.485660391369629]
    t.points[3].positions[0] = -1.9742332717529298;
    t.points[3].positions[1] = 0.8034224367370606;
    t.points[3].positions[2] = -0.709849608782959;
    t.points[3].positions[3] = 0.20401944455566406;
    t.points[3].positions[4] = 2.0302235704467773;
    t.points[3].positions[5] = 1.6850778935668946;
    t.points[3].positions[6] = -1.485660391369629;
    t.points[3].time_from_start = ros::Duration(4);

//name: ['e0', 'e1', 's0', 's1', 'w0', 'w1', 'w2']
//position: [-2.0210196857299807, 0.4122573362731934, -0.9591214865295411, 0.17832526638793947, 2.0440293975219728, 1.5550730219421387, -1.5543060315490724]
    t.points[4].positions[0] = -2.0210196857299807;
    t.points[4].positions[1] = 0.4122573362731934;
    t.points[4].positions[2] = -0.9591214865295411;
    t.points[4].positions[3] = 0.17832526638793947;
    t.points[4].positions[4] = 2.0440293975219728;
    t.points[4].positions[5] = 1.5550730219421387;
    t.points[4].positions[6] = 1.5543060315490724;
    t.points[4].time_from_start = ros::Duration(5);

//name: ['e0', 'e1', 's0', 's1', 'w0', 'w1', 'w2']
//position: [-2.031374056036377, 0.4245291825622559, -0.9802137223388673, 0.22357769957885743, 2.0888983355163577, 1.5263108822021485, -1.6513303162719728]
    t.points[5].positions[0] = -2.031374056036377;
    t.points[5].positions[1] = 0.4245291825622559;
    t.points[5].positions[2] = -0.9802137223388673;
    t.points[5].positions[3] = 0.22357769957885743;
    t.points[5].positions[4] = 2.0888983355163577;
    t.points[5].positions[5] = 1.5263108822021485;
    t.points[5].positions[6] = -1.6513303162719728;
    t.points[5].time_from_start = ros::Duration(6);

    return t;
}


trajectory_msgs::JointTrajectory get_2back_trajectory(std::vector<std::string> joint_names)
{
    size_t n_joints = joint_names.size();

    trajectory_msgs::JointTrajectoryPoint point;
    point.positions.resize(n_joints,0.0);
    point.velocities.resize(n_joints,0.0);
    point.accelerations.resize(n_joints,0.0);

    trajectory_msgs::JointTrajectory t;
    t.joint_names=joint_names;
    t.points.resize(5,point);  // This trajectory has 5 points



//name: ['e0', 'e1', 's0', 's1', 'w0', 'w1', 'w2']
//position: [-2.031374056036377, 0.4256796681518555, -0.9805972175354004, 0.22511168036499024, 2.0865973643371585, 1.5224759302368165, -1.651713811468506]
    t.points[0].positions[0] = -2.031374056036377;
    t.points[0].positions[1] = 0.4256796681518555;
    t.points[0].positions[2] = -0.9805972175354004;
    t.points[0].positions[3] = 0.22511168036499024;
    t.points[0].positions[4] = 2.0865973643371585;
    t.points[0].positions[5] = 1.5224759302368165;
    t.points[0].positions[6] = -1.651713811468506;
    t.points[0].time_from_start = ros::Duration(1);

//name: ['e0', 'e1', 's0', 's1', 'w0', 'w1', 'w2']
//position: [-2.058985710186768, 0.656927271661377, -0.8808884664367677, 0.17602429520874024, 2.0620536717590334, 1.6616846865783692, -1.967330358215332]
    t.points[1].positions[0] = -2.058985710186768;
    t.points[1].positions[1] = 0.656927271661377;
    t.points[1].positions[2] = -0.8808884664367677;
    t.points[1].positions[3] = 0.17602429520874024;
    t.points[1].positions[4] = 2.0620536717590334;
    t.points[1].positions[5] = 1.6616846865783692;
    t.points[1].positions[6] = -1.967330358215332;
    t.points[1].time_from_start = ros::Duration(2);

//name: ['e0', 'e1', 's0', 's1', 'w0', 'w1', 'w2']
//position: [-2.058218719793701, 1.6455778883239747, -0.22050973800659182, 0.3815777205505371, 1.9301313241516114, 1.9968594883483888, -2.771519785345459]
    t.points[2].positions[0] = -2.058218719793701;
    t.points[2].positions[1] = 1.6455778883239747;
    t.points[2].positions[2] = -0.22050973800659182;
    t.points[2].positions[3] = 0.3815777205505371;
    t.points[2].positions[4] = 1.9301313241516114;
    t.points[2].positions[5] = 1.9968594883483888;
    t.points[2].positions[6] = -2.771519785345459;
    t.points[2].time_from_start = ros::Duration(3);

//name: ['e0', 'e1', 's0', 's1', 'w0', 'w1', 'w2']
//position: [-2.009898325030518, 2.261471173956299, 0.7213544646789551, 0.42798063933105474, 1.79705849095459, 2.0908158114990236, -2.763849881414795]
    t.points[3].positions[0] = -2.009898325030518;
    t.points[3].positions[1] = 2.261471173956299;
    t.points[3].positions[2] = 0.7213544646789551;
    t.points[3].positions[3] = 0.42798063933105474;
    t.points[3].positions[4] = 1.79705849095459;
    t.points[3].positions[5] = 2.0908158114990236;
    t.points[3].positions[6] = -2.763849881414795;
    t.points[3].time_from_start = ros::Duration(4);

//name: ['e0', 'e1', 's0', 's1', 'w0', 'w1', 'w2']
//position: [-1.987655603631592, 2.5958789853332522, 1.69888372064209, 0.4137913170593262, 1.5619759354797365, 2.075092508441162, -2.7726702709350586]
    t.points[4].positions[0] = -1.987655603631592;
    t.points[4].positions[1] = 2.5958789853332522;
    t.points[4].positions[2] = 1.69888372064209;
    t.points[4].positions[3] = 0.4137913170593262;
    t.points[4].positions[4] = 1.5619759354797365;
    t.points[4].positions[5] = 2.075092508441162;
    t.points[4].positions[6] = -2.7726702709350586;
    t.points[4].time_from_start = ros::Duration(5);

    return t;
}
