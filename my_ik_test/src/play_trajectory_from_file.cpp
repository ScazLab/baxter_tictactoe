#include <ros/ros.h>
#include <ros/duration.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>
#include <qt4/QtCore/QFile>
#include <qt4/Qt/QtCore>

typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> Client;

#define N_JOINTS 7
#define TIME_GAP 2 // This defines the time between two points in the trajectory [=]s

std::vector<trajectory_msgs::JointTrajectoryPoint> get_points_from_file(std::string filename)
{
    ROS_DEBUG_STREAM("Reading positions from " << filename);
    QFile file(filename.c_str());
    ROS_ASSERT_MSG(file.open(QIODevice::ReadOnly | QIODevice::Text), "Error openning file to read positions");

    std::vector<trajectory_msgs::JointTrajectoryPoint> points;
    size_t counter = 1; // the first point starts 1 second after
    while (!file.atEnd()) {
        QByteArray line = file.readLine();
        if (line.startsWith("position:")) //this line contains the positions of the joints
        {
            line.remove(0,10);      //removing "position: "
            line.replace('[',' ');  //removing [
            line.replace(']',' ');  //removing ]
            line=line.trimmed();    //removing white spaces
            QList<QByteArray> joint_positions = line.split(',');
            /*foreach (QByteArray token, joint_positions) {
                ROS_DEBUG_STREAM("token = " << token.data());
            }*/
            //ROS_DEBUG_STREAM("positions read = " << line.data());
            ROS_ASSERT_MSG(joint_positions.size()==N_JOINTS, "Incongruency in number of joints positions");
            trajectory_msgs::JointTrajectoryPoint point;
            point.positions.resize(N_JOINTS,0.0);
            point.velocities.resize(N_JOINTS,0.0);
            point.accelerations.resize(N_JOINTS,0.0);
            bool ok=false;
            for (int i = 0; i < N_JOINTS; i++) {
                point.positions[i]=joint_positions[i].toDouble(&ok);
                if (!ok) ROS_WARN_STREAM("Error converting to double " << joint_positions[i].data());
                //ROS_DEBUG_STREAM("point.position[" << i << "]=" << point.positions[i] << " # joint_positions[" << i << "]=" << joint_positions[i].data() << "=>(toDouble)=>" << joint_positions[i].toDouble());
            }
            point.time_from_start=ros::Duration(TIME_GAP*counter++);
            points.push_back(point);
        }
    }
    return points;
}

int main(int argc, char **argv)
{

    ROS_INFO_STREAM("Play a left arm trajectory stored in a file");
    ROS_INFO_STREAM("Generaly the file's been created executing $>rostopic echo /robot/limb/left/joint_states -n 1 >> filename");

    ROS_ASSERT_MSG(argc>1, "File name required as a parameter");
    std::string filename = argv[1];

    ros::init(argc, argv, "trajectory_palyer");
    Client client("/sdk/robot/limb/left/follow_joint_trajectory", true); // true -> don't need to call ros::spin()
    ROS_INFO("Waiting for connecting with the server...");
    ROS_ASSERT_MSG(client.waitForServer(ros::Duration(10.0)),"Timeout. Server not available.");
    ROS_INFO("CONNECTED");

    std::vector<std::string> joint_manes(N_JOINTS);
    joint_manes[0]="left_e0";
    joint_manes[1]="left_e1";
    joint_manes[2]="left_s0";
    joint_manes[3]="left_s1";
    joint_manes[4]="left_w0";
    joint_manes[5]="left_w1";
    joint_manes[6]="left_w2";

    /*std::vector<trajectory_msgs::JointTrajectoryPoint> points = get_points_from_file(filename);
    foreach (trajectory_msgs::JointTrajectoryPoint p, points) {
        ROS_DEBUG_STREAM("Point: " << p);
    }*/

    // creating the trajectory
    trajectory_msgs::JointTrajectory traj;
    traj.joint_names=joint_manes;
    traj.points = get_points_from_file(filename);
    // Action goal
    control_msgs::FollowJointTrajectoryGoal goal;
    goal.trajectory = traj;
    ROS_INFO_STREAM("Press ENTER to start trajectory. Whatch people around Baxter!!!");
    std::cin.get();
    goal.trajectory.header.stamp = ros::Time::now(); // start now
    client.sendGoal(goal);
    ROS_ASSERT_MSG(client.waitForResult(),"Home not reached.");
    ROS_INFO("Home reached. Ready for next trajectory");
    ROS_INFO_STREAM("action client state is " << client.getState().toString());

    ros::shutdown();

}
