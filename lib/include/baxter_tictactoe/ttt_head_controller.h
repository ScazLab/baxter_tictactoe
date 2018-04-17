#include <ros/ros.h>


class TTTHeadController
{
private:
    ros::Publisher  cmd_pub; // Publisher for head moves
    float _pan;
    void pan_to(float angle);  // Moves to given position
public:
    TTTHeadController(ros::NodeHandle nh, float pan);  // pan angle in radians
    TTTHeadController(ros::NodeHandle nh);

    /* Sets pan value */
    void set_pan(float radian);

    /* Moves to panned position */
    void pan();

    /* Moves to center position (angle 0) */
    void center();
};
