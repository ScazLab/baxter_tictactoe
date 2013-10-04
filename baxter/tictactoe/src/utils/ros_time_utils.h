#ifndef ROS_TIME_UTILS_H
#define ROS_TIME_UTILS_H

class ros_time_utils
{
public:

    /**
     * Convert a ROS time variable to a human readable string.
     * \param t a ROS time value
     * \return a string in the format YYYY-MM-DD-HH:MM:SS
     **/
    static std::string ros_time_to_str(ros::Time ros_t)
    {
        char buf[1024] = "";
        time_t t = ros_t.sec;
        struct tm *tms = localtime(&t);
        strftime(buf, 1024, "%Y-%m-%d-%H:%M:%S", tms);
        return std::string(buf);
    }
}
#endif // ROS_TIME_UTILS_H
