#include "trajectory_xml_parser.h"

namespace ttt
{

bool trajectory_xml_parser::write_to_file( trajectory_msgs::JointTrajectory traj, std::string filename, std::string trajectory_id)
{
    QFile outstream(filename.c_str());
    bool new_file = !outstream.exists();
    QIODevice::OpenMode om;
    if(new_file) om = QIODevice::WriteOnly | QIODevice::Text;   // if file does not exist, we create it
    else om = QIODevice::Append | QIODevice::Text;              // else we add data to the end of the file
    if(!outstream.open(om))
    {
        ROS_ERROR_STREAM("Error openning file (" << filename << ") to write trajectory");
        return false;
    }

    QXmlStreamWriter xmlstream(&outstream);
    xmlstream.setAutoFormatting(true);
    if(new_file) xmlstream.writeStartDocument();
    xmlstream.writeStartElement("trajectory");
    xmlstream.writeAttribute("id",trajectory_id.c_str());
    xmlstream.writeAttribute("time_to_start", QString::number(traj.header.stamp.toSec())); //the time to start is converted to seconds and then to a string
    std::vector<trajectory_msgs::JointTrajectoryPoint>& points=traj.points;
    foreach (trajectory_msgs::JointTrajectoryPoint p, points) {
        xmlstream.writeStartElement("point");
        xmlstream.writeAttribute("time_from_start",QString::number(p.time_from_start.toSec())); //the time to reach this point sice the trajectory started
        for (size_t i = 0; i < p.positions.size(); ++i) {
            xmlstream.writeStartElement("joint");
            xmlstream.writeAttribute("name",traj.joint_names[i].c_str());
            xmlstream.writeAttribute("position",QString::number(p.positions[i]));
            xmlstream.writeAttribute("velocity",QString::number(p.velocities[i]));
            xmlstream.writeAttribute("acceleration",QString::number(p.accelerations[i]));
            xmlstream.writeEndElement();
        }
        xmlstream.writeEndElement();
    }
    xmlstream.writeEndElement();
    xmlstream.writeEndDocument();
    return true;
}

bool trajectory_xml_parser::write_to_file(std::vector<trajectory_msgs::JointTrajectory> trajs, std::string filename, std::vector<std::string> trajectory_ids)
{
    QFile outstream(filename.c_str());
    bool new_file = !outstream.exists();
    QIODevice::OpenMode om;
    if(new_file) om = QIODevice::WriteOnly | QIODevice::Text;   // if file does not exist, we create it
    else om = QIODevice::Append | QIODevice::Text;              // else we add data to the end of the file
    if(!outstream.open(om))
    {
        ROS_ERROR_STREAM("Error openning file (" << filename << ") to write trajectory");
        return false;
    }
    QXmlStreamWriter xmlstream(&outstream);
    xmlstream.setAutoFormatting(true);
    if(new_file) xmlstream.writeStartDocument();
    for (size_t i = 0; i < trajs.size(); ++i)
    {
        trajectory_msgs::JointTrajectory& traj=trajs[i];
        xmlstream.writeStartElement("trajectory");
        xmlstream.writeAttribute("id",trajectory_ids[i].c_str());
        xmlstream.writeAttribute("time_to_start", QString::number(traj.header.stamp.toSec())); //the time to start is converted to seconds and then to a string
        std::vector<trajectory_msgs::JointTrajectoryPoint>& points=traj.points;
        foreach (trajectory_msgs::JointTrajectoryPoint p, points) {
            xmlstream.writeStartElement("point");
            xmlstream.writeAttribute("time_from_start",QString::number(p.time_from_start.toSec())); //the time to reach this point sice the trajectory started
            for (size_t i = 0; i < p.positions.size(); ++i) {
                xmlstream.writeStartElement("joint");
                xmlstream.writeAttribute("name",traj.joint_names[i].c_str());
                xmlstream.writeAttribute("position",QString::number(p.positions[i]));
                xmlstream.writeAttribute("velocity",QString::number(p.velocities[i]));
                xmlstream.writeAttribute("acceleration",QString::number(p.accelerations[i]));
                xmlstream.writeEndElement();
            }
            xmlstream.writeEndElement();
        }
    }
    xmlstream.writeEndElement();
    xmlstream.writeEndDocument();
    if (xmlstream.hasError()) {
        ROS_ERROR_STREAM("Error writing to output xml stream");
        return false;
    }
    return true;
}

bool trajectory_xml_parser::read_from_file(std::string filename, std::vector<trajectory_msgs::JointTrajectory>& trajs, std::vector<std::string>& traj_ids)
{
    QFile instream(filename.c_str());
    if(!instream.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        ROS_ERROR_STREAM("Error openning file (" << filename << ") to read trajectories");
        return false;
    }
    trajs.clear();//we delete possible trajectories
    traj_ids.clear();    
    QXmlStreamReader xmlstream(&instream);
    while (!xmlstream.atEnd() && xmlstream.readNextStartElement()) {
        std::string aux;
        bool ok=true;
        if (xmlstream.name()=="trajectory") {
            if(xmlstream.attributes().value("id").isEmpty()) {
                ROS_ERROR("Empty id for a trajectory");
                return false;
            }
            traj_ids.push_back(xmlstream.attributes().value("id").toString().toStdString());
            trajectory_msgs::JointTrajectory t;            
            t.header.stamp = ros::Time(xmlstream.attributes().value("time_to_start").toString().toDouble(&ok));
            if (xmlstream.attributes().value("time_to_start").isEmpty() || !ok) {
                ROS_ERROR_STREAM("Error parsing trajectory time_to_start. Error converting \"" << xmlstream.attributes().value("time_to_start").toString().toStdString() << "\" to double.");
                return false;
            }            
            if(!trajectory_xml_parser::read_joint_names_from_file(filename,t.joint_names)) {
                ROS_ERROR_STREAM("Error reading joint names from " << filename);
                return false;
            }
            trajs.push_back(t);
        }
        else if (xmlstream.name()=="point") {
            trajectory_msgs::JointTrajectoryPoint p;
            if(xmlstream.attributes().value("time_from_start").isEmpty()) {
                ROS_ERROR("Empty time_from_start for a poing");
                return false;
            }
            p.time_from_start=ros::Duration(xmlstream.attributes().value("time_from_start").toString().toDouble(&ok));
            if (xmlstream.attributes().value("time_from_start").isEmpty() || !ok) {
                ROS_ERROR_STREAM("Error parsing point time_from_start. Error converting \"" << xmlstream.attributes().value("time_from_start").toString().toStdString() << "\" to double.");
                return false;
            }
            if (trajs.empty()) {
                ROS_ERROR("Trying to access point from an empty trajectory");
                return false;
            }
            trajs[trajs.size()-1].points.push_back(p); //adding the point to the last trajectory in the vector
        }
        else if (xmlstream.name()=="joint") {
            if (trajs.empty()) {
                ROS_ERROR("Trying to access joint from an empty trajectory");
                return false;
            }
            trajs[trajs.size()-1].points[trajs[trajs.size()-1].points.size()-1].positions.push_back(xmlstream.attributes().value("position").toString().toDouble(&ok));
            if(xmlstream.attributes().value("position").isEmpty() || !ok) {
                ROS_ERROR_STREAM("Error parsing joint position. Error converting \"" << xmlstream.attributes().value("position").toString().toStdString() << "\" to double.");
                return false;
            }
            trajs[trajs.size()-1].points[trajs[trajs.size()-1].points.size()-1].velocities.push_back(xmlstream.attributes().value("velocity").toString().toDouble(&ok));
            if(xmlstream.attributes().value("velocity").isEmpty() || !ok) {
                ROS_ERROR_STREAM("Error parsing joint velocity. Error converting \"" << xmlstream.attributes().value("velocity").toString().toStdString() << "\" to double.");
                return false;
            }
            trajs[trajs.size()-1].points[trajs[trajs.size()-1].points.size()-1].accelerations.push_back(xmlstream.attributes().value("acceleration").toString().toDouble(&ok));
            if(xmlstream.attributes().value("acceleration").isEmpty() || !ok) {
                ROS_ERROR_STREAM("Error parsing joint acceleration. Error converting \"" << xmlstream.attributes().value("acceleration").toString().toStdString() << "\" to double.");
                return false;
            }
        } else {
            ROS_WARN_STREAM("Unknown element: " << xmlstream.name().toString().toStdString());
        }
    }
    if (xmlstream.hasError()) {
        ROS_ERROR_STREAM("Error reading from input xml stream from " << filename << ": " << xmlstream.errorString().toStdString() << " on line " << xmlstream.lineNumber() << ", colum " << xmlstream.columnNumber());
        return false;
    }

    return true;
}

std::vector<trajectory_msgs::JointTrajectoryPoint> trajectory_xml_parser::trajectory_xml_parser::read_points_from_raw_file(std::string filename, int n_joints, double time_gap)
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
            ROS_ASSERT_MSG(joint_positions.size()==n_joints, "Incongruency in number of joints positions");
            trajectory_msgs::JointTrajectoryPoint point;
            point.positions.resize(n_joints,0.0);
            point.velocities.resize(n_joints,0.0);
            point.accelerations.resize(n_joints,0.0);
            bool ok=false;
            for (int i = 0; i < n_joints; i++) {
                point.positions[i]=joint_positions[i].toDouble(&ok);
                if (!ok) ROS_WARN_STREAM("Error converting to double " << joint_positions[i].data());
                //ROS_DEBUG_STREAM("point.position[" << i << "]=" << point.positions[i] << " # joint_positions[" << i << "]=" << joint_positions[i].data() << "=>(toDouble)=>" << joint_positions[i].toDouble());
            }
            point.time_from_start=ros::Duration(time_gap*counter++);
            points.push_back(point);
        }
    }
    return points;
}

bool trajectory_xml_parser::read_joint_names_from_file(std::string filename, std::vector<std::string>& joint_names)
{
    QFile instream(filename.c_str());
    if(!instream.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        ROS_ERROR_STREAM("Error openning file (" << filename << ") to read joint names");
        return false;
    }
    joint_names.clear();//we delete possible joint names
    QXmlStreamReader xmlstream(&instream);
    while (!xmlstream.atEnd()) {
        if (xmlstream.isStartElement() && xmlstream.name()=="point") {
            while (!xmlstream.isEndElement() && xmlstream.name()!="point") {
                if (xmlstream.name()=="joint") {
                    if(xmlstream.attributes().value("name").isEmpty()) {
                        ROS_ERROR("Empty joint name");
                        return false;
                    }
                    joint_names.push_back(xmlstream.attributes().value("name").toString().toStdString());
                }
                xmlstream.readNextStartElement();
            }
            return true;
        }
        xmlstream.readNextStartElement();
    }
    if (xmlstream.hasError()) {
        ROS_ERROR_STREAM("Error reading joint names from " << filename << ": " << xmlstream.errorString().toStdString() << " on line " << xmlstream.lineNumber() << ", colum " << xmlstream.columnNumber());
        return false;
    }
    return false;
}

}
