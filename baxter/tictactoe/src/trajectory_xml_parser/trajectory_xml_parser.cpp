#include "trajectory_xml_parser.h"

namespace ttt
{

bool trajectory_xml_parser::write_to_file(const trajectory_msgs::JointTrajectory traj, std::string filename, const std::string trajectory_id)
{
    return trajectory_xml_parser::write_to_file(std::vector<trajectory_msgs::JointTrajectory>(1,traj),filename,std::vector<std::string>(1,trajectory_id));
}

bool trajectory_xml_parser::write_to_file(const std::vector<trajectory_msgs::JointTrajectory> trajs, std::string filename,const std::vector<std::string> traj_ids)
{
    ROS_DEBUG_STREAM(trajs.size() << " trajectories and " << traj_ids.size() << " will be added to the file " << filename);
    QFile outstream(filename.c_str());
    bool new_file = !outstream.exists();

    std::vector<trajectory_msgs::JointTrajectory> existing_trajs;
    std::vector<std::string> existing_traj_ids;
    if(!new_file)
    {
        if(!trajectory_xml_parser::read_from_file(filename,existing_trajs,existing_traj_ids))
        {
            ROS_ERROR_STREAM("Impossible to access already existing trajectories in the file " << filename);
            return false;
        }
        ROS_DEBUG_STREAM(existing_trajs.size() << " trajectories and " << existing_traj_ids.size() << " ids already exist at " << filename);
    }
    existing_trajs.insert(existing_trajs.end(),trajs.begin(),trajs.end()); //concat previous and new trajectories
    existing_traj_ids.insert(existing_traj_ids.end(),traj_ids.begin(),traj_ids.end()); //concat previous and new traj. ids

    ROS_DEBUG_STREAM(existing_trajs.size() << " trajectories and " << existing_traj_ids.size() << " ids will be written to " << filename);

    if(!outstream.open(QIODevice::WriteOnly | QIODevice::Text))
    {
        ROS_ERROR_STREAM("Error openning file (" << filename << ") to write trajectory");
        return false;
    }
    QXmlStreamWriter xmlstream(&outstream);
    xmlstream.setAutoFormatting(true);
    xmlstream.writeStartDocument();
    xmlstream.writeStartElement("trajectories");
    for (size_t i = 0; i < existing_trajs.size(); ++i)
    {
        const trajectory_msgs::JointTrajectory& traj=existing_trajs[i];
        xmlstream.writeStartElement("trajectory");
        xmlstream.writeAttribute("id",existing_traj_ids[i].c_str());
        xmlstream.writeAttribute("time_to_start", QString::number(traj.header.stamp.toSec())); //the time to start is converted to seconds and then to a string
        const std::vector<trajectory_msgs::JointTrajectoryPoint>& points=traj.points;
        foreach (trajectory_msgs::JointTrajectoryPoint p, points) {
            xmlstream.writeStartElement("point");
            xmlstream.writeAttribute("time_from_start",QString::number(p.time_from_start.toSec())); //the time to reach this point sice the trajectory started
            for (size_t i = 0; i < p.positions.size(); ++i) {
                xmlstream.writeStartElement("joint");
                xmlstream.writeAttribute("name",traj.joint_names[i].c_str());
                xmlstream.writeAttribute("position",QString::number(p.positions[i]));
                xmlstream.writeAttribute("velocity",QString::number(p.velocities[i]));
                xmlstream.writeAttribute("acceleration",QString::number(p.accelerations[i]));
                xmlstream.writeEndElement(); //</joint>
            }
            xmlstream.writeEndElement(); //</point>
        }
        xmlstream.writeEndElement(); //</trajectory>
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
    trajs.clear();      // We delete all previous trajectories
    traj_ids.clear();   // and its names
    QXmlStreamReader xmlstream(&instream);
    while (!xmlstream.atEnd() && !xmlstream.hasError()) {
        QXmlStreamReader::TokenType token = xmlstream.readNext();
        std::string aux;
        bool ok=true;
        if (token!=QXmlStreamReader::StartDocument && token==QXmlStreamReader::StartElement)
        {
            if (xmlstream.name()=="trajectory")
            {
                ROS_DEBUG_STREAM("Parsing TRAJECTORY element with id " << xmlstream.attributes().value("id").toString().toStdString());
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
            else if (xmlstream.name()=="point")
            {
                ROS_DEBUG_STREAM("Parsing POINT element with starting time " << xmlstream.attributes().value("time_from_start").toString().toStdString());
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
            else if (xmlstream.name()=="joint")
            {
                ROS_DEBUG_STREAM("Parsing JOINT element named " << xmlstream.attributes().value("name").toString().toStdString());
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
            }
            else {
                ROS_WARN_STREAM("Unparsed element: " << xmlstream.name().toString().toStdString());
            }
        }
    }
    if (xmlstream.hasError()) {
        ROS_ERROR_STREAM("Error reading from input xml stream from " << filename << ": " << xmlstream.errorString().toStdString() << " on line " << xmlstream.lineNumber() << ", colum " << xmlstream.columnNumber());
        return false;
    }
    xmlstream.clear();
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
    while (!xmlstream.atEnd() && !xmlstream.hasError())
    {
        QXmlStreamReader::TokenType token = xmlstream.readNext();
        if (token!=QXmlStreamReader::StartDocument && token==QXmlStreamReader::StartElement && xmlstream.name()=="point")
        {
            ROS_DEBUG_STREAM("We found a point starting on " << xmlstream.attributes().value("time_from_start").toString().toStdString());
            while (token!=QXmlStreamReader::EndElement || xmlstream.name()!="point")
            {
                if (token==QXmlStreamReader::StartElement && xmlstream.name()=="joint")
                {
                    if(xmlstream.attributes().value("name").isEmpty()) {
                        ROS_ERROR("Empty joint name");
                        return false;
                    }
                    ROS_DEBUG_STREAM("Adding joint name " << xmlstream.attributes().value("name").toString().toStdString());
                    joint_names.push_back(xmlstream.attributes().value("name").toString().toStdString());
                }
                token = xmlstream.readNext();
            }
            return true;
        }
    }
    if (xmlstream.hasError()) {
        ROS_ERROR_STREAM("Error reading joint names from " << filename << ": " << xmlstream.errorString().toStdString() << " on line " << xmlstream.lineNumber() << ", colum " << xmlstream.columnNumber());
        return false;
    }
    return false;
}

}
