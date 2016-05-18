#include "baxterTictactoe/tictactoe_utils.h"

using namespace std;
using namespace ttt;

std::string cell_state_to_str(cellState c_s)
{
    switch(c_s)
    {
        case empty: return std::string("empty");
        case blue: return std::string("blue");
        case red: return std::string("red");
        case undefined: return std::string("undefined");
        default: return std::string("[value unknown]");
    }
}

/**************************************************************************/
/**                        COLOR_RANGE                                   **/
/**************************************************************************/

colorRange & colorRange::operator=(const colorRange &_cr)
{
    min=_cr.min;
    max=_cr.max;
    return *this;
}

/**************************************************************************/
/**                         HSV_COLOR                                    **/
/**************************************************************************/

hsvColorRange::hsvColorRange(XmlRpc::XmlRpcValue _params)
{
    ROS_ASSERT(_params.getType()==XmlRpc::XmlRpcValue::TypeStruct);

    for (XmlRpc::XmlRpcValue::iterator i=_params.begin(); i!=_params.end(); ++i)
    {
        ROS_ASSERT(i->second.getType()==XmlRpc::XmlRpcValue::TypeArray);
        for(int j=0; j<i->second.size(); ++j)
        {
            ROS_ASSERT(i->second[j].getType()==XmlRpc::XmlRpcValue::TypeInt);
        }
        // printf("%s %i %i\n", i->first.c_str(), static_cast<int>(i->second[0]), static_cast<int>(i->second[1]));
        if (i->first == "H") H=colorRange(static_cast<int>(i->second[0]),static_cast<int>(i->second[1]));
        if (i->first == "S") S=colorRange(static_cast<int>(i->second[0]),static_cast<int>(i->second[1]));
        if (i->first == "V") V=colorRange(static_cast<int>(i->second[0]),static_cast<int>(i->second[1]));
    }
}

string hsvColorRange::toString()
{
    stringstream res;
    res <<"H=["<<H.min<<"\t"<<H.max<<"]\t"
        <<"S=["<<S.min<<"\t"<<S.max<<"]\t"
        <<"V=["<<V.min<<"\t"<<V.max<<"]";
    return res.str();
}

hsvColorRange & hsvColorRange::operator=(const hsvColorRange &_hsvc)
{
    H=_hsvc.H;
    S=_hsvc.S;
    V=_hsvc.V;
    return *this;
}

/**************************************************************************/
/**                        CELL                                          **/
/**************************************************************************/

cv::Mat Cell::mask_image(const cv::Mat &_src)
{
    cv::Mat mask = cv::Mat::zeros(_src.rows, _src.cols, CV_8UC1);

    // CV_FILLED fills the connected components found with white
    cv::drawContours(mask, std::vector<std::vector<cv::Point> >(1,contours),
                                            -1, cv::Scalar(255), CV_FILLED);  

    cv::Mat im_crop(_src.rows, _src.cols, CV_8UC3);                           
    im_crop.setTo(cv::Scalar(0));
    _src.copyTo(im_crop, mask);                  

    // normalize so imwrite(...)/imshow(...) shows the mask correctly
    //cv::normalize(mask.clone(), mask, 0.0, 255.0, CV_MINMAX, CV_8UC1);

    // show the images
    // cv::imshow("original", _src);
    // cv::imshow("mask", mask);
    // cv::imshow("masked cell", im_crop);

    return im_crop;
}

bool Cell::get_cell_centroid(cv::Point& centroid)
{
    uint sumX = 0, sumY = 0;
    size_t size = contours.size();
    centroid.x=centroid.y=0;
    if(size > 0){
        for (std::vector<cv::Point>::iterator it_point = contours.begin(); it_point != contours.end(); ++it_point) {
            sumX += it_point->x;
            sumY += it_point->y;
        }
        // TODO throw exception if size <= 0
        centroid.x = sumX/size;
        centroid.y = sumY/size;
        return true;
    }
    else
        return false;
}

string Cell::toString()
{
    stringstream res;

    res<<"State:"<<cell_state_to_str(state)<<"\t";
    res<<"Red  Area: "<<cell_area_red<<"\t";
    res<<"Blue Area: "<<cell_area_blue<<"\t";

    if (contours.size()==0)
    {
        res << "Points:\tNONE;\t";
    }
    else
    {
        res << "Points:\t";
        for (int i = 0; i < contours.size(); ++i)
        {
            res <<"["<<contours[i].x<<"  "<<contours[i].y<<"]\t";
        }
    }

    return res.str();
}

/**************************************************************************/
/**                                 BOARD                                **/
/**************************************************************************/

bool Board::resetState()
{
    if (cells.size()==0)
    {
        return false;
    }

    for (int i = 0; i < cells.size(); ++i)
    {
        cells[i].state      = empty;
        cells[i].cell_area_red  = 0;
        cells[i].cell_area_blue = 0;
    }

    return true;
}

string Board::stateToString()
{
    if (cells.size()==0)
    {
        return "";
    }

    stringstream res;
    res << cell_state_to_str(cells[0].state);
    for (int i = 1; i < cells.size(); ++i)
    {
        res << "\t" << cell_state_to_str(cells[i].state);
    }

    return res.str();
}

std::vector<std::vector<cv::Point> > Board::as_vector_of_vectors()
{
    std::vector<std::vector<cv::Point> > result;
    
    for (int i = 0; i < cells.size(); ++i)
    {
        result.push_back(cells[i].contours);
    }

    return result;
};

bool Board::save() 
{
    if (!cells.empty())
    {
        QApplication app(0,0);
        QString fileName = QFileDialog::getSaveFileName(0, "Save File", QDir::currentPath(), "XML files (*.xml)", new QString("XML files (*.xml)"));
        ROS_DEBUG_STREAM("File Name selected= " << fileName.toStdString());
        if(!fileName.isEmpty())
        {
            QFile output(fileName);
            if(!output.open(QIODevice::WriteOnly))
            {
                ROS_WARN("Error opening file to write cells data");
                return false;
            }

            QXmlStreamWriter stream(&output);
            stream.setAutoFormatting(true);
            stream.writeStartDocument();

            stream.writeStartElement("board");

            for (size_t i=0; i< cells.size();++i)
            {
                stream.writeStartElement("cell");
                stream.writeAttribute("id", QString::number(i));
                for (std::vector<cv::Point>::iterator it_vertex = cells[i].contours.begin(); it_vertex != cells[i].contours.end(); ++it_vertex) {
                    stream.writeEmptyElement("vertex");
                    stream.writeAttribute("x", QString::number(it_vertex->x));
                    stream.writeAttribute("y", QString::number(it_vertex->y));
                }
                stream.writeEndElement(); // cell
            }

            stream.writeEndElement(); // board
            stream.writeEndDocument();
            output.close();
        }
        else return false;
    }
    else
    {
        ROS_INFO("No cells in the board to be saved.");
        return false;
    }
    ROS_INFO("Board saved.");
    return true;
};

bool Board::load(std::string cells_param)
{
    ROS_DEBUG("Ready to read xml data of cells");

    std::string xml_cells;

    if (ros::param::get(cells_param, xml_cells)) //reading the xml data from the parameter server
    {
        cells.clear(); //cleaning all previous cells

        QXmlStreamReader xml;
        xml.addData(QString::fromStdString(xml_cells)); //adding the xml content to the input stream
        while(!xml.atEnd() && !xml.hasError()) {
            /* Read next element.*/
            QXmlStreamReader::TokenType token = xml.readNext();
            ROS_DEBUG_STREAM("name=" << xml.name().toString().toStdString());
            /* If token is just StartDocument, we'll go to next.*/
            if(token == QXmlStreamReader::StartDocument) {
                continue;
            }
            /* If token is StartElement, we'll see if we can read it.*/
            if(token == QXmlStreamReader::StartElement) {
                /* If it's named cell, we'll add a new cell to the board.*/
                if(xml.name() == "cell") {
                    cells.push_back(Cell());
                    continue;
                }
                /* If it's named vertex, we'll dig the information from there.*/
                if(xml.name() == "vertex") {
                    /* Let's get the attributes for vertex */
                    QXmlStreamAttributes attributes = xml.attributes();
                    /* Let's check that vertex has x and y attribute. */
                    if(attributes.hasAttribute("x") && attributes.hasAttribute("y")) {
                        /* We'll add it to the cell */
                        cells.back().contours.push_back(cv::Point(attributes.value("x").toString().toInt(), attributes.value("y").toString().toInt()));
                    }
                    else xml.raiseError("Vertex corrupted: x and/or y value is missing");
                }
            }
        }
        ROS_WARN_COND(xml.hasError(),"Error parsing xml data (l.%d, c.%d):%s", (int)xml.lineNumber(), (int)xml.columnNumber(), xml.errorString().toStdString().c_str());

        ROS_INFO("Xml data successfully loaded. %i cells loaded.", (int)cells.size());

        for (int i = 0; i < cells.size(); ++i)
        {
            ROS_INFO("Cell %i:  %s",i,cells[i].toString().c_str());
        }
        return true;
    }
    else
    {
        ROS_FATAL_STREAM("xml cell data not loaded!");
        return false;
    }
}

cv::Mat Board::mask_image(const cv::Mat &_src)
{
    cv::Mat mask = cv::Mat::zeros(_src.rows, _src.cols, CV_8UC1);

    // CV_FILLED fills the connected components found with white
    cv::drawContours(mask, as_vector_of_vectors(),
                     -1, cv::Scalar(255), CV_FILLED);  

    cv::Mat im_crop(_src.rows, _src.cols, CV_8UC3);                           
    im_crop.setTo(cv::Scalar(0));
    _src.copyTo(im_crop, mask);

    return im_crop;
}

cv::Mat hsv_threshold(const cv::Mat& _src, hsvColorRange _hsv)
{
    cv::Mat res = _src.clone();

    // If H.lower is higher than H.upper it means that we would like to
    // detect something in the range [0-upper] & [lower-180] (i.e. the red)
    // So the thresholded image will be made with two opencv calls to inRange
    // and then the two will be merged into one
    if (_hsv.H.min > _hsv.H.max)
    {
        cv::Mat resA = _src.clone();
        cv::Mat resB = _src.clone();

        cv::inRange(_src, cv::Scalar(         0, _hsv.S.min, _hsv.V.min),
                          cv::Scalar(_hsv.H.max, _hsv.S.max, _hsv.V.max), resA);
        cv::inRange(_src, cv::Scalar(_hsv.H.min, _hsv.S.min, _hsv.V.min),
                          cv::Scalar(       180, _hsv.S.max, _hsv.V.max), resB);

        cv::bitwise_or(resA,resB,res);
    }
    else
    {
        cv::inRange(_src, cv::Scalar(_hsv.get_hsv_min()),
                          cv::Scalar(_hsv.get_hsv_max()), res);
    }

    return res;
}

