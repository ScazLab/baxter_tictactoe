#ifndef TTT_DEFINITIONS_H
#define TTT_DEFINITIONS_H

#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <QFile>
#include <QXmlStreamWriter>
#include <QXmlStreamReader>
#include <QFileDialog>
#include <QApplication>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/core/core.hpp>

#include "ttt_board_sensor/ttt_board.h"

namespace enc = sensor_msgs::image_encodings;

namespace ttt
{

// Used to determine the three possible states of a cell.
// Undefined is just used when it is created and non state has been assigned.
typedef enum {empty=ttt_board_sensor::ttt_board::EMPTY,
              blue=ttt_board_sensor::ttt_board::BLUE,
              red=ttt_board_sensor::ttt_board::RED,
              undefined=ttt_board_sensor::ttt_board::UNDEFINED} cellState;

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

const std::size_t NUMBER_OF_CELLS=9;
const std::string CELLS_DATA_PARAM_NAME="/baxter_tictactoe/board_file";

struct Cell
{
public:
    std::vector<cv::Point> contours;
    cellState state;

public:
    Cell() {};
    Cell(std::vector<cv::Point> _vec) : contours(_vec) {};
    ~Cell() {};

    std::vector<cv::Point> get_contours() { return contours; };
    
    /**
    Returns a mask for a cell, i.e. a new image that preserves the size of the original one, but 
    keeps only the portion delimited by the cell (the rest is set to black)
    @param img  the original image, where the mask will be extracted from.
    @param cell the cell defining the mask.
    @return     the mask for a cell. It has the same size than the original image.
     */
    cv::Mat mask_image(const cv::Mat img)
    {
        cv::Mat mask = cv::Mat::zeros(img.rows, img.cols, CV_8UC1);

        // CV_FILLED fills the connected components found with white (white RGB value = 255,255,255)
        cv::drawContours(mask, std::vector<std::vector<cv::Point> >(1,contours), -1, cv::Scalar(255), CV_FILLED);  

        cv::Mat im_crop(img.rows, img.cols, CV_8UC3);                           
        im_crop.setTo(cv::Scalar(0));
        img.copyTo(im_crop, mask);                  

        // normalize so imwrite(...)/imshow(...) shows the mask correctly
        //cv::normalize(mask.clone(), mask, 0.0, 255.0, CV_MINMAX, CV_8UC1);

        // show the images
        // cv::imshow("original", img);
        // cv::imshow("mask", mask);
        // cv::imshow("masked cell", im_crop);

        return im_crop;
    }

    /**
    Computes the centroid of a cell.
    @param cell the cell whose centroid is going to be computed
    @param centroid the central point.
    @return true/false if success/failure.
    */
    bool get_cell_centroid(cv::Point& centroid)
    {
        uint sumX = 0, sumY = 0;
        size_t size = contours.size();
        centroid.x=centroid.y=0;
        if(size > 0){
            for (std::vector<cv::Point>::iterator it_point = contours.begin(); it_point != contours.end(); ++it_point) {
                sumX += it_point->x;
                sumY += it_point->y;
            }
            // TODO through exception if size <= 0
            centroid.x = sumX/size;
            centroid.y = sumY/size;
            return true;
        }
        else
            return false;
    }
};

struct Board
{
public: 
    std::vector<ttt::Cell> cells;

public:
    Board()  {};
    ~Board() {};

    std::vector<std::vector<cv::Point> > as_vector_of_vectors()
    {
        std::vector<std::vector<cv::Point> > result;
        
        for (int i = 0; i < cells.size(); ++i)
        {
            result.push_back(cells[i].contours);
        }

        return result;
    };

    /**
    Saves the board to file (whose name will be specified with a dialog window).
    The format of the file is as follows:
        <board>
            <cell id="0">
                <vertex x="195" y="50"/>
                [...]
            </cell>
            [...]
        </board>
    @param board where all the Cells are stored
    @return true/false if success/failure.
     */
    bool save() 
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
        return true;
    };

    /**
    Reads the contours of the cells from the parameter server, in a param named cells_param.
    @param board where the Cells are going to be stored
    @param cells_param the name of the parameter where the raw data from the cells is stored. This data is formatted as a xml file.
    The format of the data is as follows:
            <board>
                <cell id="0">
                    <vertex x="195" y="50"/>
                    [...]
                </cell>
                [...]
            </board>
    @return true/false if success/failure.
     */
    bool load(std::string cells_param)
    {
        ROS_DEBUG("Ready to read xml data of cells");

        std::string xml_cells;

        if (ros::param::get(cells_param, xml_cells)) //reading the xml data from the parameter server
        {
            ROS_INFO("xml cell data successfully loaded");

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

            ROS_INFO("Xml data successfully loaded. %i cells loaded", (int)cells.size());
            return true;
        }
        else
        {
            ROS_FATAL_STREAM("xml cell data not loaded!");
            return false;
        }
    }
};

}

#endif // TTT_DEFINITIONS_H
