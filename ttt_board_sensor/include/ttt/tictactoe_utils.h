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

typedef std::vector<cv::Point> t_Cell;    // vector of points delimiting a cell
typedef std::vector<t_Cell> t_Board;   // vector of cells, i.e. a vector of vectors of points

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
                    for (t_Cell::const_iterator it_vertex = cells[i].contours.begin(); it_vertex != cells[i].contours.end(); ++it_vertex) {
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
};

}

#endif // TTT_DEFINITIONS_H
