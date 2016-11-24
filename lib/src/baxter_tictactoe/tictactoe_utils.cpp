#include "baxter_tictactoe/tictactoe_utils.h"

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <QFile>
#include <QXmlStreamWriter>
#include <QXmlStreamReader>
#include <QFileDialog>
#include <QApplication>

using namespace std;
using namespace ttt;

cv::Mat ttt::hsvThreshold(const cv::Mat& _src, hsvColorRange _hsv)
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

Cell::Cell(std::string _s, int _ar, int _ab) :
           state(_s), area_red(_ar), area_blue(_ab)
{
    contour.clear();
}

Cell::Cell(Contour _c, std::string _s, int _ar, int _ab) :
           contour(_c), state(_s), area_red(_ar), area_blue(_ab)
{

}

void Cell::resetCell()
{
    state     = "empty";
    area_red  =       0;
    area_blue =       0;
    contour.clear();
}

bool Cell::computeState()
{
    if (area_red || area_blue)
    {
        area_red>area_blue?setState(COL_RED):setState(COL_BLUE);
        return true;
    }
    return false;
}

cv::Mat Cell::maskImage(const cv::Mat &_src)
{
    cv::Mat mask = cv::Mat::zeros(_src.rows, _src.cols, CV_8UC1);

    // CV_FILLED fills the connected components found with white
    cv::drawContours(mask, std::vector<std::vector<cv::Point> >(1,contour),
                                            -1, cv::Scalar(255), CV_FILLED);

    cv::Mat im_crop(_src.rows, _src.cols, CV_8UC3);
    im_crop.setTo(cv::Scalar(0));
    _src.copyTo(im_crop, mask);

    return im_crop;
}

cv::Point Cell::getCentroid()
{
    cv::Point centroid(0,0);

    if(contour.size() > 0)
    {
        cv::Moments mom;
        mom = cv::moments(contour, false);
        centroid = cv::Point( int(mom.m10/mom.m00) , int(mom.m01/mom.m00) );
    }

    return centroid;
}

int Cell::getContourArea()
{
    if (contour.size() > 0)  return cv::moments(getContour(),false).m00;

    return 0;
}

string Cell::toString()
{
    stringstream res;

    res<<"State:"<<getState()<<"\t";
    res<<"Red  Area: "<<area_red<<"\t";
    res<<"Blue Area: "<<area_blue<<"\t";

    if (contour.size()==0)
    {
        res << "Points:\tNONE;\t";
    }
    else
    {
        res << "Points:\t";
        for (int i = 0; i < contour.size(); ++i)
        {
            res <<"["<<contour[i].x<<"  "<<contour[i].y<<"]\t";
        }
    }

    return res.str();
}

/**************************************************************************/
/**                                 BOARD                                **/
/**************************************************************************/

bool Board::resetState()
{
    if (getNumCells()==0) return false;

    for (int i = 0; i < getNumCells(); ++i)
    {
        cells[i].resetCell();
    }

    return true;
}

bool Board::computeState()
{
    if (getNumCells()==0) return false;

    for (int i = 0; i < getNumCells(); ++i)
    {
        cells[i].computeState();
    }

    return true;
}

void Board::fromMsgBoard(const baxter_tictactoe::MsgBoard &msgb)
{
    resetState();

    for (int i = 0; i < msgb.cells.size(); ++i)
    {
        if      (msgb.cells[i].state == COL_RED  ) addCell(Cell(COL_RED  , 1, 0));
        else if (msgb.cells[i].state == COL_BLUE ) addCell(Cell(COL_BLUE , 0, 1));
        else if (msgb.cells[i].state == COL_EMPTY) addCell(Cell(COL_EMPTY, 0, 0));
        else
        {
            ROS_WARN("MsgBoard cell state %s not allowed!", msgb.cells[i].state.c_str());
        }
    }
}

baxter_tictactoe::MsgBoard Board::toMsgBoard()
{
    baxter_tictactoe::MsgBoard res;
    res.header = std_msgs::Header();

    if (getNumCells() == 0 || getNumCells() != res.cells.size())
    {
        for (int i = 0; i < res.cells.size(); ++i)
        {
            res.cells[i].state = COL_EMPTY;
        }

        if (getNumCells() != res.cells.size())
        {
            ROS_WARN("Number of cells in board [%i] different from those in MsgBoard [%lu].",
                                                            getNumCells(), res.cells.size());
        }
    }
    else if (getNumCells() == res.cells.size())
    {
        for (int i = 0; i < res.cells.size(); ++i)
        {
            res.cells[i].state = getCellState(i);
        }
    }

    return res;
}

string Board::toString()
{
    if (getNumCells()==0)   return "";

    stringstream res;
    res << cells[0].getState();
    for (int i = 1; i < getNumCells(); ++i)
    {
        res << "\t" << cells[i].getState();
    }

    return res.str();
}

Contours Board::getContours()
{
    Contours result;

    for (int i = 0; i < getNumCells(); ++i)
    {
        result.push_back(getCellContour(i));
    }

    return result;
};

cv::Mat Board::maskImage(const cv::Mat &_src)
{
    cv::Mat mask = cv::Mat::zeros(_src.rows, _src.cols, CV_8UC1);

    // CV_FILLED fills the connected components found with white
    cv::drawContours(mask, getContours(), -1, cv::Scalar(255), CV_FILLED);

    cv::Mat im_crop(_src.rows, _src.cols, CV_8UC3);
    im_crop.setTo(cv::Scalar(0));
    _src.copyTo(im_crop, mask);

    return im_crop;
}

