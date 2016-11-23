#include "baxterTictactoe/tictactoe_utils.h"

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <QFile>
#include <QXmlStreamWriter>
#include <QXmlStreamReader>
#include <QFileDialog>
#include <QApplication>

using namespace std;
using namespace ttt;

std::string ttt::cell_state_to_str(cellState c_s)
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

cv::Mat ttt::hsv_threshold(const cv::Mat& _src, hsvColorRange _hsv)
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

Cell::Cell()
{
    state=empty;
    cell_area_red=0;
    cell_area_blue=0;
    contours.clear();
}

Cell::Cell(std::vector<cv::Point> _vec) : contours(_vec)
{
    state=empty;
    cell_area_red=0;
    cell_area_blue=0;
}

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

cv::Point Cell::get_centroid()
{
    cv::Point centroid(0,0);

    if(contours.size() > 0)
    {
        cv::Moments mom;
        mom = cv::moments(contours, false);
        centroid = cv::Point( int(mom.m10/mom.m00) , int(mom.m01/mom.m00) );
    }

    return centroid;
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
    if (cells_size()==0)
    {
        return false;
    }

    for (int i = 0; i < cells_size(); ++i)
    {
        cells[i].state      = empty;
        cells[i].cell_area_red  = 0;
        cells[i].cell_area_blue = 0;
    }

    return true;
}

string Board::stateToString()
{
    if (cells_size()==0)
    {
        return "";
    }

    stringstream res;
    res << cell_state_to_str(cells[0].state);
    for (int i = 1; i < cells_size(); ++i)
    {
        res << "\t" << cell_state_to_str(cells[i].state);
    }

    return res.str();
}

std::vector<std::vector<cv::Point> > Board::as_vector_of_vectors()
{
    std::vector<std::vector<cv::Point> > result;

    for (int i = 0; i < cells_size(); ++i)
    {
        result.push_back(cells[i].contours);
    }

    return result;
};

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

