#include "baxter_tictactoe/tictactoe_utils.h"

#include <stdlib.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <QFile>
#include <QXmlStreamWriter>
#include <QXmlStreamReader>
#include <QFileDialog>
#include <QApplication>

using namespace std;
using namespace baxter_tictactoe;

/**************************************************************************/
/**                        CELL                                          **/
/**************************************************************************/

Cell::Cell(string _s, int _ar, int _ab) :
           state(_s), area_red(_ar), area_blue(_ab)
{
    contour.clear();
    checkIntegrity();
}

Cell::Cell(Contour _c, string _s, int _ar, int _ab) :
           contour(_c), state(_s), area_red(_ar), area_blue(_ab)
{
    checkIntegrity();
}

Cell::Cell(const Cell &_c) :
           contour(_c.contour), state(_c.state),
           area_red(_c.area_red), area_blue(_c.area_blue)
{
    checkIntegrity();
}

bool Cell::checkIntegrity()
{
    // check for integrity of state
    if (state != COL_RED && state != COL_BLUE && state != COL_EMPTY)
    {
        resetState();
        return false;
    }

    // check for integrity for state and area colors
    if (state == COL_EMPTY)
    {
        if (area_red == 0 && area_blue == 0) { return true; }
        else
        {
            resetState();
            return false;
        }
    }
    else if (state == COL_RED)
    {
        if (area_red > area_blue) { return true; }
        else
        {
            area_red = area_blue + 1;
            return false;
        }
    }
    else if (state == COL_BLUE)
    {
        if (area_blue > area_red) { return true; }
        else
        {
            area_blue = area_red + 1;
        }
    }
    return true;
}

Cell& Cell::operator=(const Cell& _c)
{
    // self-assignment check
    if (this != &_c)
    {
        contour   = _c.contour;
        state     = _c.state;
        area_red  = _c.area_red;
        area_blue = _c.area_blue;
    }

    return *this;
}

bool Cell::operator==(const Cell &_c) const
{
    return state == _c.state;
}

bool Cell::operator!=(const Cell &_c) const
{
    return not (*this == _c);
}

bool Cell::resetState()
{
    state     = COL_EMPTY;
    area_red  =         0;
    area_blue =         0;

    return true;
}

bool Cell::resetCell()
{
    resetState();
    contour.clear();

    return true;
}

bool Cell::computeState()
{
    if (area_red || area_blue)
    {
        area_red>area_blue?setState(COL_RED):setState(COL_BLUE);
        return true;
    }

    setState(COL_EMPTY);
    return false;
}

cv::Mat Cell::maskImage(const cv::Mat &_src)
{
    cv::Mat mask = cv::Mat::zeros(_src.rows, _src.cols, CV_8UC1);

    // CV_FILLED fills the connected components found with white
    cv::drawContours(mask, vector<vector<cv::Point> >(1,contour),
                                            -1, cv::Scalar(255), CV_FILLED);

    cv::Mat im_crop(_src.rows, _src.cols, CV_8UC3);
    im_crop.setTo(cv::Scalar(0));
    _src.copyTo(im_crop, mask);

    return im_crop;
}

cv::Point Cell::getCentroid()
{
    cv::Point centroid(0,0);

    if (contour.size() > 0)
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

bool Cell::setState(const string& _s)
{
    if (_s == COL_RED || _s == COL_BLUE || _s == COL_EMPTY)
    {
        state = _s;

        // Ensure consistency of the number pixels w.r.t. the state
        if (_s == COL_RED && getRedArea() < getBlueArea())
        {
            setRedArea(getBlueArea()+1);
        }
        else if (_s == COL_BLUE && getBlueArea() < getRedArea())
        {
            setBlueArea(getRedArea()+1);
        }
        else if (_s == COL_EMPTY)
        {
            resetState();
        }

        return true;
    }

    return false;
}

string Cell::toString()
{
    stringstream res;

    res<<"State: "<<getState()<<"\t";
    res<<"Red  Area: "<<area_red<<"\t";
    res<<"Blue Area: "<<area_blue<<"\t";

    if (contour.size()==0)
    {
        res << "Points:\tNONE;\t";
    }
    else
    {
        res << "Points:\t";
        for (size_t i = 0; i < contour.size(); ++i)
        {
            res <<"["<<contour[i].x<<"  "<<contour[i].y<<"]\t";
        }
    }

    return res.str();
}

/**************************************************************************/
/**                                 BOARD                                **/
/**************************************************************************/
Board::Board()
{

}

Board::Board(size_t n_cells)
{
    for (size_t i = 0; i < n_cells; ++i)
    {
        addCell(Cell());
    }
}

Board::Board(const Board &_b) : cells(_b.cells)
{

}

Board& Board::operator=(const Board& _b)
{
    // self-assignment check
    if (this != &_b)
    {
        resetBoard();

        for (size_t i = 0; i < _b.cells.size(); ++i)
        {
            addCell(Cell(_b.cells[i]));
        }
    }

    return *this;
}

bool Board::operator==(const Board &_b) const
{
    if (cells.size() != _b.cells.size())  { return false; };

    for (size_t i = 0; i < _b.cells.size(); ++i)
    {
        if (cells[i] != _b.cells[i])      { return false; };
    }

    return true;
}

bool Board::operator!=(const Board &_b) const
{
    return not (*this == _b);
}

bool Board::addCell(const Cell& _c)
{
    cells.push_back(_c);

    return true;
}

bool Board::resetCellStates()
{
    if (getNumCells()==0) { return false; }

    for (size_t i = 0; i < getNumCells(); ++i)
    {
        cells[i].resetState();
    }

    return true;
}

bool Board::resetCells()
{
    if (getNumCells()==0) { return false; }

    for (size_t i = 0; i < getNumCells(); ++i)
    {
        cells[i].resetCell();
    }

    return true;
}

bool Board::resetBoard()
{
    cells.clear();

    return true;
}

bool Board::computeState()
{
    if (getNumCells()==0) { return false; }

    for (size_t i = 0; i < getNumCells(); ++i)
    {
        cells[i].computeState();
    }

    return true;
}

bool Board::isFull()
{
    for (size_t i = 0; i < getNumCells(); i++)
    {
        if (getCellState(i)==COL_EMPTY) { return false; }
    }
    return true;
}

bool Board::isEmpty()
{
    for (size_t i = 0; i < getNumCells(); i++)
    {
        if (getCellState(i)==COL_RED || getCellState(i)==COL_BLUE) { return false; }
    }
    return true;
}

size_t Board::getNumTokens()
{
    return getNumTokens(COL_RED) + getNumTokens(COL_BLUE);
}

size_t Board::getNumTokens(const string& _col)
{
    if (_col != COL_RED && _col != COL_BLUE) { return 0; }

    size_t cnt = 0;

    for (size_t i = 0; i < getNumCells(); i++)
    {
        if (getCellState(i)==_col) ++cnt;
    }

    return cnt;
}

bool Board::isOneTokenAddedRemoved(const Board& _new)
{
    if (getNumCells() != _new.cells.size())  { return false; };

    size_t sum = 0;

    for (size_t i = 0; i < getNumCells(); i++)
    {
        Cell c = _new.cells[i];

        if (getCell(i) != c)
        {
            if ( getCell(i).getState() == COL_EMPTY ||
                          c.getState() == COL_EMPTY)
            {
                sum += 1;
            }
            else
            {
                sum += 2;
            }
        }
    }

    return sum == 1;
}

bool Board::isOneTokenAdded(const Board& _new)
{
    if (not isOneTokenAddedRemoved(_new)) { return false; }

    return isOneTokenAdded(_new, COL_RED) || isOneTokenAdded(_new, COL_BLUE);
}

bool Board::isOneTokenAdded(const Board &_new, const string& _col)
{
    if (not isOneTokenAddedRemoved(_new)) { return false; }
    if (_col!=COL_BLUE && _col!=COL_RED)  { return false; }

    Board b(_new);
    return b.getNumTokens(_col) > getNumTokens(_col);
}

bool Board::isOneTokenRemoved(const Board& _new)
{
    if (not isOneTokenAddedRemoved(_new)) { return false; }

    return isOneTokenRemoved(_new, COL_RED) || isOneTokenRemoved(_new, COL_BLUE);
}

bool Board::isOneTokenRemoved(const Board &_new, const string& _col)
{
    if (not isOneTokenAddedRemoved(_new)) { return false; }
    if (_col!=COL_BLUE && _col!=COL_RED)  { return false; }

    Board b(_new);
    return b.getNumTokens(_col) < getNumTokens(_col);
}

bool Board::threeInARow()
{
    return threeInARow(COL_RED) || threeInARow(COL_BLUE);
}

bool Board::threeInARow(const string& _col)
{
    if (_col!=COL_BLUE && _col!=COL_RED) { return false; }

    // Rows
    if (getCellState(0)==_col && getCellState(1)==_col && getCellState(2)==_col) { return true; }
    if (getCellState(3)==_col && getCellState(4)==_col && getCellState(5)==_col) { return true; }
    if (getCellState(6)==_col && getCellState(7)==_col && getCellState(8)==_col) { return true; }
    // Columns
    if (getCellState(0)==_col && getCellState(3)==_col && getCellState(6)==_col) { return true; }
    if (getCellState(1)==_col && getCellState(4)==_col && getCellState(7)==_col) { return true; }
    if (getCellState(2)==_col && getCellState(5)==_col && getCellState(8)==_col) { return true; }
    // Diagonals
    if (getCellState(0)==_col && getCellState(4)==_col && getCellState(8)==_col) { return true; }
    if (getCellState(2)==_col && getCellState(4)==_col && getCellState(6)==_col) { return true; }

    return false;
}

void Board::fromMsgBoard(const baxter_tictactoe::MsgBoard &msgb)
{
    resetBoard();

    for (size_t i = 0; i < msgb.cells.size(); ++i)
    {
        // We want to keep the cell self-consistent. To this end, we add a fake
        // non empty area if the cell is red or blue colored.
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
        for (size_t i = 0; i < res.cells.size(); ++i)
        {
            res.cells[i].state = COL_EMPTY;
        }

        if (getNumCells() != res.cells.size())
        {
            ROS_WARN("Number of cells in board [%lu] different from those in MsgBoard [%lu].",
                                                             getNumCells(), res.cells.size());
        }
    }
    else if (getNumCells() == res.cells.size())
    {
        for (size_t i = 0; i < res.cells.size(); ++i)
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
    for (size_t i = 1; i < getNumCells(); ++i)
    {
        res << "\t" << cells[i].getState();
    }

    return res.str();
}

Contours Board::getContours()
{
    Contours result;

    for (size_t i = 0; i < getNumCells(); ++i)
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

bool Board::setCellState(size_t i, const string& _s)
{
    if (i >= getNumCells()) { return false; }

    return cells[i].setState(_s);
}

bool Board::setCell(size_t i, const Cell& _c)
{
    cells[i] = _c;

    return true;
}

Board::~Board()
{

}
