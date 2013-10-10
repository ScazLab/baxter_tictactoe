
#include "src/board_cells_delimitation.h"


namespace ttt
{

    bool CellDelimitation::remove_point(const cv::Point & p)
    {
        if (this->points.empty()) {
            ROS_DEBUG("Imposible to delte from an empty vector of points");
            return false;
        }

        for(t_Cell::iterator it_points = this->points.begin() ; it_points != this->points.end(); ++it_points)
        {

            if(it_points->x > (p.x-CellDelimitation::POINT_RADIUS) && it_points->x < (p.x+CellDelimitation::POINT_RADIUS) &&
                    it_points->y > (p.y-CellDelimitation::POINT_RADIUS) && it_points->y < (p.y+CellDelimitation::POINT_RADIUS))
            {
                this->points.erase(it_points);
                return true;
            }
        }
        return false;
    }

    bool CellDelimitation::remove_cell(const cv::Point & p)
    {
        if (this->board.empty()) {
            ROS_DEBUG("Imposible to delte from an empty board of cells");
            return false;
        }

        for(t_Board::iterator it_cell = this->board.begin() ; it_cell != this->board.end(); ++it_cell)
        {
            if (cv::pointPolygonTest(*it_cell,p,false)>0) // the point is inside the polygon
            {
                this->board.erase(it_cell);
                return true;
            }
        }
        return false;
    }

    void CellDelimitation::show_how_to(cv::Mat& img)
    {
        cv::putText(img,"DOUBLE-LEFT-CLICK to add/remove points and cells", cv::Point(20,400),cv::FONT_HERSHEY_SIMPLEX,0.6,cv::Scalar(255,255,0));
        cv::putText(img,"Press SPACE bar to add the cell to the board", cv::Point(20,420),cv::FONT_HERSHEY_SIMPLEX,0.6,cv::Scalar(255,255,0));
        cv::putText(img,"Press 'R' to show the results", cv::Point(20,440),cv::FONT_HERSHEY_SIMPLEX,0.6,cv::Scalar(255,255,0));
        cv::putText(img,"Press 'S' to save the board", cv::Point(20,460),cv::FONT_HERSHEY_SIMPLEX,0.6,cv::Scalar(255,255,0));
    }

    // thanks to http://bytefish.de/blog/extracting_contours_with_opencv/
    void CellDelimitation::cropping_cells(cv_bridge::CvImageConstPtr& cv_cp_img,const t_Board& a_board)
    {
        cv::Mat mask = cv::Mat::zeros(cv_cp_img->image.rows, cv_cp_img->image.cols, CV_8UC1);
        cv::drawContours(mask, a_board, -1, cv::Scalar(255), CV_FILLED);        // CV_FILLED fills the connected components found with white (white RGB value = 255,255,255)

        cv::Mat im_crop(cv_cp_img->image.rows, cv_cp_img->image.cols, CV_8UC3); // let's create a new 8-bit 3-channel image with the same dimensions
        im_crop.setTo(cv::Scalar(0));                                           // we fill the new image with a color, in this case we set background to black.
        cv_cp_img->image.copyTo(im_crop, mask);                                 // copy just the elements from the original image indicated by the non-zero elements from mask to crop
        cv::normalize(mask.clone(), mask, 0.0, 255.0, CV_MINMAX, CV_8UC1);      // normalize so imwrite(...)/imshow(...) shows the mask correctly!

        // show the images
        cv::imshow("original", cv_cp_img->image);
        cv::imshow("mask", mask);
        cv::imshow("cropped", im_crop);
    }

    CellDelimitation::CellDelimitation()
        : it_(nh_),POINT_RADIUS(5)
    {
        image_sub_ = it_.subscribe("in", 1, &CellDelimitation::imageCb, this);

        cv::namedWindow(CellDelimitation::WINDOW);
    }

    CellDelimitation::~CellDelimitation()
    {
        cv::destroyWindow(CellDelimitation::WINDOW);
    }

    /* mouse event handler function */
    void CellDelimitation::onMouse( int event, int x, int y, int, void* param)
    {
        if( event != cv::EVENT_LBUTTONDBLCLK )
            return;

        cv::Point p = cv::Point(x,y);
        CellDelimitation * img_conv = (CellDelimitation*) param;

        // If the point is inside an already defined cell, the cell is removed
        if(img_conv->remove_cell(p)) {
            ROS_INFO_STREAM("Cell deleted. Remaining " << img_conv->board.size() << " cells");
            return;
        }

        // If the point correspond to a vertex, the vertex is removed
        if(img_conv->remove_point(p)) {
            ROS_INFO_STREAM("Point deleted. Remaining " << img_conv->points.size() << " points");
        }
        // If the point is new, it is added to the current cell
        else {
            img_conv->points.push_back(p);
            ROS_INFO_STREAM("Point added: " << p.x << " , " << p.y << ". Total points in this cell: "  << img_conv->points.size() );
        }

        return;
    }


    void CellDelimitation::imageCb(const sensor_msgs::ImageConstPtr& msg)
    {
        //converting ROS image format to opencv image format
        cv_bridge::CvImageConstPtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvShare(msg, enc::BGR8);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        cv::Mat img_aux = cv_ptr->image.clone();

        // how-to on screen
        this->show_how_to(img_aux);

        // drawing all points of the current cell
        for (t_Cell::iterator it_drawing = this->points.begin();it_drawing != this->points.end();++it_drawing) {
            cv::circle(img_aux,*it_drawing,CellDelimitation::POINT_RADIUS,cv::Scalar(0,0,255),-1);
        }

        // drawing all cells of the board game
        for (t_Board::iterator it_cell = this->board.begin(); it_cell != this->board.end(); ++it_cell) {
            cv::fillConvexPoly(img_aux,it_cell->data(),it_cell->size(), cv::Scalar(0,0, 255));
        }
        cv::drawContours(img_aux,this->board,-1, cv::Scalar(123,125,0),2); // drawing the borders in a different color

        cv::setMouseCallback(CellDelimitation::WINDOW, onMouse, this);
        cv::imshow(CellDelimitation::WINDOW, img_aux);


        int c = cv::waitKey(3);
        if( (c & 255) == 27 ) // ESC key pressed
        {
            ROS_INFO_STREAM("Exiting with ESC key ..." << this->board.size() << " cells selected");
            ros::shutdown();
        }
        else if ((char)c == ' ') {  // SPACE bar pressed
            if(!this->points.empty())
            {
                this->board.push_back(this->points);
                this->points.clear();
                ROS_INFO("Add points to the new cell");
            }
            else ROS_INFO_STREAM("The current cell is empty. Add points to it before you create a new one");
        }
        else if ((char)c=='r') { // 'R' key pressed
            this->cropping_cells(cv_ptr, this->board);
        }
        else if ((char)c=='s') { // 'S' key pressed
            Cells::save_to_file(this->board);
        }

    }    

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "cell_delimitation");
    ttt::CellDelimitation cd;
    ros::spin();
    return 0;
}
