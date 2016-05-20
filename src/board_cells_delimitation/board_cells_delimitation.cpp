#include "board_cells_delimitation.h"


namespace ttt
{

    cellDelimitation::cellDelimitation() : image_transport(node_handle), point_radius(5), window_name("Cell Delimitation")
    {
        image_subscriber = image_transport.subscribe("in", 1, &cellDelimitation::imageCallback, this);

        cv::namedWindow(cellDelimitation::window_name);
    }

    bool cellDelimitation::remove_point(const cv::Point & p)
    {
        for(std::vector<cv::Point>::iterator it_points = cell.contours.begin() ; it_points != cell.contours.end(); ++it_points)
        {

            if(it_points->x > (p.x-cellDelimitation::point_radius) && it_points->x < (p.x+cellDelimitation::point_radius) &&
                    it_points->y > (p.y-cellDelimitation::point_radius) && it_points->y < (p.y+cellDelimitation::point_radius))
            {
                cell.contours.erase(it_points);
                return true;
            }
        }
        return false;
    }

    bool cellDelimitation::point_is_inside_cell(const cv::Point & p)
    {
        for(std::vector<Cell>::iterator it_cell = board.cells.begin() ; it_cell != board.cells.end(); ++it_cell)
        {
            if (cv::pointPolygonTest(it_cell->contours,p,false)>0) // the point is inside the polygon
            {
                board.cells.erase(it_cell);
                return true;
            }
        }
        return false;
    }

    void cellDelimitation::show_tutorial(cv::Mat& img)
    {
        cv::putText(img,"LEFT-CLICK to add/remove points and cells",    cv::Point(20,385),cv::FONT_HERSHEY_PLAIN,0.9,cv::Scalar(255,255,0));
        cv::putText(img,"Press SPACE bar to add the cell to the board", cv::Point(20,400),cv::FONT_HERSHEY_PLAIN,0.9,cv::Scalar(255,255,0));
        cv::putText(img,"LEFT-CLICK inside a cell to remove it",        cv::Point(20,415),cv::FONT_HERSHEY_PLAIN,0.9,cv::Scalar(255,255,0));
        cv::putText(img,"LEFT-CLICK on a point to remove it",           cv::Point(20,430),cv::FONT_HERSHEY_PLAIN,0.9,cv::Scalar(255,255,0));
        cv::putText(img,"Press 'R' to show the results",                cv::Point(20,445),cv::FONT_HERSHEY_PLAIN,0.9,cv::Scalar(255,255,0));
        cv::putText(img,"Press 'S' to save the board",                  cv::Point(20,460),cv::FONT_HERSHEY_PLAIN,0.9,cv::Scalar(255,255,0));
    }

    // thanks to http://bytefish.de/blog/extracting_contours_with_opencv/
    void cellDelimitation::crop_cells(cv_bridge::CvImageConstPtr& cv_cp_img, const std::vector<std::vector<cv::Point> > cntrs)
    {
        cv::Mat mask = cv::Mat::zeros(cv_cp_img->image.rows, cv_cp_img->image.cols, CV_8UC1);
        cv::drawContours(mask, cntrs, -1, cv::Scalar(255), CV_FILLED);        // CV_FILLED fills the connected components found with white (white RGB value = 255,255,255)

        cv::Mat im_crop(cv_cp_img->image.rows, cv_cp_img->image.cols, CV_8UC3); // let's create a new 8-bit 3-channel image with the same dimensions
        im_crop.setTo(cv::Scalar(0));                                           // we fill the new image with a color, in this case we set background to black.
        cv_cp_img->image.copyTo(im_crop, mask);                                 // copy just the elements from the original image indicated by the non-zero elements from mask to crop
        cv::normalize(mask.clone(), mask, 0.0, 255.0, CV_MINMAX, CV_8UC1);      // normalize so imwrite(...)/imshow(...) shows the mask correctly!

        // show the images
        cv::imshow("original", cv_cp_img->image);
        cv::imshow("mask", mask);
        cv::imshow("cropped", im_crop);
    }

    cellDelimitation::~cellDelimitation()
    {
        cv::destroyWindow(cellDelimitation::window_name);
    }

    /* mouse event handler function */
    void cellDelimitation::onMouseClick( int event, int x, int y, int, void* param)
    {
        if( event != cv::EVENT_LBUTTONDOWN )
            return;

        cv::Point p = cv::Point(x,y);
        cellDelimitation * img_conv = (cellDelimitation*) param;

        // If the point is inside an already defined cell, the cell is removed
        if(img_conv->point_is_inside_cell(p)) {
            ROS_INFO_STREAM("Cell removed because point was in cell. Remaining " << img_conv->board.cells.size() << " cells");
            return;
        }

        // If the point corresponds to an already selected point, the point is removed
        if(img_conv->remove_point(p)) {
            ROS_INFO_STREAM("Point removed. Remaining " << img_conv->cell.contours.size() << " points");
        }
        // If the point is new, it is added to the current cell
        else {
            img_conv->cell.contours.push_back(p);
            ROS_INFO_STREAM("Point added: " << p.x << " , " << p.y << ". Total points in this cell: "  << img_conv->cell.contours.size() );
        }

        return;
    }


    void cellDelimitation::imageCallback(const sensor_msgs::ImageConstPtr& msg)
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
        show_tutorial(img_aux);

        // drawing all points of the current cell
        for (std::vector<cv::Point>::iterator image_transportdrawing = cell.contours.begin();image_transportdrawing != cell.contours.end();++image_transportdrawing) {
            cv::circle(img_aux,*image_transportdrawing,cellDelimitation::point_radius,cv::Scalar(0,0,255),-1);
        }

        // drawing all cells of the board game
        for (std::vector<Cell>::iterator it_cell = board.cells.begin(); it_cell != board.cells.end(); ++it_cell) {
            cv::fillConvexPoly(img_aux,it_cell->contours.data(),it_cell->contours.size(), cv::Scalar(0,0, 255));
        }
        cv::drawContours(img_aux, board.as_vector_of_vectors(),-1, cv::Scalar(123,125,0),2); // drawing the borders in a different color

        cv::setMouseCallback(cellDelimitation::window_name, onMouseClick, this);
        cv::imshow(cellDelimitation::window_name, img_aux);

        int c = cv::waitKey(3);
        if( (c & 255) == 27 ) // ESC key pressed
        {
            ROS_INFO_STREAM("Exiting with ESC key ..." << board.cells.size() << " cells selected");
            ros::shutdown();
        }
        else if ((char)c == ' ') {  // SPACE bar pressed
            if(!cell.contours.empty())
            {
                board.cells.push_back(cell);
                cell.contours.clear();
                ROS_INFO("Add points to the new cell"); 
            }
            else ROS_INFO_STREAM("The current cell is empty. Add points to it before you create a new one");
        }
        else if ((char)c=='r') { // 'R' key pressed
            std::vector<std::vector<cv::Point> > cntrs = board.as_vector_of_vectors();
            crop_cells(cv_ptr, cntrs);
        }
        else if ((char)c=='s') { // 'S' key pressed
            board.save();
        }

    }    

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "cell_delimitation");
    ttt::cellDelimitation cd;
    ros::spin();
    return 0;
}

/*
#include "board_cells_delimitation.h"


namespace ttt
{

    cellDelimitation::cellDelimitation() : image_transport(node_handle), point_radius(5), window_name("Cell Delimitation")
    {
        image_subscriber = image_transport.subscribe("in", 1, &cellDelimitation::imageCallback, this);

        cv::namedWindow(cellDelimitation::window_name);
    }

    bool cellDelimitation::remove_point(const cv::Point & p)
    {
        for(std::vector<cv::Point>::iterator it_points = cell.contours.begin() ; it_points != cell.contours.end(); ++it_points)
        {

            if(it_points->x > (p.x-cellDelimitation::point_radius) && it_points->x < (p.x+cellDelimitation::point_radius) &&
                    it_points->y > (p.y-cellDelimitation::point_radius) && it_points->y < (p.y+cellDelimitation::point_radius))
            {
                cell.contours.erase(it_points);
                return true;
            }
        }
        return false;
    }

    bool cellDelimitation::point_is_inside_cell(const cv::Point & p)
    {
        for(std::vector<Cell>::iterator it_cell = board.cells.begin() ; it_cell != board.cells.end(); ++it_cell)
        {
            if (cv::pointPolygonTest(it_cell->contours,p,false)>0) // the point is inside the polygon
            {
                board.cells.erase(it_cell);
                return true;
            }
        }
        return false;
    }

    void cellDelimitation::show_tutorial(cv::Mat& img)
    {
        cv::putText(img,"LEFT-CLICK to add/remove points and cells",    cv::Point(20,385),cv::FONT_HERSHEY_PLAIN,0.9,cv::Scalar(255,255,0));
        cv::putText(img,"Press SPACE bar to add the cell to the board", cv::Point(20,400),cv::FONT_HERSHEY_PLAIN,0.9,cv::Scalar(255,255,0));
        cv::putText(img,"LEFT-CLICK inside a cell to remove it",        cv::Point(20,415),cv::FONT_HERSHEY_PLAIN,0.9,cv::Scalar(255,255,0));
        cv::putText(img,"LEFT-CLICK on a point to remove it",           cv::Point(20,430),cv::FONT_HERSHEY_PLAIN,0.9,cv::Scalar(255,255,0));
        cv::putText(img,"Press 'R' to show the results",                cv::Point(20,445),cv::FONT_HERSHEY_PLAIN,0.9,cv::Scalar(255,255,0));
        cv::putText(img,"Press 'S' to save the board",                  cv::Point(20,460),cv::FONT_HERSHEY_PLAIN,0.9,cv::Scalar(255,255,0));
    }

    // thanks to http://bytefish.de/blog/extracting_contours_with_opencv/
    void cellDelimitation::crop_cells(cv_bridge::CvImageConstPtr& cv_cp_img, const std::vector<std::vector<cv::Point> > cntrs)
    {
        cv::Mat mask = cv::Mat::zeros(cv_cp_img->image.rows, cv_cp_img->image.cols, CV_8UC1);
        cv::drawContours(mask, cntrs, -1, cv::Scalar(255), CV_FILLED);        // CV_FILLED fills the connected components found with white (white RGB value = 255,255,255)

        cv::Mat im_crop(cv_cp_img->image.rows, cv_cp_img->image.cols, CV_8UC3); // let's create a new 8-bit 3-channel image with the same dimensions
        im_crop.setTo(cv::Scalar(0));                                           // we fill the new image with a color, in this case we set background to black.
        cv_cp_img->image.copyTo(im_crop, mask);                                 // copy just the elements from the original image indicated by the non-zero elements from mask to crop
        cv::normalize(mask.clone(), mask, 0.0, 255.0, CV_MINMAX, CV_8UC1);      // normalize so imwrite(...)/imshow(...) shows the mask correctly!

        // show the images
        cv::imshow("original", cv_cp_img->image);
        cv::imshow("mask", mask);
        cv::imshow("cropped", im_crop);
    }

    cellDelimitation::~cellDelimitation()
    {
        cv::destroyWindow(cellDelimitation::window_name);
    }

    // mouse event handler function 
    void cellDelimitation::onMouseClick( int event, int x, int y, int, void* param)
    {
        if( event != cv::EVENT_LBUTTONDOWN )
            return;

        cv::Point p = cv::Point(x,y);
        cellDelimitation * img_conv = (cellDelimitation*) param;

        // If the point is inside an already defined cell, the cell is removed
        if(img_conv->point_is_inside_cell(p)) {
            ROS_INFO_STREAM("Cell removed because point was in cell. Remaining " << img_conv->board.cells.size() << " cells");
            return;
        }

        // If the point corresponds to an already selected point, the point is removed
        if(img_conv->remove_point(p)) {
            ROS_INFO_STREAM("Point removed. Remaining " << img_conv->cell.contours.size() << " points");
        }
        // If the point is new, it is added to the current cell
        else {
            img_conv->cell.contours.push_back(p);
            ROS_INFO_STREAM("Point added: " << p.x << " , " << p.y << ". Total points in this cell: "  << img_conv->cell.contours.size() );
        }

        return;
    }


    void cellDelimitation::imageCallback(const sensor_msgs::ImageConstPtr& msg)
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
        show_tutorial(img_aux);

        // drawing all points of the current cell
        for (std::vector<cv::Point>::iterator image_transportdrawing = cell.contours.begin();image_transportdrawing != cell.contours.end();++image_transportdrawing) {
            cv::circle(img_aux,*image_transportdrawing,cellDelimitation::point_radius,cv::Scalar(0,0,255),-1);
        }

        // drawing all cells of the board game
        for (std::vector<Cell>::iterator it_cell = board.cells.begin(); it_cell != board.cells.end(); ++it_cell) {
            cv::fillConvexPoly(img_aux,it_cell->contours.data(),it_cell->contours.size(), cv::Scalar(0,0, 255));
        }
        cv::drawContours(img_aux, board.as_vector_of_vectors(),-1, cv::Scalar(123,125,0),2); // drawing the borders in a different color

        cv::setMouseCallback(cellDelimitation::window_name, onMouseClick, this);
        cv::imshow(cellDelimitation::window_name, img_aux);

        int c = cv::waitKey(3);
        if( (c & 255) == 27 ) // ESC key pressed
        {
            ROS_INFO_STREAM("Exiting with ESC key ..." << board.cells.size() << " cells selected");
            ros::shutdown();
        }
        else if ((char)c == ' ') {  // SPACE bar pressed
            if(!cell.contours.empty())
            {
                board.cells.push_back(cell);
                cell.contours.clear();
                ROS_INFO("Add points to the new cell"); 
            }
            else ROS_INFO_STREAM("The current cell is empty. Add points to it before you create a new one");
        }
        else if ((char)c=='r') { // 'R' key pressed
            std::vector<std::vector<cv::Point> > cntrs = board.as_vector_of_vectors();
            crop_cells(cv_ptr, cntrs);
        }
        else if ((char)c=='s') { // 'S' key pressed
            board.save();
        }

    }    

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "cell_delimitation");
    ttt::cellDelimitation cd;
    ros::spin();
    return 0;
}

*/
