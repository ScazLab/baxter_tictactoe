#include "robot_perception/cartesian_estimator.h"


class SegmentedObTTT : public SegmentedObj
{
public:

    /* CONSTRUCTOR */
    SegmentedObTTT(std::vector<double> _size, hsvColorRange _col);
    SegmentedObTTT(std::string _name, std::vector<double> _size, int _area_thres, hsvColorRange _col);

    /* DESTRUCTOR */
    ~SegmentedObTTT();

    /**
     * Detects the object in the image
     *
     * @param _in        Input image to detect objects from
     * @param _out       Output image to show the result of the segmentation
     *
     * @return true/false if success/failure
     */
    bool detectObject(const cv::Mat& _in, cv::Mat& _out);

    /**
     * Detects the object in the image
     *
     * @param _in        Input image to detect objects from
     * @param _out       Output image to show the result of the segmentation
     * @param _out_thres Optional output image to show the thresholded image
     *
     * @return true/false if success/failure
     */
    bool detectObject(const cv::Mat& _in, cv::Mat& _out, cv::Mat& _out_thres);

    /**
     * Converts the segmented object to a string.
     * @return the segmented object as a string
     */
    operator std::string();
}
