#ifndef CONTOUR_BLOB_DETECTOR_H
#define CONTOUR_BLOB_DETECTOR_H

#include <opencv2/features2d/features2d.hpp>

class contour_blob_detector : public cv::SimpleBlobDetector {

public:

    contour_blob_detector(const cv::SimpleBlobDetector::Params& parameters = cv::SimpleBlobDetector::Params());
    const std::vector<std::vector<cv::Point> > getContours();

protected:

    virtual void detectImpl(const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints, const cv::Mat& mask = cv::Mat()) const;
    virtual void findBlobs(const cv::Mat& image, const cv::Mat& binaryImage,
                           std::vector<Center>& centers, std::vector<std::vector<cv::Point> >& contours) const;

    mutable std::vector<std::vector<cv::Point> > _contours;

};

#endif // CONTOUR_BLOB_DETECTOR_H
