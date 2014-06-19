#ifndef IMAGE_HELPER_H_
#define IMAGE_HELPER_H_

// cv includes
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"

using namespace cv;
using namesapce std;

class ImageHelper
{
private:
    // query image related items
    Mat img;
    vector<KeyPoint> keyPoints;
    Mat descriptors;

    int currentFileIndex;
    vector<string> filenames; // names of the files that we cycle through for the base images
    string listOfFiles;       // a .txt file with a list of the paths to the images we want to use

    bool videoMode; // true if we want to use a video feed as the query images
    bool pause;     // True is the user wants to pause the incoming video feed

public:
    explicit ImageHelper(string listfile, bool videomode = false);

    vector<string> getListOfFiles() const;
    bool setListofFiles(string listfile);

    bool setNextImage();

    bool setImage(Mat newimg);
    Mat & getImage() const;

    bool setKeyPoints(vector<KeyPoint> kps);
    vector<KeyPoint> & getKeyPoints() const;

    bool setDescriptor(Mat img);
    Mat & getDescriptor() const;

    bool toggleVideoMode(bool onoff); // toggle on and off video mode
    bool getVideoMode() const;

    bool togglePause();               // toggle the pause on the video feed
    bool getPause() const;
};

#endif