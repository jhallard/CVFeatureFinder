#ifndef FEATURE_FINDER_H_
#define FEATURE_FINDER_H_

// Self-Defined includes
#include "ImageHelper.h"

// std c++ includes
#include <vector>
#include <string>
#include <stdio.h>
#include <iostream>
#include <fstream>

// ROS includes
#include "ros/ros.h"
#include <sensor_msgs/image_encodings.h>
#include "sensor_msgs/Image.h"

// OpenCV includes
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"

using namespace cv;
using namespace std;

class FeatureFinder
{

private:
    //========================//
    // ---  MEMBER FIELDS --- //
    //========================//

    ImageHelper * leftFrame;   // a helper class that contains an image, it's keypoints, and it's descriptors. We have one for the left frame
    ImageHelper * rightFrame;  // and one for the right frame

    // Feature detector and extractor
    cv::Ptr<cv::FeatureDetector> detector;
    cv::Ptr<cv::DescriptorExtractor> extractor;

    // feature matcher between the two frames
    cv::FlannBasedMatcher * matcher;
    vector<DMatch> matches;

    Mat img_matches;           // the final image that shows all the matches, we write this to the screen

    // ROS related variable
    ros::NodeHandle * node;          // our ROS node to recieve published video data
    ros::Subscriber subscriber;    // subscribes to the kinect data feed


    //----===========================----//
    // ---  PRIVATE HELPER FUNCTIONS --- //
    //----===========================----//

    // private helper function called by the constructors
    void init();

    // changes the type of feature detector we are using (SIFT, SURF, ORB, MSER, etc.)
    bool setFeatureDetector( string type );

    // changes the type of extractor we are using (SIFT or SURF currently)
    bool setFeatureExtractor( string type);

    // detects and computes descriptors for either the left or right image/video feed
    bool detectAndDescribeFeatures(int leftright); 

    // computes the matches between the current left and right images, stores the results in the matches vector
    bool computeMatches();

    // show the currently stored images in the left/right ImageHelper objects to the window
    bool showCurrentFrames();

    // changes the currently shown image for either the left or right frame to the next one in the lest of filenames stored in the object
    // returns false if the frame is currently in video mode 
    bool changeImage(int leftright);




public:

    //----===================----//
    // ---  PUBLIC FUNCTIONS --- //
    //----===================----//

    // Constructors
    FeatureFinder(const string);
    explicit FeatureFinder(const string, string lfile, string rfile);

    // Destructor
    ~FeatureFinder();

    ImageHelper * getLeft() const;  // get a pointer to the left image helper
    ImageHelper * getRight() const; // get a poiter to the right image helper

    // callback for our video feed
    void videoCallback (const sensor_msgs::Image::ConstPtr& img);

    bool enableVideoMode(); // this only needs to be called once to connect to the webcam/kinect camera. Once that is done then the user can toggle the videomode of 
                            // the left of right frame by calling the ImageHelper::togglevideoMode() function.




    //----===========================----//
    // ---      PUBLIC DEFINES       --- //
    //----===========================----//
    const int LEFT_IMG;
    const int RIGHT_IMG;
    bool videoEnabled;

    const string WINDOW_NAME;   // name of the window that this program makes



};


#endif