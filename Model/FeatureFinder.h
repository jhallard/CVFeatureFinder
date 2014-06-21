#ifndef FEATURE_FINDER_H_
#define FEATURE_FINDER_H_


/**
*   @File           - FeatureFinder.h
*   @Author         - John H Allard
*   @Date           - 6/19/2014
*   @Description    - This file holds the declaration of the FeatureFinder class. This class serves as a simplification of some key features
*                   of the OpenCV and ROS libraries. These features include, importing 2 images, extracting keypoints from the images, detectings
*                   features present in the images, matching the features together between images, and showing these matches to the user. 
*                   Also, everything that was just described can also be applied to a video feed from a webcam or kinect camera. This means the user
*                   can perform real-time matching between a video feed and an image, or even two video feeds!
*                   
*                   The main idea behind this program is to show a window that is divided into two parts horizontally. Each of these two parts,
*                   left and right halves, contains an image or a frame from a video feed. We then compute the keypoints of the individual images
*                   and extract the features and feature descriptors of the images and/or frame from a video feed. We then match the features
*                   from both individual images together, and display these images side by side with a line connecting each keypoint. 
*
*                   The key features are a few things. First, the Feature Dectection and Feature Descriptor Extractor Algorithms are all customizable
*                   to the end user. This means the user can build a GUI with push buttons and sliders and calibrate the algorithms in real-time
*                   to see which combination of parameters works the best for detection of specific objects. 
*
*   @PRE-CONDITIONS - Before using this class for video feeds (aka calling FeatureFinder::enableVideoFeed(), you must call roscore to start 
*                    the ROS drivers. This is because we use the ROS publishing features to transfer video feed data around the class. 
*
*                   - You must also call 'roslaunch openni_launch openni.launch' if you wish to use a kinect camera as the video feed.
*
*                   - You must have the desktop build of ros-hydro installed on your computer, other versions might work but have not been tested.
*
*
**/

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

    /**___Image, Window, and Video-Feed Fields__**/
    ImageHelper * leftFrame;        // a helper class that contains an image, it's keypoints, and it's descriptors. We have one for the left frame
    ImageHelper * rightFrame;       // and one for the right frame. These items are wrapped in a class for convenience and to simplify this class.
    bool pauseLeft, pauseRight, videoLeft, videoRight; // state flags that determine if the user wants to forward video feed to the 
                                                       // left and/or right frames, and if they want to currently pause the video feed
    
    /**___OpenCV Related Fields__**/    
    cv::Ptr<cv::FeatureDetector> detector;      // A pointer to a FeatureDetector object which contains a Feature detection algorithm.     
    cv::Ptr<cv::DescriptorExtractor> extractor; // A pointer to an object that extracts descriptors of keypoints.

    cv::FlannBasedMatcher * matcher;            // our matching object (contains the flann matching algorithm)
    vector<DMatch> matches;                     // vector specifying the matching pairs of keypoints
    Mat img_matches;                            // the final image that shows all the matches, we write this to the screen


    /**___ROS related variables__**/
    ros::NodeHandle * node;                     // our ROS node to subscribe to a video data publisher and to publish ressages ourselves
    ros::Subscriber subscriber;                 // subscribes to the kinect data feed


    //----===========================----//
    // ---  PRIVATE HELPER FUNCTIONS --- //
    //----===========================----//

    // private helper function called by the constructors
    void init();

    // changes the type of feature detector we are using (SIFT, SURF, ORB, MSER, etc.)
    bool setFeatureDetector( string type );

    // changes the type of extractor we are using (SIFT or SURF currently)
    bool setFeatureExtractor( string type );

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
    FeatureFinder(const string);                                // input the name for the cv::NamedWindow object to be referenced under
    FeatureFinder(const string, string lfile, string rfile);    // 

    // Destructor
    ~FeatureFinder();

    ImageHelper * getLeft() const;  // get a pointer to the left image helper
    ImageHelper * getRight() const; // get a poiter to the right image helper

    // callback for our video feed
    void videoCallback (const sensor_msgs::Image::ConstPtr& img);

    bool enableVideoMode(); // this only needs to be called once to connect to the webcam/kinect camera. Once that is done then the user can toggle the videomode of 
                            // the left of right frame by calling the ImageHelper::togglevideoMode() function.

    bool toggleVideoMode(bool onoff, int leftright); // toggle on and off video mode
    bool getVideoMode(int leftright) const;

    bool togglePause(int leftright);               // toggle the pause on the video feed
    bool getPause(int leftright) const;


    //----===========================----//
    // ---      PUBLIC DEFINES       --- //
    //----===========================----//
    const int LEFT_IMG;         // an int that refers to the left image of the screen
    const int RIGHT_IMG;        // && same for the right image
    const string WINDOW_NAME;   // name of the window that this program makes
};


#endif