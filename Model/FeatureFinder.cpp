#include "FeatureFinder.h"
#include <stdio.h>


// default constructor
FeatureFinder::FeatureFinder(const string wind) 
: LEFT_IMG(1), RIGHT_IMG(2), node(new ros::NodeHandle), matcher(new FlannBasedMatcher()), WINDOW_NAME(wind)
{
    this->init();
}

// listfile is the path to .txt file containing the paths to the images that the user wants the left/right frames to cycle through
FeatureFinder::FeatureFinder(const string wind, string leftlistfile, string rightlistfile)
: LEFT_IMG(1), RIGHT_IMG(2), node(new ros::NodeHandle), matcher(new FlannBasedMatcher()), WINDOW_NAME(wind)
{
    this->init();

    if(!leftFrame->setListofFiles(leftlistfile))
        ROS_ERROR("Invalid list of filenames for left frame");
    else
        std:: cout << "\n Files for left Have been set successfully";

    if(!rightFrame->setListofFiles(rightlistfile))
        ROS_ERROR("Invalid list of filenames for right frame");
    else
        std:: cout << "\n Files for left Have been set successfully";

}

 void FeatureFinder::init()
 {
    this->videoEnabled = false;
    this->videoLeft = false;
    this->videoRight = false;
    this->pauseLeft = false;
    this->pauseRight = false;

    // set the frames to contain an index of default pictures enumerated in the deafultListFile.txt file.
    // if the user passes in string arguments for these names then they will simply be overwritten and a new index will be made
    leftFrame = new ImageHelper("../pics/defaultListfile.txt");
    rightFrame = new ImageHelper("../pics/defaultListfile1.txt");

    //initialize our feature detect and descriptor extractor
    this->detector = nullptr;
    this->extractor = nullptr;

    cv::initModule_nonfree();

    ROS_INFO_STREAM("Initializing Feature Detector and Extractor\n");
    this->setFeatureDetector("SURF");
    this->setFeatureExtractor("SURF");

    // compute the features and descriptors of our default images
    detectAndDescribeFeatures(this->LEFT_IMG);
    detectAndDescribeFeatures(this->RIGHT_IMG);

    // create the window for our program
    // ROS_INFO_STREAM("Opening window\n");
     cv::namedWindow(WINDOW_NAME);

    // show the matches between the default images
    this->computeMatches();
    this->showCurrentFrames();

    cv::waitKey(2);
 }


// destructor
FeatureFinder::~FeatureFinder()
{
    delete leftFrame;
    delete rightFrame;
    delete node;
}


bool FeatureFinder::setFeatureDetector( string type )
{
    if(type == "SURF" || type == "surf" || type == "Surf")
    {
        this->detector = new cv::SurfFeatureDetector(800);
    }
    else if(type == "SIFT" || type == "Sift" || type == "sift")
    {
        this->detector = new cv::SiftFeatureDetector(80);
    }
    else if(type == "ORB" || type == "Orb" || type == "orb")
    {
        this->detector = new cv::OrbFeatureDetector(10);
    }
    else if(type == "MSER" || type == "Mser" || type == "mser")
    {
        this->detector = new cv::MserFeatureDetector();
    }
    else
    {
        ROS_ERROR("Invalid type passed to setFeatureDetector");
        return false;
    }

    return true;
}

bool FeatureFinder::setFeatureExtractor( string type )
{
    if(type == "SURF" || type == "surf" || type == "Surf")
    {
        this->extractor = cv::DescriptorExtractor::create("SURF");
        return true;
    }
    else if(type == "SIFT" || type == "Sift" || type == "sift")
    {
       this->extractor = cv::DescriptorExtractor::create("SIFT");
       return true;
    }
    else
    {
        ROS_ERROR("Invalid type passed to setFeatureExtractor");
        return false;
    }
}


bool FeatureFinder::enableVideoMode()
{
    // if we have already enabled video
    if(this->videoEnabled) 
        return false;

    // subscribe to the kinect rgb data publisher
    this->subscriber = node->subscribe("/camera/rgb/image_color", 4, &FeatureFinder::videoCallback, this);

    ROS_INFO_STREAM("Connecting To Camera.. may take a moment..");
    while(!this->subscriber.getNumPublishers())
    {
        // wait for the feed to connect
    }

    ROS_INFO_STREAM("Connection Successful, Video Data Feed Running");

    this->videoEnabled = true;
    return true; // everything worked!
}


// callback for our video feed
void FeatureFinder::videoCallback(const sensor_msgs::Image::ConstPtr& img)
{
        // if the left frame is in the video mode and it isn't in paused mode
    bool updateLeft =  this->getVideoMode(LEFT_IMG) && !this->getPause(LEFT_IMG);

    // if the right frame is in the video mode and it isn't in paused mode
    bool updateRight = this->getVideoMode(RIGHT_IMG) && !this->getPause(RIGHT_IMG);
    
    cv_bridge::CvImagePtr cv_ptr;
    try 
    { 
        cv_ptr = cv_bridge::toCvCopy(*img, sensor_msgs::image_encodings::BGR8); //enc::RGB8 also used
    }
    catch (cv_bridge::Exception& e) 
    {
        ROS_ERROR("cv_bridge exception: Video Callback : %s", e.what());
        return;
    }

    if(updateLeft)
    {
        this->leftFrame->setImage(cv_ptr->image);
        this->detectAndDescribeFeatures(this->LEFT_IMG);
        if(leftFrame->getImage().empty())
            ROS_INFO_STREAM("Left Image Empty");
    }
    if(updateRight)
    {
        this->rightFrame->setImage(cv_ptr->image);
        this->detectAndDescribeFeatures(this->RIGHT_IMG);
        if(rightFrame->getImage().empty())
            ROS_INFO_STREAM("Right Image Empty");
    }
    // if either of them is in video mode
    if(updateLeft || updateRight)
    {
        // try to compute the matches, throw an error if it doesn't work
        if(!this->computeMatches())
        {
            ROS_ERROR("Problem computing matches in Video Callback");
            return;
        }
    }
    // show the current frames
    this->showCurrentFrames();
}


bool FeatureFinder::detectAndDescribeFeatures(int leftright)
{

    vector<KeyPoint> holdl;// = leftFrame->getKeyPoints();
    Mat imgl;// = leftFrame->getDescriptor();

    vector<KeyPoint> holdr;// = rightFrame->getKeyPoints();
    Mat imgr;// = rightFrame->getDescriptor();

  if(leftright == this->LEFT_IMG)
  {
    detector->detect(leftFrame->getImage(), holdl);

    if(!holdl.size())
        ROS_ERROR("left keypoints empty");

    leftFrame->setKeyPoints(holdl);

    extractor->compute(leftFrame->getImage(), holdl, imgl);

    leftFrame->setDescriptor(imgl);
  }

  else if(leftright == this->RIGHT_IMG)
  {
    detector->detect(rightFrame->getImage(), holdr);

    if(!holdr.size())
        ROS_ERROR("right keypoints empty");

    rightFrame->setKeyPoints(holdr);

    extractor->compute(rightFrame->getImage(), holdr, imgr);

    rightFrame->setDescriptor(imgr);
  }

  return true;
}


bool FeatureFinder::computeMatches()
{
    std::vector< DMatch > good_matches;

    if(!(leftFrame->getDescriptor().empty() || rightFrame->getDescriptor().empty()))
    {
        matcher->knnMatch(leftFrame->getDescriptor(), rightFrame->getDescriptor(), this->vecmatches, 2);
        this->matches.clear();
        for (int i = 0; i < vecmatches.size(); ++i)
        {
            // PLAY WITH THE RATIO VALUE, IT MAKES A HUGGGEEE FUCKING DIFFERENCE. 
            // 0.8 GIVES MORE FALSE POSITIVES BUT ALSO GIVES HIGHER READING ON TRUE-POSITIVES.
            // 0.75 SEEMS TO BE THE BEST, IT COMPLETELY REJECTS BAD MATCHES BUT HIGHLY MATCHES GOOD IMAGES.
            const float ratio = 0.75; // As in Lowe's paper; can be tuned
            if (vecmatches[i][0].distance < ratio * vecmatches[i][1].distance)
            {
                this->matches.push_back(vecmatches[i][0]);
            }
        }

        // double max_dist = 0; double min_dist = 100;

        // //-- Quick calculation of max and min distances between keypoints
        // for( int i = 0; i < leftFrame->getDescriptor().rows; i++ )
        // { 
        //     double dist = matches[i].distance;
        //     if( dist < min_dist ) min_dist = dist;
        //     if( dist > max_dist ) max_dist = dist;
        // }

        // printf("-- Max dist : %f \n", max_dist );
        // printf("-- Min dist : %f \n", min_dist );

        // for( int i = 0; i < leftFrame->getDescriptor().rows; i++ )
        // { 
        //     if( this->matches[i].distance <= max(3*min_dist, 0.03) )
        //     { 
        //         good_matches.push_back( matches[i]); 
        //     }
        // }
    }
    else
    {
        this->matches.clear();
        ROS_ERROR("DrawMatches() : Descriptors empty");
    }

    if(this->matches.size() == 0)
        ROS_ERROR("could not compute any matches");

    if(leftFrame->getKeyPoints().size() && rightFrame->getKeyPoints().size())
    {
        drawMatches(leftFrame->getImage(), leftFrame->getKeyPoints(), rightFrame->getImage(), rightFrame->getKeyPoints(), this->matches, this->img_matches);
    }
    else
    {
        ROS_ERROR("DrawMatches : KeyPoints Empty");
        return false;
    }

    return true;

}

// show the currently stored images in the left/right ImageHelper objects to the window
bool FeatureFinder::showCurrentFrames()
{
    if(!this->img_matches.empty())
    {
        //ROS_INFO_STREAM("Drawing Matches");
        cv::imshow(this->WINDOW_NAME, this->img_matches);
        cv::waitKey(2);
        return true;
    }
    else
    {
        ROS_INFO_STREAM("Matches image empty!\n");
        return false;
    }
}



bool FeatureFinder::changeImage(int leftright)
{
    if(leftright == this->LEFT_IMG)
    {
        leftFrame->setNextImage();  // shuffle the image in the left frame
        detectAndDescribeFeatures(this->LEFT_IMG); 
    }
    else if(leftright == this->RIGHT_IMG)
    {
        rightFrame->setNextImage(); // shuffle the image in the right frame
        detectAndDescribeFeatures(this->RIGHT_IMG);
    }
    else
        return false; // invalid parameter

    this->computeMatches();
    this->showCurrentFrames();
}



// =================== //*
// === GET AND SET === //
// =================== // 
ImageHelper * FeatureFinder::getLeft() const
{
    return this->leftFrame;
}

ImageHelper * FeatureFinder::getRight() const
{
    return this->rightFrame;
}

bool FeatureFinder::toggleVideoMode(bool onoff, int leftright)
{
    if(leftright == this->LEFT_IMG)
        videoLeft = onoff;
    else if(leftright == this->RIGHT_IMG)
        videoRight = onoff;
    else
        return false;

    return true;
}
bool FeatureFinder::getVideoMode(int leftright) const
{
    return ((leftright == this->LEFT_IMG)? videoLeft : videoRight);
}

bool FeatureFinder::togglePause(int leftright)
{
    if(leftright == this->LEFT_IMG)
        pauseLeft = !pauseLeft;
    else if(leftright == this->RIGHT_IMG)
        pauseRight = !pauseRight;
    else
        return false;

    return true;
}
bool FeatureFinder::getPause(int leftright) const
{
    return ((leftright == this->LEFT_IMG)? pauseLeft : pauseRight);
}