#include "FeatureFinder.h"

// default constructor
FeatureFinder::FeatureFinder() 
: LEFT_IMG(1), RIGHT_IMG(2), node(new ros::NodeHandle), matcher(new FlannBasedMatcher()), WINDOW_NAME("CVFeatureFinderWindow")
{
    this->init();
}

// listfile is the path to .txt file containing the paths to the images that the user wants the left/right frames to cycle through
FeatureFinder::FeatureFinder(string leftlistfile, string rightlistfile)
: LEFT_IMG(1), RIGHT_IMG(2), node(new ros::NodeHandle), matcher(new FlannBasedMatcher()), WINDOW_NAME("CVFeatureFinderWindow")
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
    leftFrame = new ImageHelper("../pics/defaultListfile.txt");
    rightFrame = new ImageHelper("../pics/defaultListfile.txt");

    leftFrame->toggleVideoMode(true);

    this->detector = nullptr;
    this->extractor = nullptr;

    cv::initModule_nonfree();

    ROS_INFO_STREAM("Initializing Feature Detector and Extractor\n");
    this->setFeatureDetector("SURF");
    this->setFeatureExtractor("SIFT");


    // create the window for our program
    ROS_INFO_STREAM("Opening window\n");
    cv::namedWindow(WINDOW_NAME);

    // compute the features and descriptors of our default images
    detectAndDescribeFeatures(this->LEFT_IMG);
    detectAndDescribeFeatures(this->RIGHT_IMG);

    ROS_INFO_STREAM("Features Found\n");

    // show the matches between the default images
    showCurrentFrames();
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
        this->detector = new cv::SurfFeatureDetector(3000, 6, 2, true, true);
    }
    else if(type == "SIFT" || type == "Sift" || type == "sift")
    {
        this->detector = new cv::SiftFeatureDetector(50);
    }
    else if(type == "ORB" || type == "Orb" || type == "orb")
    {
        this->detector = new cv::OrbFeatureDetector(50);
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

    ROS_INFO_STREAM("Connection Successful");

    this->videoEnabled = true;

    return true; // everything worked!
}


// callback for our video feed
void FeatureFinder::videoCallback (const sensor_msgs::Image::ConstPtr& img)
{
        // if the left frame is in the video mode and it isn't in paused mode
    bool updateLeft =  this->leftFrame->getVideoMode() && !this->leftFrame->getPause();

    // if the right frame is in the video mode and it isn't in paused mode
    bool updateRight = this->rightFrame->getVideoMode() && !this->rightFrame->getPause();
    
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
    }
    if(updateRight)
    {
        this->rightFrame->setImage(cv_ptr->image);
        this->detectAndDescribeFeatures(this->RIGHT_IMG);
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

        // show the current frames
        this->showCurrentFrames();
    }

}


bool FeatureFinder::detectAndDescribeFeatures(int leftright)
{

    vector<KeyPoint> holdl = leftFrame->getKeyPoints();

    Mat imgl = leftFrame->getDescriptor();

    vector<KeyPoint> holdr = rightFrame->getKeyPoints();
    Mat imgr = rightFrame->getDescriptor();

  if(leftright == this->LEFT_IMG)
  {
    detector->detect(leftFrame->getImage(), holdl);

    leftFrame->setKeyPoints(holdl);

    //SiftDescriptorExtractor extractor;
    extractor->compute(leftFrame->getImage(), holdl, imgl);

    leftFrame->setDescriptor(imgl);
  }

  else if(leftright == this->RIGHT_IMG)
  {
    // Detect keypoints in both images.
    //SiftFeatureDetector detector(NP);
    detector->detect(rightFrame->getImage(), holdr);

    rightFrame->setKeyPoints(holdr);

    //SiftDescriptorExtractor extractor;
    extractor->compute(rightFrame->getImage(), holdr, imgr);

    rightFrame->setDescriptor(imgr);
  }

  return true;
}


bool FeatureFinder::computeMatches()
{
    matcher->match(leftFrame->getDescriptor(), rightFrame->getDescriptor(), this->matches);

    if(!this->matches.size())
        ROS_ERROR("could not compute any matches");

    drawMatches(leftFrame->getImage(), leftFrame->getKeyPoints(), leftFrame->getImage(), leftFrame->getKeyPoints(), this->matches, this->img_matches);

    return true;

}

// show the currently stored images in the left/right ImageHelper objects to the window
bool FeatureFinder::showCurrentFrames()
{
    if(!this->img_matches.empty())
    {
        cv::imshow(this->WINDOW_NAME, this->img_matches);
        return true;
    }
    else
        return false;
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







// =================== //
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

