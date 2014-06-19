#include "FeatureFinder.h"

// default constructor
FeatureFinder::FeatureFinder() 
: LEFT_IMG(1), RIGHT_IMG(2), nopde(new ros::NodeHandle) WINDOW_NAME("CVFeatureFinderWindow")
{
    this->videoEnabled = false;
    leftFrame = new ImageHelper("../pics/defaultListfile.txt");
    rightFrame = new ImageHelper("../pics/defaultListfile.txt");

    // create the window for our program
    cv::namedWindow(WINDOW_NAME);
}

// listfile is the path to .txt file containing the paths to the images that the user wants the left/right frames to cycle through
FeatureFinder::FeatureFinder(string leftlistfile, string rightlistfile) :
FeatureFinder()
{
    if(!leftFrame.setListofFiles(leftlistfile))
        ROS_ERROR("Invalid list of filenames for left frame");
    else
        std:: cout << "\n Files for left Have been set successfully";

    if(!rightFrame.setListofFiles(rightlistfile))
        ROS_ERROR("Invalid list of filenames for right frame");
    else
        std:: cout << "\n Files for left Have been set successfully";
}


// destructor
FeatureFinder::~FeatureFinder()
{
    delete leftFrame;
    delete rightFrame;
    delete node;
}


bool enableVideoMode()
{
    // if we have already enabled video
    if(this->videoEnabled) 
        return false;

    // subscribe to the kinect rgb data publisher
    this->subscriber = node->subscribe("/camera/rgb/image_color", 4, &FeatureFinder::videoCallback, &this);

    // if this is 0 then the above failed
    if(!this->subscriber.getNumPublishers())
        return false;

    this->videoEnabled = true;

    return true; // everything worked!
}





// callback for our video feed
void FeatureFinder::videoCallback (const sensor_msgs::Image::ConstPtr& img)
{
        // if the left frame is in the video mode and it isn't in paused mode
    bool updateLeft =  this->leftFrame->getVideoMode() && !this->leftFrame->getPause();

    // if the right frame is in the video mode and it isn't in paused mode
    bool updateright = this->rightFrame->getVideoMode() && !this->rightFrame->getPause()
    
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


bool FeatureFinder::detectAndDescribeFeatures(int leftright);
{
  if(leftright == this->LEFT_IMG)
  {
    // Detect keypoints in both images.
    //SiftFeatureDetector detector(NP);
    detector->detect(leftFrame->getImage, leftFrame->getKeyPoints());

    //SiftDescriptorExtractor extractor;
    extractor->compute(leftFrame->getImage(), leftFrame->getKeyPoints(), leftFrame->getDescriptor());

  else if(leftright == this->RIGHT_IMG)
  {
    // Detect keypoints in both images.
    //SiftFeatureDetector detector(NP);
    detector->detect(rightFrame->getImage, rightFrame->getKeyPoints());

    //SiftDescriptorExtractor extractor;
    extractor->compute(rightFrame->getImage(), rightFrame->getKeyPoints(), leftFrame->getDescriptor());
  }

  return true;
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

