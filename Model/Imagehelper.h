#ifndef IMAGE_HELPER_H_
#define IMAGE_HELPER_H_

using namespace cv;
using namesapce std;

class ImageHelper
{
private:
    // query image related items
    Mat img;
    vector<KeyPoint> keyPoints;
    Mat descriptors;

    vector<string> filenames; // names of the files that we cycle through for the base images
    string listoffiles;       // a .txt file with a list of the paths to the images we want to use

    bool videoMode; // true if we want to use a video feed as the query images
    bool pause;     // True is the user wants to pause the incoming video feed

public:
    explicit ImageHelper(string listfile, bool videomode = false);

    bool setListofFiles(string listfile);

    bool setImage(Mat newimg);
    Mat  getImage() const;

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