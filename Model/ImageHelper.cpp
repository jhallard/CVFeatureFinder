#include "ImageHelper.h"

ImageHelper::ImageHelper(string listfile)
{
    this->currentFileIndex = 0;
    this->listOfFiles = listfile;
    this->setListofFiles(listfile);
}


vector<string> ImageHelper::getListOfFiles() const
{
    return this->filenames;
}
bool ImageHelper::setListofFiles(string listfile)
{
    filenames.clear();
    this->currentFileIndex = 0;
    this->listOfFiles = listfile;
    
    ifstream file(this->listOfFiles.c_str());

    if ( !file.is_open() )
        return false;

    while( !file.eof() )
    {
        string str; getline( file, str );
        if( str.empty() ) break;
        filenames.push_back(str);
    }
    file.close();

    this->img = cv::imread(this->filenames[this->currentFileIndex]);
    if(this->img.empty())
    {
        ROS_ERROR("Error : Invalid Path to Picture '%s'",this->filenames[this->currentFileIndex].c_str());
    }
    return true;
}

bool ImageHelper::setNextImage()
{
    this->currentFileIndex++;

    if(this->currentFileIndex == this->filenames.size())
        this->currentFileIndex = 0;

    this->img = cv::imread(this->filenames[this->currentFileIndex]);


    if(this->img.empty())
    {
        ROS_INFO_STREAM("ERROR Reading Images! in ImageHelper::setNextImage()\n");
        return false;
    }

    return true;
}



bool ImageHelper::setImage(Mat newimg)
{
    if(!newimg.empty())
    {
        this->img = newimg;
        return true;
    }

    return false;
}

Mat ImageHelper::getImage() const
{
    return this->img;
}



bool ImageHelper::setKeyPoints(vector<KeyPoint> kps)
{
    this->keyPoints.clear();
    this->keyPoints = kps;

    return true;
}

vector<KeyPoint> ImageHelper::getKeyPoints()
{
    //vector<KeyPoint> & hold = this->keyPoints;
    return keyPoints;
}


bool ImageHelper::setDescriptor(Mat newimg)
{
    if(!newimg.empty())
    {
        this->descriptors = newimg;
        return true;
    }
    else
    {
        ROS_ERROR("ImageHelper : Error Setting Descriptors, empty\n");
        return false;
    }
}

Mat ImageHelper::getDescriptor() const
{
    return this->descriptors;
}


