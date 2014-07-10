#include <QtGui/QApplication>
#include <QApplication>
#include <QWidget>
#include <boost/thread.hpp>

#include "../Model/FeatureFinder.h"
#include "../View/CVFF_MainWindow.h"
#include <iostream>

class CVFF_MainWindow;

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "FeatureFinder");

    FeatureFinder * finder = new FeatureFinder("CVWindowName");

    if(argc == 2)
    {
        finder->enableVideoMode();
        if(argv[1][0] == 'l')
            finder->toggleVideoMode(true, finder->LEFT_IMG);
        else if(argv[1][0] == 'r')
            finder->toggleVideoMode(true, finder->RIGHT_IMG);
        else
        {
            finder->toggleVideoMode(true,finder->LEFT_IMG );
            finder->toggleVideoMode(true,finder->RIGHT_IMG);
        }
    }

    
    if(finder->videoEnabled)
        ros::spin();
    else
    {
        while(ros::ok())
        {
            char c = cv::waitKey(2);//ros::spinOnce();
            if(c == 'l')
                finder->changeImage(finder->LEFT_IMG);
            else if(c == 'r')
                finder->changeImage(finder->RIGHT_IMG);
        }
    }
    cv::destroyWindow(finder->WINDOW_NAME);

    return 0;
}

// int main(int argc, char *argv[])
// {
//     QApplication app(argc, argv);
 
//     CVFF_MainWindow mainWindow(0);
//     mainWindow.showMaximized();
//     return app.exec();
// }



// int main(int argc, char ** argv)
// {
//     ros::init(argc, argv, "FeatureFinder");

//     FeatureFinder * finder = new FeatureFinder("CVWindowName");
//     finder->enableVideoMode();

//     if(argc == 2)
//     {
//         if(argv[1][0] == 'l')
//             finder->toggleVideoMode(true, finder->LEFT_IMG);
//         else if(argv[1][0] == 'r')
//             finder->toggleVideoMode(true, finder->RIGHT_IMG);
//         else
//         {
//             finder->toggleVideoMode(true,finder->LEFT_IMG );
//             finder->toggleVideoMode(true,finder->RIGHT_IMG);
//         }
//     }

    
//     //while(ros::ok())
//     //    ros::spinOnce();
//     ros::spin();
//     cv::destroyWindow(finder->WINDOW_NAME);

//     return 0;
// }