#include "../Model/FeatureFinder.h"
#include <iostream>

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "FeatureFinder");

    FeatureFinder * finder = new FeatureFinder("CVWindowName");
    finder->enableVideoMode();

    if(argc == 2)
    {
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

    
    //while(ros::ok())
    //    ros::spinOnce();
    ros::spin();
    cv::destroyWindow(finder->WINDOW_NAME);

    return 0;
}






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