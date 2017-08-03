/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<iomanip>
#include<chrono>

#include<opencv2/core/core.hpp>

#include<System.h>


#include "camera.h"
#include "utility.h"


using namespace std;
using namespace mynteye;




int main(int argc, char **argv)
{
    std::uint32_t timestamp;
    float time;
    vector<IMUData> imudatas;
    // Open camera
    std::cout << "Open Camera: "<< std::endl;
    stringstream device;
    device << argv[3];
    mynteye::Camera cam;
    mynteye::InitParameters params(device.str());
    std::cout << "Open Camera:1 " << std::endl;

    cam.Open(params);
    std::cout << "Open Camera: "  << std::endl;
    if (!cam.IsOpened()) {
        std::cerr << "Error: Open camera failed" << std::endl;
        return 1;
    }
    std::cout << "\033[1;32mPress ESC/Q on Windows to terminate\033[0m\n";

    while(cam.Grab() != ErrorCode::SUCCESS) {
        std::cout << "Warning: Grab failed <" << ">" << std::endl;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::STEREO,true);


    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;


    cv::Mat img_left;
    cv::Mat img_right;
    for(;;)
    {

        cam.Grab();
        cam.RetrieveImage(img_left, View::VIEW_LEFT_UNRECTIFIED);
        cam.RetrieveImage(img_right, View::VIEW_RIGHT_UNRECTIFIED);
        cam.RetrieveIMUData(imudatas,timestamp);
        time = (float)(timestamp/10000.0);

        SLAM.TrackStereo(img_left, img_right,time);

    }
    SLAM.Shutdown();
    SLAM.SaveTrajectoryKITTI("CameraTrajectory.txt");

    cam.Close();
    cv::destroyAllWindows();

    return 0;

}

