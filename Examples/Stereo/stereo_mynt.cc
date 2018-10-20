/**
* Th file is part of ORB-SLAM2.
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

#include <iostream>
#include <fstream>
#include <algorithm>
#include <cassert>
#include <opencv2/opencv.hpp>
#include "../../include/System.h"

#include "mynteye/logger.h"
#include "mynteye/device.h"
#include "mynteye/utils.h"
#include "mynteye/times.h"

MYNTEYE_USE_NAMESPACE

bool flag = true;
void exit_while(int sig)
{
  flag = false;
}

class ImageGrabber
{
    public: 
    	ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){}
        void GrabStereo(const cv::Mat& left_img, const cv::Mat& right_img, double timeStamp);
    ORB_SLAM2::System* mpSLAM;
    bool do_rectify;
    cv::Mat M1l, M2l, M1r, M2r;
};

void ImageGrabber::GrabStereo(const cv::Mat& left_img, const cv::Mat& right_img, double timeStamp)
{
    if (left_img.empty() || right_img.empty())
    {
	assert("left_img == empty || right_img == empty");
    }

    if (do_rectify)
    {
    	cv::Mat _left_img, _right_img;
	cv::remap(left_img, _left_img, M1l, M2l, cv::INTER_LINEAR);
	cv::remap(right_img, _right_img, M1r, M2r, cv::INTER_LINEAR);
	mpSLAM->TrackStereo(_left_img, _right_img, timeStamp);
    }
    else
    {
      mpSLAM->TrackStereo(left_img, right_img, timeStamp);
    }
}

int main(int argc, char** argv)
{

    glog_init _(argc, argv);


    if (argc != 4 )
    {
    	std::cout << std::endl << "Usage: ./stereo_mynt path_to_vocabulary path_to_setting do_rectify " << std::endl;
	return 1;
    }

    std::cout << "--args: " << std::endl
    	      << "	path_to_vocabulary: " << argv[1] << std::endl
	      << "	path_to_setting: " << argv[2] << std::endl
	      << "	do_rectify(ture | false): " << argv[3] << std::endl;

    ORB_SLAM2::System SLAM(argv[1], argv[2], ORB_SLAM2::System::STEREO, true);
    ImageGrabber igb(&SLAM);

    std::stringstream ss(argv[3]);
    	ss >> boolalpha >> igb.do_rectify;

    if (igb.do_rectify)
    {
    	cv::FileStorage fsSetting(argv[2], cv::FileStorage::READ);
	if (!fsSetting.isOpened())
	{
	    std::cerr << "error: wrong path to setting"	<< std::endl;
	    return -1;
	}

	cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
	fsSetting["LEFT.K"] >> K_l;
	fsSetting["RIGHT.K"] >>  K_r;

	fsSetting["LEFT.P"] >> P_l;
	fsSetting["RIGHT.P"] >> P_r;

	fsSetting["LEFT.R"] >> R_l;
	fsSetting["RIGHT.R"] >> R_r;

	fsSetting["LEFT.D"] >> D_l;
	fsSetting["RIGHT.D"] >> D_r;

	int rows_l = fsSetting["LEFT.height"];
	int cols_l = fsSetting["LEFT.width"];
	int rows_r = fsSetting["RIGHT.height"];
	int cols_r = fsSetting["RIGHT.width"];

        if(K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() || D_l.empty() || D_r.empty() ||
                rows_l==0 || rows_r==0 || cols_l==0 || cols_r==0)	
	{
		std::cerr << "error: calibration parameters to rectify stereo are missing!" << std::endl;
		return -1;
	}

	cv::initUndistortRectifyMap(K_l,D_l,R_l,P_l.rowRange(0,3).colRange(0,3),cv::Size(cols_l,rows_l),CV_32F,igb.M1l,igb.M2l);
	cv::initUndistortRectifyMap(K_r,D_r,R_r,P_r.rowRange(0,3).colRange(0,3),cv::Size(cols_r,rows_r),CV_32F,igb.M1r,igb.M2r);
	fsSetting.release();
    }

    auto &&device = device::select();
    if (!device)
    	return 1;
    /*
  {  // auto-exposure
    device->SetOptionValue(Option::EXPOSURE_MODE, 0);
    device->SetOptionValue(Option::MAX_GAIN, 40);  // [0.48]
    device->SetOptionValue(Option::MAX_EXPOSURE_TIME, 120);  // [0,240]
    device->SetOptionValue(Option::DESIRED_BRIGHTNESS, 200);  // [0,255]
  }
  {  // manual-exposure
    device->SetOptionValue(Option::EXPOSURE_MODE, 1);
    device->SetOptionValue(Option::GAIN, 20);  // [0.48]
    device->SetOptionValue(Option::BRIGHTNESS, 20);  // [0,240]
    device->SetOptionValue(Option::CONTRAST, 20);  // [0,255]
  }
  device->SetOptionValue(Option::IR_CONTROL, 80);
  device->SetOptionValue(Option::FRAME_RATE, 25);
  device->SetOptionValue(Option::IMU_FREQUENCY, 500);
  */
    device->LogOptionInfos();

    // device->RunOptionAction(Option::ZERO_DRIFT_CALIBRATION);

  device->SetStreamCallback(
      Stream::LEFT, [](const device::StreamData &data) {
        //CHECK_NOTNULL(data.img);
	});
  device->SetStreamCallback(
      Stream::RIGHT, [](const device::StreamData &data) {
        //CHECK_NOTNULL(data.img);
      });

    // Enable this will cache the motion datas until you get them.
  device->EnableMotionDatas();
  device->Start(Source::ALL);

  while (flag) 
  {
    device->WaitForStreams();

    device::StreamData left_data = device->GetLatestStreamData(Stream::LEFT);
    device::StreamData right_data = device->GetLatestStreamData(Stream::RIGHT);
    cv::Mat left_img(
        left_data.frame->height(), left_data.frame->width(), CV_8UC1,
        left_data.frame->data());
    cv::Mat right_img(
        right_data.frame->height(), right_data.frame->width(), CV_8UC1,
        right_data.frame->data());

    igb.GrabStereo(left_img, right_img, left_data.img->timestamp*0.00001f);
   }

   SLAM.Shutdown();
   device->Stop(Source::ALL);
   std::cout << "save camera trajectory..." << std::endl;
   SLAM.SaveTrajectoryKITTI("CameraTrajectory.txt");
   std::cout << "save camera trajectory complete." << std::endl;

    return 0;
}
