#include "DataManager.h"
#include "AppUtils.h"
#include "filesystem"
#include "boost/filesystem.hpp"
#include <random>

DataManager::DataManager(ApplicationState& appState)
    : _directory(appState.DATASET_PATH + "image_0/"), _secondDirectory(appState.DATASET_PATH + "image_1/")
{
   printf("DataManager: %s\n", _directory.c_str());

   _leftCam.SetParams(718.856, 718.856, 607.193, 185.216);
   _rightCam.SetParams(718.856, 718.856, 607.193, 185.216);
   _baseline =0.54;

   /* KITTI Images */
   AppUtils::getFileNames(_directory, _fileNames, false);
  if(_secondDirectory != "") AppUtils::getFileNames(_secondDirectory, _secondFileNames, false);

        /* Ground Truth Poses*/
//        if(poseFile != "")
//        {
//            ifstream in_file;
//            in_file.open(poseFile);
//            _poses = xt::load_csv<double>(in_file, ' ');
//
//            _poses.reshape({-1, 12});
//
//            CLAY_LOG_INFO("Poses Shape: {} {}", _poses.shape().at(0), _poses.shape().at(1));
//        }


}

void DataManager::ShowNext()
{
   cv::Mat nextImage = GetNextImage();
   cv::imshow("DataManager", nextImage);
   if(_secondDirectory != "")
   {
      cv::Mat nextSecondImage = GetNextSecondImage();
      cv::imshow("DataManager Stereo", nextSecondImage);
   }
   cv::waitKey(1);
}

cv::Mat DataManager::GetNextImage()
{
   printf("Loading Image (%d) : %s\n", _counter, (_directory + _fileNames[_counter]).c_str());
   if(_counter == _fileNames.size() - 1) _counter = 0;
   return cv::imread(_directory + _fileNames[_counter++], cv::IMREAD_COLOR);
}

cv::Mat DataManager::GetNextSecondImage()
{
   std::cout << "Loading Image: " << (_secondDirectory + _secondFileNames[_secondCounter]) << std::endl;
   if(_secondCounter == _secondFileNames.size() - 1) _secondCounter = 0;
   return cv::imread(_secondDirectory + _secondFileNames[_secondCounter++], cv::IMREAD_COLOR);
}

cv::Mat DataManager::GetImageByIndex(uint32_t index)
{
   std::cout << "Loading Image: " << (_directory + _fileNames[index]) << std::endl;
   if(index >= _fileNames.size() - 1) index = 0;
   return cv::imread(_directory + _fileNames[index], cv::IMREAD_COLOR);
}

cv::Mat DataManager::GetSecondImageByIndex(uint32_t index)
{
   std::cout << "Loading Image: " << (_secondDirectory + _secondFileNames[index]) << std::endl;
   if(index >= _secondFileNames.size() - 1) index = 0;
   return cv::imread(_secondDirectory + _secondFileNames[index], cv::IMREAD_COLOR);
}


cv::Mat DataManager::ReadImage(std::string filename)
{
   std::cout << "PATH:" << filename << std::endl;
   return imread(filename, cv::IMREAD_COLOR);
}

void DataManager::load_sample_depth(std::string filename, cv::Mat& depth)
{
   depth = imread(filename, cv::IMREAD_ANYDEPTH);
}

void DataManager::get_sample_color(cv::Mat color)
{
   for (int i = 0; i < color.rows; i++)
   {
      for (int j = 0; j < color.cols; j++)
      {
         color.at<cv::Vec3b>(i, j) = cv::Vec3b(0, 0, 255);
      }
   }
}

void DataManager::get_sample_depth(cv::Mat depth, float mean, float stddev)
{
   std::default_random_engine generator;
   std::normal_distribution<double> distribution(mean, stddev);
   for (int i = 0; i < depth.cols; i++)
   {
      for (int j = 0; j < depth.rows; j++)
      {
         float d = 10.04;
         d += distribution(generator);
         if (160 < i && i < 350 && 200 < j && j < 390)
         {
            // d = 0.008 * i + 0.014 * j;
            depth.at<short>(j, i) = (d - 2.0f) * 1000;
         } else
         {
            depth.at<short>(j, i) = d * 1000;
         }
      }
   }
}

void DataManager::load_sample_color(std::string filename, cv::Mat& color)
{
   color = imread(filename, cv::IMREAD_COLOR);
}
