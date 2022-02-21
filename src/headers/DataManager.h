//
// Created by quantum on 5/15/21.
//

#ifndef FILEIO_H
#define FILEIO_H

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"
#include "ApplicationState.h"
#include "CameraParams.h"

class DataManager
{
   public:
      DataManager() = default;

      DataManager(ApplicationState& appState);

      cv::Mat GetImageByIndex(uint32_t index);

      cv::Mat GetSecondImageByIndex(uint32_t index);

      void get_sample_depth(cv::Mat depth, float mean, float stddev);

      void get_sample_color(cv::Mat color);

      void load_sample_depth(std::string filename, cv::Mat& depth);

      void load_sample_color(std::string filename, cv::Mat& color);

      cv::Mat GetNextImage();

      cv::Mat GetNextSecondImage();

      void ShowNext();

      static cv::Mat ReadImage(std::string filename);



      void SetCamera(CameraParams& leftCam, CameraParams& rightCam)
      {
          _leftCam = leftCam; _rightCam = rightCam;
      };

      CameraParams& GetLeftCamera() {return _leftCam; }

      CameraParams& GetRightCamera() {return _rightCam; }

      void SetStereoBaseline(float baseline) { _baseline = baseline; }

      float GetStereoBaseline() const { return _baseline;}

   private:
      float _baseline;
      std::string _directory, _secondDirectory;
      std::vector<std::string> _fileNames;
      std::vector<std::string> _secondFileNames;
      uint32_t _counter = 0;
      uint32_t _secondCounter = 0;
      CameraParams _leftCam, _rightCam;
};

#endif //FILEIO_H
