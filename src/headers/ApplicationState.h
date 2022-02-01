//
// Created by quantum on 12/24/20.
//

#ifndef SRC_APPLICATIONSTATE_H
#define SRC_APPLICATIONSTATE_H

#include "string"
#include "CameraParams.h"

class ApplicationState
{
   public:
      ApplicationState()
      {
         KITTI_CAM_PARAMS = CameraParams(718.856, 718.856, 607.193, 185.216);
      }

      const std::string& getDepthFile() const;

      void setDepthFile(const std::string& depthFile);

      const std::string& getColorFile() const;

      void setColorFile(const std::string& colorFile);

      void update();

   public:
      /*
       * NOTE: The following parameters should meet these requirements.
       * a) InputHeight should be divisible by (KernelResLevel * AspectRatioHeight)
       * b) InputWidth should be divisible by (KernelResLevel * AspectRatioWidth)
       * */

      int INPUT_HEIGHT = 0;
      int INPUT_WIDTH = 0;
      int KERNEL_SLIDER_LEVEL = 2;
      int PATCH_HEIGHT = KERNEL_SLIDER_LEVEL;
      int PATCH_WIDTH = KERNEL_SLIDER_LEVEL;
      int SUB_H = INPUT_HEIGHT / PATCH_HEIGHT;
      int SUB_W = INPUT_WIDTH / PATCH_WIDTH;

      int FILTER_KERNEL_SIZE = 4;
      int FILTER_SUB_H = INPUT_HEIGHT / FILTER_KERNEL_SIZE;
      int FILTER_SUB_W = INPUT_WIDTH / FILTER_KERNEL_SIZE;

      bool STEREO_ODOMETRY_ENABLED = false;
      bool DATASET_ENABLED = false;

      /*  VISUALIZATION-ONLY */
      float DISPLAY_WINDOW_SIZE = 1.5f;

      /* Stereo Matching Parameters */
      int STEREO_NUM_DISPARITIES = 1;
      int STEREO_BLOCK_SIZE = 2;
      int STEREO_PRE_FILTER_SIZE = 2;
      int STEREO_PRE_FILTER_TYPE = 1;
      int STEREO_PRE_FILTER_CAP = 31;
      int STEREO_MIN_DISPARITY = 0;
      int STEREO_TEXTURE_THRESHOLD = 10;
      int STEREO_UNIQUENESS_RATIO = 15;
      int STEREO_SPECKLE_RANGE = 0;
      int STEREO_SPECKLE_WINDOW_SIZE = 0;
      int STEREO_DISP_12_MAX_DIFF = -1;

      CameraParams KITTI_CAM_PARAMS;

      std::string DATASET_PATH = "../datasets/KITTI_Sequence_00/";


};

#endif //SRC_APPLICATIONSTATE_H
