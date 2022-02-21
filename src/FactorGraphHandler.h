//
// Created by quantum on 1/24/22.
//

#ifndef MAP_SENSE_FACTORGRAPHHANDLER_H
#define MAP_SENSE_FACTORGRAPHHANDLER_H

#include <gtsam/geometry/Point2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/DoglegOptimizer.h>
#include <gtsam/slam/ProjectionFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/SimpleCamera.h>
#include <gtsam/geometry/Point3.h>

#include <vector>

#include "PointLandmark.h"

class FactorGraphHandler
{
   public:
      FactorGraphHandler() = default;

      void VisualISAM2Example();

      void ComputeNonLinear(std::vector<gtsam::Point2>& measurements, std::vector<gtsam::Pose3>& poses, std::vector<gtsam::Point3>& points);

      void Update(std::vector<PointLandmark>& landmarks, std::vector<Eigen::Matrix4f>& eigenPoses);

      std::vector<gtsam::Point3> createPoints();

      std::vector<gtsam::Pose3> createPoses();

   private:
      // Create a Factor Graph and Values to hold the new data
      gtsam::NonlinearFactorGraph graph;
      gtsam::Values initial;
      gtsam::Cal3_S2::shared_ptr K;
      gtsam::Vector6 odomVariance;
      gtsam::noiseModel::Diagonal::shared_ptr odometryNoise;
      gtsam::noiseModel::Isotropic::shared_ptr cameraMeasurementNoise;


};

#endif //MAP_SENSE_FACTORGRAPHHANDLER_H
