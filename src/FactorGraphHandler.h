//
// Created by quantum on 1/24/22.
//

#ifndef MAP_SENSE_FACTORGRAPHHANDLER_H
#define MAP_SENSE_FACTORGRAPHHANDLER_H

// For loading the data
//#include "SFMdata.h"

// Camera observations of landmarks will be stored as Point2 (x, y).
#include <gtsam/geometry/Point2.h>

// Each variable in the system (poses and landmarks) must be identified with a
// unique key. We can either use simple integer keys (1, 2, 3, ...) or symbols
// (X1, X2, L1). Here we will use Symbols
#include <gtsam/inference/Symbol.h>

// We want to use iSAM2 to solve the structure-from-motion problem
// incrementally, so include iSAM2 here
#include <gtsam/nonlinear/ISAM2.h>

// iSAM2 requires as input a set of new factors to be added stored in a factor
// graph, and initial guesses for any new variables used in the added factors
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/DoglegOptimizer.h>

// In GTSAM, measurement functions are represented as 'factors'. Several common
// factors have been provided with the library for solving robotics/SLAM/Bundle
// Adjustment problems. Here we will use Projection factors to model the
// camera's landmark observations. Also, we will initialize the robot at some
// location using a Prior factor.
#include <gtsam/slam/ProjectionFactor.h>

#include <vector>


#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>

// We will also need a camera object to hold calibration information and perform projections.
#include <gtsam/geometry/SimpleCamera.h>

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


};

#endif //MAP_SENSE_FACTORGRAPHHANDLER_H
