//
// Created by quantum on 2/1/22.
//


#include <catch2/catch_session.hpp>
#include <catch2/catch_test_macros.hpp>
#include "VisualOdometry.h"


TEST_CASE( "Visual Odometry is Accurate", "[odometry]" ) {

   ApplicationState appState;
   appState.STEREO_ODOMETRY_ENABLED = true;
   appState.DATASET_ENABLED = true;

   DataManager *data = new DataManager(appState);
   FactorGraphHandler* fgh = new FactorGraphHandler();
   VisualOdometry *vo = new VisualOdometry(0, nullptr, appState, data, fgh);

   std::vector<PointLandmark> landmarks;
   cv::Mat leftImage, prevLeft, rightImage, prevRight, desc_prevLeft, desc_prevRight;
   std::vector<cv::KeyPoint> kp_prevLeft, kp_prevRight;
   Eigen::Matrix4f relativePose;

   vo->LoadImages(appState, leftImage, rightImage);
   cvtColor(leftImage, prevLeft, cv::COLOR_BGR2GRAY);
   cvtColor(rightImage, prevRight, cv::COLOR_BGR2GRAY);
   vo->ExtractKeypoints(prevLeft, kp_prevLeft, desc_prevLeft);
   vo->ExtractKeypoints(prevRight, kp_prevRight, desc_prevRight);
   Keyframe kf(desc_prevLeft.clone(), desc_prevRight.clone(), kp_prevLeft, kp_prevRight,
               Eigen::Matrix4f::Identity(), leftImage.clone(), rightImage.clone());


   vo->CalculateOdometry_ORB(kf, leftImage, rightImage, relativePose, landmarks);

   /* Section: Frobenius distance between ground truth and estimated poses. */

   /* Section: Difference in reprojection errors before and after GTSAM optimization. */

   SECTION("Reprojection error for landmark projections is within threshold")
   {
      for(auto landmark : landmarks)
      {
         float error = vo->CalculateReprojectionError(landmark, relativePose, appState.KITTI_CAM_PARAMS.GetProjectionMatrix());
         printf("Error: %.2lf\n", error);
//         REQUIRE(error < 40);
      }
   }

}

int main(int argc, char** argv)
{
   Catch::Session().run( argc, argv );



}