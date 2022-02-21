//
// Created by quantum on 6/30/21.
//

#ifndef VISUAL_ODOMETRY_H
#define VISUAL_ODOMETRY_H

#include "opencv2/opencv.hpp"
#include "ApplicationState.h"
#include "PointLandmark.h"
#include "CameraParams.h"
#include "BundleAdjustment.h"
#include "DataManager.h"
#include "../FactorGraphHandler.h"

struct Keyframe
{
   public:
      Keyframe(cv::Mat desc, std::vector<cv::KeyPoint> kp, Eigen::Matrix4f pose)
            : _descLeft(desc), _keypointsLeft(kp), _pose(pose) {};
      Keyframe(cv::Mat descLeft, cv::Mat descRight, std::vector<cv::KeyPoint> kpLeft, std::vector<cv::KeyPoint> kpRight, Eigen::Matrix4f pose, cv::Mat left, cv::Mat right)
            : _descLeft(descLeft), _descRight(descRight), _keypointsLeft(kpLeft), _keypointsRight(kpRight), _pose(pose), _leftImage(left), _rightImage(right) {};
      cv::Mat _descLeft, _descRight;
      std::vector<cv::KeyPoint> _keypointsLeft, _keypointsRight;
      Eigen::Matrix4f _pose;
      cv::Mat _leftImage, _rightImage;

      const cv::Mat& getDescLeft() const
      {
         return _descLeft;
      }

      void setDescLeft(const cv::Mat& descLeft)
      {
         _descLeft = descLeft;
      }

      const cv::Mat& getDescRight() const
      {
         return _descRight;
      }

      void setDescRight(const cv::Mat& descRight)
      {
         _descRight = descRight;
      }

      const std::vector<cv::KeyPoint>& getKeypointsLeft() const
      {
         return _keypointsLeft;
      }

      void setKeypointsLeft(const std::vector<cv::KeyPoint>& keypointsLeft)
      {
         _keypointsLeft = keypointsLeft;
      }

      const std::vector<cv::KeyPoint>& getKeypointsRight() const
      {
         return _keypointsRight;
      }

      void setKeypointsRight(const std::vector<cv::KeyPoint>& keypointsRight)
      {
         _keypointsRight = keypointsRight;
      }

      const Eigen::Matrix4f& getPose() const
      {
         return _pose;
      }

      void setPose(const Eigen::Matrix4f& pose)
      {
         _pose = pose;
      }

      const cv::Mat& getLeftImage() const
      {
         return _leftImage;
      }

      void setLeftImage(const cv::Mat& leftImage)
      {
         _leftImage = leftImage;
      }

      const cv::Mat& getRightImage() const
      {
         return _rightImage;
      }

      void setRightImage(const cv::Mat& rightImage)
      {
         _rightImage = rightImage;
      }
};

class VisualOdometry
{
   public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      VisualOdometry(int argc, char **argv, ApplicationState& app, DataManager *data, FactorGraphHandler& fgh);
      void LoadImages(ApplicationState& appState, cv::Mat& left, cv::Mat& right, uint32_t index = -1);
      void Initialize();
      bool Update(ApplicationState& appState);

      void ExtractKeypoints_FAST(cv::Mat img_1, std::vector<cv::Point2f>& points1);
      void ExtractKeypoints(cv::Mat img, std::vector<cv::KeyPoint>& points, cv::Mat& desc);
      void TrackKeypoints(cv::Mat prev, cv::Mat cur, std::vector<cv::Point2f>& prev_pts, std::vector<cv::Point2f>& cur_pts);
      void MatchKeypoints(cv::Mat& desc1, cv::Mat& desc2, std::vector<cv::DMatch>& matches);
      void GridSampleKeypoints(std::vector<cv::KeyPoint>& keypoints, std::vector<cv::DMatch>& matches);
      void ExtractFinalSet(std::vector<cv::DMatch> leftMatches, std::vector<cv::KeyPoint> curLeftKp, std::vector<PointLandmark>& points3D);
      void ExtractPoseLinear();

      void DrawMatches(cv::Mat& img, std::vector<cv::Point2f> prev_pts, std::vector<cv::Point2f> cur_pts);
      void DrawLandmarks(cv::Mat& img, std::vector<PointLandmark>& landmarks);
      void DrawAllMatches(cv::Mat& image);
      const Eigen::Matrix4f& EstimateMotion(std::vector<PointLandmark> points, int cameraID);
      void EstimateMotion_2D2D(std::vector<cv::Point2f>& prevFeatures, std::vector<cv::Point2f>& curFeatures, cv::Mat& mask, const CameraParams& cam, Eigen::Matrix4f& pose);

      void Show();
      void TriangulateStereoNormal(std::vector<cv::KeyPoint>& pointsTrain, std::vector<cv::KeyPoint>& pointsQuery, std::vector<cv::DMatch>& matches,
                                   std::vector<PointLandmark>& points3D);
      void TriangulateStereoPoints(cv::Mat& leftPoseWorld, std::vector<cv::KeyPoint> kpLeft, std::vector<cv::KeyPoint> kpRight, std::vector<cv::DMatch> stereoMatches,
                                   std::vector<PointLandmark> points3D);

      cv::Mat TriangulatePoints(std::vector<cv::Point2f>& prevPoints, std::vector<cv::Point2f>& curPoints, const CameraParams& cam, cv::Mat relativePose);

      void CalculateOdometry_ORB(Keyframe& kf, cv::Mat leftImage, cv::Mat rightImage, Eigen::Matrix4f& pose, std::vector<PointLandmark>& points3D);
      void CalculateOdometry_FAST(ApplicationState& appState, Eigen::Matrix4f& transform);
      void ImGuiUpdate(ApplicationState& app);

      cv::Mat CalculateStereoDepth(cv::Mat left, cv::Mat right);

      float CalculateReprojectionError(PointLandmark landmark, Eigen::Matrix4f pose, Eigen::Matrix<float, 3, 4> projection);

   private:

      Eigen::Matrix4f cameraPose;

      bool _initialized = false;
      uint32_t count = 0;
      uint32_t kFeatures = 600;
      uint32_t kMinFeatures = 300;
      uint32_t width = 0;
      uint32_t height = 0;
      uint32_t xGridCount = 60;
      uint32_t yGridCount = 30;
      float scalar = 0.03f;


      cv::Ptr<cv::ORB> orb = cv::ORB::create(kFeatures);
      cv::Mat prevLeft, prevRight, curLeft, curRight, leftImage, rightImage;
      std::vector<cv::DMatch> matchesLeft, matchesRight, prevMatchesStereo, curMatchesStereo;
      cv::Mat desc_prevRight, desc_prevLeft, desc_curRight, desc_curLeft;
      cv::Mat curFinalDisplay, prevFinalDisplay;
      cv::Mat curPoseLeft, prevPosLeft, curPoseRight, prevPoseRight;
      cv::Mat curDisparity;
      std::vector<cv::KeyPoint> kp_prevLeft, kp_prevRight, kp_curLeft, kp_curRight;
      Eigen::Matrix4f cvCurPose;
      std::vector<PointLandmark> _prevPoints3D, _curPoints3D;

      cv::Ptr<cv::StereoBM> stereo = cv::StereoBM::create();
      std::vector<cv::Point2f> prevFeaturesLeft, curFeaturesLeft;
      std::vector<cv::Point2f> prevPoints2D, curPoints2D;

      BundleAdjustment* _bundleAdjustment;
      DataManager *_data;

      std::vector<Keyframe> _keyframes;

      ApplicationState _app;
      FactorGraphHandler _fgh;
};

#endif //VISUAL_ODOMETRY_H
