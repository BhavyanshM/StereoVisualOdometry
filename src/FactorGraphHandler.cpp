//
// Created by quantum on 1/24/22.
//

#include "FactorGraphHandler.h"




/* ************************************************************************* */
std::vector<gtsam::Point3> FactorGraphHandler::createPoints() {

   // Create the set of ground-truth landmarks
   std::vector<gtsam::Point3> points;
   points.push_back(gtsam::Point3(10.0,10.0,10.0));
   points.push_back(gtsam::Point3(-10.0,10.0,10.0));
   points.push_back(gtsam::Point3(-10.0,-10.0,10.0));
   points.push_back(gtsam::Point3(10.0,-10.0,10.0));
   points.push_back(gtsam::Point3(10.0,10.0,-10.0));
   points.push_back(gtsam::Point3(-10.0,10.0,-10.0));
   points.push_back(gtsam::Point3(-10.0,-10.0,-10.0));
   points.push_back(gtsam::Point3(10.0,-10.0,-10.0));

   return points;
}

/* ************************************************************************* */
std::vector<gtsam::Pose3> FactorGraphHandler::createPoses() {

   // Create the set of ground-truth poses
   std::vector<gtsam::Pose3> poses;
   double radius = 30.0;
   int i = 0;
   double theta = 0.0;
   gtsam::Point3 up(0,0,1);
   gtsam::Point3 target(0,0,0);
   for(; i < 8; ++i, theta += 2*M_PI/8) {
      gtsam::Point3 position = gtsam::Point3(radius*cos(theta), radius*sin(theta), 0.0);
      gtsam::SimpleCamera camera = gtsam::SimpleCamera::Lookat(position, target, up);
      poses.push_back(camera.pose());
   }
   return poses;
}
/* ************************************************************************* */

void FactorGraphHandler::VisualISAM2Example()
{
   // Define the camera calibration parameters
   gtsam::Cal3_S2::shared_ptr K(new gtsam::Cal3_S2(50.0, 50.0, 0.0, 50.0, 50.0));

   // Define the camera observation noise model, 1 pixel stddev
   auto measurementNoise = gtsam::noiseModel::Isotropic::Sigma(2, 1.0);

   // Create the set of ground-truth landmarks
   std::vector<gtsam::Point3> points = createPoints();

   // Create the set of ground-truth poses
   std::vector<gtsam::Pose3> poses = createPoses();

   // Create an iSAM2 object. Unlike iSAM1, which performs periodic batch steps
   // to maintain proper linearization and efficient variable ordering, iSAM2
   // performs partial relinearization/reordering at each step. A parameter
   // structure is available that allows the user to set various properties, such
   // as the relinearization threshold and type of linear solver. For this
   // example, we we set the relinearization threshold small so the iSAM2 result
   // will approach the batch result.
   gtsam::ISAM2Params parameters;
   parameters.relinearizeThreshold = 0.01;
   parameters.relinearizeSkip = 1;
   gtsam::ISAM2 isam(parameters);

   // Create a Factor Graph and Values to hold the new data
   gtsam::NonlinearFactorGraph graph;
   gtsam::Values initialEstimate;

   // Loop over the poses, adding the observations to iSAM incrementally
   for (size_t i = 0; i < poses.size(); ++i) {
      // Add factors for each landmark observation
      for (size_t j = 0; j < points.size(); ++j) {
         gtsam::PinholeCamera<gtsam::Cal3_S2> camera(poses[i], *K);
         gtsam::Point2 measurement = camera.project(points[j]);
         graph.emplace_shared<gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3_S2> >(
               measurement, measurementNoise, gtsam::Symbol('x', i), gtsam::Symbol('l', j), K);
      }

      // Add an initial guess for the current pose
      // Intentionally initialize the variables off from the ground truth
      static gtsam::Pose3 kDeltaPose(gtsam::Rot3::Rodrigues(-0.1, 0.2, 0.25),
                                     gtsam::Point3(0.05, -0.10, 0.20));
      initialEstimate.insert(gtsam::Symbol('x', i), poses[i] * kDeltaPose);

      // If this is the first iteration, add a prior on the first pose to set the
      // coordinate frame and a prior on the first landmark to set the scale Also,
      // as iSAM solves incrementally, we must wait until each is observed at
      // least twice before adding it to iSAM.
      if (i == 0) {
         // Add a prior on pose x0, 30cm std on x,y,z and 0.1 rad on roll,pitch,yaw
         static auto kPosePrior = gtsam::noiseModel::Diagonal::Sigmas(
               (gtsam::Vector(6) << gtsam::Vector3::Constant(0.1), gtsam::Vector3::Constant(0.3))
                     .finished());
         graph.addPrior(gtsam::Symbol('x', 0), poses[0], kPosePrior);

         // Add a prior on landmark l0
         static auto kPointPrior = gtsam::noiseModel::Isotropic::Sigma(3, 0.1);
         graph.addPrior(gtsam::Symbol('l', 0), points[0], kPointPrior);

         // Add initial guesses to all observed landmarks
         // Intentionally initialize the variables off from the ground truth
         static gtsam::Point3 kDeltaPoint(-0.25, 0.20, 0.15);
         for (size_t j = 0; j < points.size(); ++j)
            initialEstimate.insert<gtsam::Point3>(gtsam::Symbol('l', j), points[j] + kDeltaPoint);

      } else {

         graph.print("Graph");
         initialEstimate.print("Initial");
         isam.printStats();
         isam.print("ISAM");

         // Update iSAM with the new factors
         isam.update(graph, initialEstimate);
         // Each call to iSAM2 update(*) performs one iteration of the iterative
         // nonlinear solver. If accuracy is desired at the expense of time,
         // update(*) can be called additional times to perform multiple optimizer
         // iterations every step.
         isam.update();
         gtsam::Values currentEstimate = isam.calculateEstimate();
         std::cout << "****************************************************" << std::endl;
         std::cout << "Frame " << i << ": " << std::endl;
         currentEstimate.print("Current estimate: ");

         // Clear the factor graph and values for the next iteration
         graph.resize(0);
         initialEstimate.clear();
      }
   }
}
