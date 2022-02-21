//
// Created by quantum on 2/6/22.
//

#include "VisualOdometryClayLauncher.h"

void extractArgs(int argc, char **argv, ApplicationState& appState)
{
   std::vector<std::string> args(argv, argv + argc);
   for (int i = 0; i < args.size(); i++)
   {
      if (args[i] == "--dataset-path")
      {
         std::string dirPath;
         dirPath = args[i + 1];
         appState.DATASET_PATH = dirPath;
      }
   }
}

VisualOdometryClayLauncher::VisualOdometryClayLauncher(int argc, char **argv) : Clay::ApplicationLauncher(argc, argv)
{
   printf("VisualOdometry");

   appState.STEREO_ODOMETRY_ENABLED = true;
   appState.DATASET_ENABLED = true;

   extractArgs(argc, argv, appState);

   _data = new DataManager(appState);

   /* KITTI:  float fx = 718.856, fy = 718.856, cx = 607.193, cy = 185.216; */
   /* L515 Color: fx = 602.25927734375, cx = 321.3750915527344, fy = 603.0400390625, cy = 240.51527404785156; */
   //   data->SetCamera(CameraParams(602.25927734375, 603.0400390625, 321.3750915527344, 240.51527404785156),
   //                    CameraParams(602.25927734375, 603.0400390625, 321.3750915527344, 240.51527404785156));

   printf("Params: %.2lf %.2lf %.2lf %.2lf\n", appState.KITTI_CAM_PARAMS._fx, appState.KITTI_CAM_PARAMS._cx, appState.KITTI_CAM_PARAMS._fy,
          appState.KITTI_CAM_PARAMS._cy);

   _vo = new VisualOdometry(argc, argv, appState, _data, fgh);

}

void VisualOdometryClayLauncher::MapsenseUpdate()
{
   bool result = _vo->Update(appState);
   _vo->Show();
}

/* ///////////////////////////////////////////////////////////
 * //////////////// Application Begins //////////////////////
 * //////////////////////////////////////////////////////////*/

class VisualOdometryApp : public Clay::Application
{
   public:
      VisualOdometryApp(int argc, char** argv);
      ~VisualOdometryApp() = default;
};


VisualOdometryApp::VisualOdometryApp(int argc, char** argv)
{
   //   PushLayer(new ExampleLayer());
   VisualOdometryClayLauncher* app = new VisualOdometryClayLauncher(argc, argv);
   PushLayer(app);
}

int main(int argc, char** argv)
{
   Clay::Log::Init();   CLAY_LOG_INFO("Welcome to Clay Engine!");

   CLAY_PROFILE_BEGIN_SESSION("Startup", "ClayProfile-Startup.json");
   VisualOdometryApp app(argc, argv);
   CLAY_PROFILE_END_SESSION();

   CLAY_PROFILE_BEGIN_SESSION("Runtime", "ClayProfile-Runtime.json");
   app.Run();
   CLAY_PROFILE_END_SESSION();

   return 0;
}