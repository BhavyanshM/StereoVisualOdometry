#include "cstdio"
#include "VisualOdometry.h"



void run(VisualOdometry* vo, DataManager* data, ApplicationState& appState);




int main(int argc, char** argv)
{
   printf("VisualOdometry");

   ApplicationState appState;
   appState.STEREO_ODOMETRY_ENABLED = true;
   appState.DATASET_ENABLED = true;

   DataManager* data = new DataManager(appState, "/home/quantum/Workspace/Storage/Other/Temp/dataset/sequences/00/image_0/",
                           "/home/quantum/Workspace/Storage/Other/Temp/dataset/sequences/00/image_1/",
                           "/home/quantum/Workspace/Storage/Other/Temp/dataset/data_odometry_poses/poses/00.txt");


   /* KITTI:  float fx = 718.856, fy = 718.856, cx = 607.193, cy = 185.216; */
   /* L515 Color: fx = 602.25927734375, cx = 321.3750915527344, fy = 603.0400390625, cy = 240.51527404785156; */
   //   data->SetCamera(CameraParams(602.25927734375, 603.0400390625, 321.3750915527344, 240.51527404785156),
   //                    CameraParams(602.25927734375, 603.0400390625, 321.3750915527344, 240.51527404785156));

   printf("Params: %.2lf %.2lf %.2lf %.2lf\n", appState.KITTI_CAM_PARAMS._fx, appState.KITTI_CAM_PARAMS._cx, appState.KITTI_CAM_PARAMS._fy, appState.KITTI_CAM_PARAMS._cy);
   VisualOdometry* vo = new VisualOdometry(argc, argv, appState, data);


   BundleAdjustment* ba = new BundleAdjustment();
   ba->Optimize();

//   while(true)
//   {
//      run(vo, data, appState);
//   }

}

void run(VisualOdometry* vo, DataManager* data, ApplicationState& appState)
{
   bool result = vo->Update(appState);
   vo->Show();
}


