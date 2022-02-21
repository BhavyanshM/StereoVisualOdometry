//
// Created by quantum on 2/6/22.
//

#ifndef MAP_SENSE_VISUALODOMETRYCLAYLAUNCHER_H
#define MAP_SENSE_VISUALODOMETRYCLAYLAUNCHER_H

#include "ApplicationLauncher.h"
#include "VisualOdometry.h"
#include "DataManager.h"

   class VisualOdometryClayLauncher : public Clay::ApplicationLauncher
{

   public:
      VisualOdometryClayLauncher(int argc, char **argv);

      void MapsenseUpdate() override;

      void ImGuiUpdate(ApplicationState& appState) override {};

   private:
      VisualOdometry* _vo;
      DataManager* _data;
      Clay::Ref <Clay::PointCloud> firstCloud;
      ApplicationState appState;
      FactorGraphHandler fgh;
};

#endif //MAP_SENSE_VISUALODOMETRYCLAYLAUNCHER_H
