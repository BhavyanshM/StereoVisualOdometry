//
// Created by quantum on 2/16/21.
//

#include <limits>
#include <iomanip>
#include <dirent.h>
#include "AppUtils.h"

void AppUtils::getFileNames(string dirName, vector<string>& files, bool printList)
{
   if (auto dir = opendir(dirName.c_str()))
   {
      while (auto f = readdir(dir))
      {

         if (!f->d_name || f->d_name[0] == '.')
            continue;
         files.emplace_back(f->d_name);
      }
      closedir(dir);
   }
   sort(files.begin(), files.end());

   if (printList)
   {
      printf("[");
      for (string file : files)
      {
         printf("%s, ", file.c_str());
      }
      printf("]\n");
   }
}

long long AppUtils::GetSteadyClockTime()
{
   auto start_point = std::chrono::steady_clock::now();
   return std::chrono::time_point_cast<std::chrono::microseconds>(start_point).time_since_epoch().count();
}

void AppUtils::appendToDebugOutput(cv::Mat disp)
{
   if (disp.rows <= 0 || disp.cols <= 0)
   {
      return;
   }
   if (disp.type() == CV_8UC1)
      cvtColor(disp, disp, cv::COLOR_GRAY2BGR);
   if (disp.type() == CV_16UC3 || disp.type() == CV_16UC1)
   {
      disp.convertTo(disp, CV_8U, 0.00390625);
      disp.convertTo(disp, CV_8UC3);
   }
   cv::resize(disp, disp, cv::Size(640 * 480 / disp.rows, 480));
   images.emplace_back(disp);
}

void AppUtils::displayDebugOutput(ApplicationState appState)
{
   hconcat(images, debugOutput);
   if (debugOutput.cols > 0 && debugOutput.rows > 0 && !debugOutput.empty())
   {
      cv::namedWindow("DebugOutput", cv::WINDOW_NORMAL);
      cv::resizeWindow("DebugOutput", (int) (debugOutput.cols * appState.DISPLAY_WINDOW_SIZE), (int) (debugOutput.rows * appState.DISPLAY_WINDOW_SIZE));
      cv::imshow("DebugOutput", debugOutput);
      cv::waitKey(1);
   }
   clearDebug();
}

void AppUtils::clearDebug()
{
   images.clear();
}

void AppUtils::setDisplayResolution(uint16_t rows, uint16_t cols)
{
   displayOutput = cv::Mat(rows, cols, CV_8UC3);
   displayOutput.setTo(0);
}

void AppUtils::canvasToMat(BoolDynamicMatrix canvas, Eigen::Vector2i windowPos, uint8_t windowSize)
{
   for (int i = 0; i < canvas.rows(); i++)
   {
      for (int j = 0; j < canvas.cols(); j++)
      {
         if (canvas(i, j) == 1)
            circle(AppUtils::displayOutput, cv::Point(i * 6, j * 6), 2, cv::Scalar(255, 0, 0), -1);
         else if (windowPos.x() != -1 && windowPos.y() != -1 && i > windowPos.x() - windowSize && i < windowPos.x() + windowSize &&
                  j > windowPos.y() - windowSize && j < windowPos.y() + windowSize)
         {
            circle(AppUtils::displayOutput, cv::Point(i * 6, j * 6), 2, cv::Scalar(0, 0, 255), -1);
         }
      }
   }
}

void AppUtils::displayPointSet2D(vector<Eigen::Vector2f> points, Eigen::Vector2f offset, int scale)
{
   displayOutput.setTo(0);
   for (int i = 0; i < points.size(); i++)
   {
      circle(AppUtils::displayOutput, cv::Point((int) (points[i].x() * scale + offset.x()) * 6, (int) (points[i].y() * scale + offset.y()) * 6), 2,
             cv::Scalar(255, 0, 0), -1);
      display(20);
   }
}

void AppUtils::displayCanvasWithWindow(BoolDynamicMatrix canvas, Eigen::Vector2i windowPos, uint8_t windowSize)
{
   canvasToMat(canvas, windowPos, windowSize);
   display(20);
}

void AppUtils::display(uint16_t delay)
{
   if (displayOutput.cols > 0 && displayOutput.rows > 0 && !displayOutput.empty())
   {
      cv::namedWindow("Display", cv::WINDOW_NORMAL);
      cv::resizeWindow("Display", (int) (displayOutput.cols), (int) (displayOutput.rows));
      cv::imshow("Display", displayOutput);
      cv::waitKey(delay);
   }
}

void AppUtils::checkMemoryLimits()
{
//   struct rlimit old_lim, lim, new_lim;
//
//   // Get old limits
//   if (getrlimit(RLIMIT_NOFILE, &old_lim) == 0)
//   {
//      printf("Old limits -> soft limit= %ld \t"
//             " hard limit= %ld \n", old_lim.rlim_cur, old_lim.rlim_max);
//   } else
//   {
//      fprintf(stderr, "%s\n", strerror(errno));
//   }
//
//   // Set new value
//   lim.rlim_cur = 1024 * 1024 * 1024;
//   lim.rlim_max = 1024 * 1024 * 1024;
//
//   // Set limits
//   if (setrlimit(RLIMIT_NOFILE, &lim) == -1)
//   {
//      fprintf(stderr, "%s\n", strerror(errno));
//   }
//   // Get new limits
//   if (getrlimit(RLIMIT_NOFILE, &new_lim) == 0)
//   {
//      printf("New limits -> soft limit= %ld "
//             "\t hard limit= %ld \n", new_lim.rlim_cur, new_lim.rlim_max);
//   } else
//   {
//      fprintf(stderr, "%s\n", strerror(errno));
//   }
}

void AppUtils::DisplayImage(cv::Mat disp, const ApplicationState& app)
{
   if (disp.cols > 0 && disp.rows > 0 && !disp.empty())
   {
      cv::namedWindow("DisplayImage", cv::WINDOW_NORMAL);
      cv::resizeWindow("DisplayImage", (int) (disp.cols * app.DISPLAY_WINDOW_SIZE), (int) (disp.rows * app.DISPLAY_WINDOW_SIZE));
      cv::imshow("DisplayImage", disp);
      cv::waitKey(1);
   }
}

