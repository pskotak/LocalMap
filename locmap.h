#ifndef LOCMAP_H
#define LOCMAP_H

#include <atomic>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp> // For glm::quat
#include <glm/gtx/euler_angles.hpp> // For glm::yawPitchRoll

#include "../SayaGlobals/globals.h"

#define GridResolutionM 0.08f
#define GridSizeM 8.08f
#define GridCells 101 //(GridSizeM / GridResolutionM)
#define GridCenter 51
#define GridHeightLimitTop 0.7
#define GridHeightLimitBot -1.5

#define GridUpscale 4.0

extern std::atomic<uint32_t> Vrtule;

extern std::atomic<bool> NewLocMap;
extern std::atomic<bool> ShutdownLocMap;
extern cv::Mat show_grid;
extern cv::Mat flat_ethgrid;
extern cv::Mat showDist;

extern void RunLocMap();

#endif
