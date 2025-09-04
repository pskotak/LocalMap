#ifndef LOCMAP_H
#define LOCMAP_H

#include "../SayaGlobals/globals.h"

namespace locmap {

#define GridResolutionM 0.08f
#define GridSizeM 8.08f
#define GridCells 101 //(GridSizeM / GridResolutionM)
#define GridCenter 51

#define GridHeightLimitTop 0.7
// 0.7 m nad stred depth kamery
// - Vyssi prekazky jsou ignorovany a neobjevi se v mape
// - Podjedeme je
#define GridHeightLimitBot -1.5
// 1.5 m pod stred depth kamery
// - Nizsi prekazky jsou ignorovany a neobjevi se v mape
// - Prejedeme je nebo koukame do diry

extern float ObstacleDelta; // TODO Nacitat z configu
extern int InflateRadius; // TODO Nacitat z configu
extern std::atomic<bool> UpdateGridMap;

// Flat grid map: Cells - hlavni vystup s binarizovanymi a nafouknutymi prekazkam
typedef struct {
    bool unknown; // binarizovana neznama
    bool free; // binarizovana volna
    bool occupied; // binarizovana obsazena
    bool obstacle; // volna, ale nafouknutim oznacena jako prekazka
    bool path; // je to path?
} TCell;
extern TCell ObstacleGrid[GridCells][GridCells];
//extern std::atomic<uint32_t> Vrtule;

extern std::atomic<bool> ShutdownLocMap;
extern cv::Mat lmap_depth_image; // DEPRECATED - Input depth bitmap CV16C1 -> prevzit pointcloud z modulu vision

extern void RunLocMap();
extern void SetGoal(const int Xidx, const int Yidx);
extern void Plan();

} // end namespace

#endif
