#ifndef LOCMAP_H
#define LOCMAP_H

#include "../SayaGlobals/globals.h"

namespace locmap {

#define LocMapVersion "1.0.1"

// Point cloud
extern int LocmapPC_start;
extern int LocmapPC_end;
extern int LocmapPC_IgnoreFromLeft;

#define GridResolutionM 0.08f
#define GridSizeM 8.08f
#define GridCells 101 //(GridSizeM / GridResolutionM)
#define GridCenter 50

#define GridHeightLimitTop 0.7f
// 0.7 m nad stred depth kamery
// - Vyssi prekazky jsou ignorovany a neobjevi se v mape
// - Podjedeme je
#define GridHeightLimitBot -1.5f
// 1.5 m pod stred depth kamery
// - Nizsi prekazky jsou ignorovany a neobjevi se v mape
// - Prejedeme je nebo koukame do diry

// VFH
#define AngleResDeg 5
#define Sectors (360 / AngleResDeg) // Muai byt cele cislo
#define AngleResRad (AngleResDeg * DegRad)
#define SectorOffs  (PI / AngleResRad)
extern float VFHisto[Sectors];
extern float RawHisto[Sectors];

extern float ObstacleDelta; // TODO Nacitat z configu
extern int InflateRadius; // TODO Nacitat z configu
extern std::atomic<bool> UpdateGridMap;
extern float HeightFilterAlpha;

// VFH vector magnitude m = (a - b * ObstacleDistnce) -> pri tomto nastaveni je magnitude v intervalu <0,1>
// pokud plati
// - m = 0 kdyz ObstacleDistnce > Dmax
// - m = 1 kdyz ObstacleDistnce < RobotRadius
// - m = (CoefA - CoefB * ObstacleDistnce)
#define RobotRadius 0.4f
#define Dmax ((GridSizeM / 2.0f) - 0.1f)
#define CoefA (Dmax / (Dmax-RobotRadius))
#define CoefB (CoefA / Dmax)

// Flat grid map: Cells - hlavni vystup s binarizovanymi a nafouknutymi prekazkam
typedef struct {
    bool unknown; // binarizovana neznama
    bool free; // binarizovana volna
    bool occupied; // binarizovana obsazena
    bool obstacle; // volna, ale nafouknutim oznacena jako prekazka
    bool path; // je to path?
} TCell;
extern TCell ObstacleGrid[GridCells][GridCells];

extern std::atomic<bool> ShutdownLocMap;
extern cv::Mat lmap_depth_image; // DEPRECATED - Input depth bitmap CV16C1 -> prevzit pointcloud z modulu vision

//extern TPoint2D BresenhamLimObstacle(int X1, int Y1, int X2, int Y2, int Min, int Max);
//extern bool BresenhamLimObstacle(int X1, int Y1, int X2, int Y2, int Min, int Max, TPoint2D &point);
extern void RunLocMap();
extern void ClearLocMap();
extern void CalcVFH();
extern void SetGoal(const int Xidx, const int Yidx);
extern void Plan();

} // end namespace

#endif
