#include <iostream>
#include "locmap.h"
#include "grid_map/include/grid_map_core/GridMap.hpp"

namespace locmap {

#define D455HStart 160

cv::Mat lmap_depth_image(D455H,D455W,CV_16UC1); // DEPRECATED - prevzit pointcloud z modulu vision

float ObstacleDelta = 0.27; // TODO Nacitat z configu
std::atomic<bool> UpdateGridMap(false);
//std::atomic<uint32_t> Vrtule(0);

std::atomic<bool> ShutdownLocMap(false);
std::unique_lock<std::mutex> LocMap_lock(LocMap_mutex, std::defer_lock);

cv::Mat mapgrid(GridCells,GridCells,CV_32FC1,cv::Scalar(-1e6));

typedef struct {
    float h;
    bool modified;
    bool free;
} TValCell;
TValCell vgrid[GridCells][GridCells];

// // Flat grid map: Cells - hlavni vystup s binarizovanymi a nafouknutymi prekazkam
// typedef struct {
//     bool unknown; // binarizovana neznama
//     bool free; // binarizovana volna
//     bool occupied; // binarizovana obsazena
//     bool obstacle; // volna, ale nafouknutim oznacena jako prekazka
//     bool path; // je to path?
// } TCell;
TCell ObstacleGrid[GridCells][GridCells];

grid_map::GridMap map;
grid_map::Position mappos;

// VFH ------------------------------------------------------------------------

// Predpokladam, ze horni stred gridmapy (GridCenter,0) je SEVER, uhel 0.0 (deg i rad).
// TODO Je potreba zajistit korektni inicializaci Yaw na SEVER
// T265 vraci +/- PI rad
// Provedu virualni scan oproti aktualnimu yaw z T265
// Scan je 270 deg, +/-135 deg, krok 0.2 deg = 1350 paprsku / binu
// float StartAngle = -2.35619449; // rad (-135 deg)
// float EndAngle = 2.35619449; // rad (135 deg)
// float ScanStep = 0.003490659; // rad (0.2 deg)
// int NumSteps = 1350;
#define MagnitudeMax sqrt((DepthCamMaxDistM*DepthCamMaxDistM)+(DepthCamMaxDistM*DepthCamMaxDistM))
std::vector<float> ScanDist;
std::vector<TPoint2D> ScanPts;

TPoint2D BresenhamLimObstacle(int X1, int Y1, int X2, int Y2, int Min, int Max) {
    TPoint2D point;
    
    long dx =  abs(X2-X1), sx = X1<X2 ? 1 : -1;
    long dy = -abs(Y2-Y1), sy = Y1<Y2 ? 1 : -1;
    long err = dx+dy, e2; // error value e_xy    
    for(;;) {
        if ((X1 < Min) || (X1 > Max) || (Y1 < Min) || (Y1 > Max)) {
            break;
        }
        else {
            if (ObstacleGrid[X1][Y1].obstacle) {
                point.x = X1; point.y = Y1;
                return point;
            }
            e2 = 2*err;
            if (e2 >= dy) { err += dy; X1 += sx; } // e_xy+e_x > 0
            if (e2 <= dx) { err += dx; Y1 += sy; } // e_xy+e_y < 0
        }
    }
    point.x = -X1; point.y = -Y1;
    return point;
}

// ----------------------------------------------------------------------------
void InitLocMap() {
    //mapgrid.setTo(cv::Scalar((2*GridHeightLimitBot)));
    map = grid_map::GridMap({"h","delta","obstacle"});
    map.setFrameId("map");
    map.setGeometry(grid_map::Length(GridSizeM,GridSizeM),GridResolutionM);

    map["h"].setConstant(NAN);
    map["delta"].setZero();
    map["obstacle"].setConstant(-1.0); //map["obstacle"].setZero();
}

// ============================================================================
void UpdateLocMap() {
    if (UpdateGridMap) {
        //Vrtule++;
        mapgrid.setTo(cv::Scalar((2*GridHeightLimitBot)));

        for(int y=0;y<GridCells;y++) {
            for(int x=0;x<GridCells;x++) {
                vgrid[x][y].h = -1e6;
                vgrid[x][y].modified = false;
                vgrid[x][y].free = false;
            }
        }

// Calc pointcloud
        for (int v = D455HStart; v < D455H; ++v) {
            for (int u = 110; u < D455W; ++u) { // WARNING Je potreba vynechat "mrtvy" svisly pas depth dat vlevo !!!
                uint16_t depth_value_mm = lmap_depth_image.at<uint16_t>(v, u);

                if (depth_value_mm == 0) continue;

                float z_cam = static_cast<float>(depth_value_mm) * DepthCamScale;
                float x_cam = (static_cast<float>(u) - D455_depth_Ppx) / D455_depth_Fx;
                float y_cam = (static_cast<float>(v) - D455_depth_Ppy) / D455_depth_Fy;
                x_cam *= z_cam;
                y_cam *= z_cam;

// Rotace bodu z pointcloudu
                glm::vec3 Q;
                Q.x = x_cam;
                Q.y = -y_cam;
                Q.z = -z_cam;

                glm::vec3 V;
                V = BotOrientation * Q;

                int grid_y_idx = static_cast<int>(floor(-V.z / GridResolutionM + GridCenter));
                grid_y_idx = GridCells - grid_y_idx;
                int grid_x_idx = static_cast<int>(floor(V.x / GridResolutionM + GridCenter));

                if (grid_x_idx >= 0 && grid_x_idx < GridCells && grid_y_idx >= 0 && grid_y_idx < GridCells) {
                    //float height_value = -y_cam;
                    float height_value = V.y;

                    if (height_value > GridHeightLimitTop) {
                        continue;
                    }

                    float& current_max = mapgrid.at<float>(grid_y_idx, grid_x_idx);
                    if (height_value > current_max) {
                        current_max = height_value;
                    }
                }
            }
        }
// Z mapgrid vynechej hodnoty mensi nez GridHeightLimitBot a od ostatnich odecti GridHeightLimitBot a vloz je do vgrid
        for (int v = 0; v < GridCells; ++v) {
            for (int u = 0; u < GridCells; ++u) {
                float gridval = mapgrid.at<float>(u,v);
                if (gridval < GridHeightLimitBot) {
                }
                else {
                    gridval += -GridHeightLimitBot;
                    mapgrid.at<float>(u,v) = gridval;
                    vgrid[u][v].h = gridval;
                    vgrid[u][v].modified = true;
                }
            }
        }
// Pro vsechny hodnoty ve vgrid proved raycast ze stredu. Pokud ray narazi na modified, nastavi vsechny cells mezi stredem a timto bodem na free -> vznikne "laser scan"
// vgrid[x][y].free se vyuzije k promazani binarizovanych prekazek, ktere jsou mensi nez GridHeightLimitBot
        for (int v = 0; v < GridCells; ++v) {
            for (int u = 0; u < GridCells; ++u) {
                if (vgrid[u][v].modified) {
                    std::vector<TPoint2D> bpts;
                    BresenhamLim2(GridCenter,GridCenter,u,v,0,GridCells-1,bpts);
                    int j=0;
                    int cx, cy;
                    for (j=0;j<bpts.size();j++) {
                        cx = bpts[j].x; cy = bpts[j].y;
                        if (vgrid[cx][cy].modified) {
                            for (int i=0;i<j;i++) {
                                cx = bpts[i].x; cy = bpts[i].y;
                                vgrid[cx][cy].free = true;

                            }
                            vgrid[u][v].free = false; // Pro jistotu
                        }
                    }
                }
            }
        }

// Update ETH grid_map
        glm::vec3 posV;
        grid_map::Index Idx;
        grid_map::Position position;
        glm::vec3 V;
        glm::vec3 sky = {0.0,1.0,0.0};
// WARNING TODO BotOrientation, BotPos - nejsou thread safe
        posV = BotOrientation * posV;
        posV = posV + BotPos;
        mappos = grid_map::Position(-BotPos.z,-BotPos.x);
        map.move(mappos);
        auto& hLayer = map["h"];
        auto& deltaLayer = map["delta"];
        auto& obstacleLayer = map["obstacle"];

        grid_map::Index index;
        grid_map::Index startindex;

// Aktualizovat vysky v "map" hodnotami z "vgrid", ktere jsou "modified"
        startindex = map.getStartIndex();
        index = startindex;
        for(int y=0;y<GridCells;y++) {
            index(0) = startindex(0);
            for(int x=0;x<GridCells;x++) {
                if (vgrid[x][y].modified) {
                    hLayer(index(0), index(1)) = vgrid[x][y].h;
                }
                index(0) = index(0)+1; if (index(0) >= GridCells) index(0) = 0;
            }
            index(1) = index(1)+1; if (index(1) >= GridCells) index(1) = 0;
        }
// Calculate normals
        // Budeme iterovat vnitrkem mapy bez okraje v sirce 1 cell, protoze pro vypocet normal potrebujeme "kriz" +/- 1 cell
        int32_t X,Y,L,R,U,D;
        startindex = map.getStartIndex();
        // startindex(0) = startindex(0)+1; if (startindex(0) >= GridCells-1) startindex(0) = 0;
        // startindex(1) = startindex(1)+1; if (startindex(1) >= GridCells-1) startindex(1) = 0;
        startindex(0) = startindex(0)+1; if (startindex(0) >= GridCells) startindex(0) = 0;
        startindex(1) = startindex(1)+1; if (startindex(1) >= GridCells) startindex(1) = 0;
        index = startindex;
        for(int y=1;y<GridCells-1;y++) {
            index(0) = startindex(0);
            for(int x=1;x<GridCells-1;x++) {

                X = index(0); Y = index(1);
                U = index(1)-1; if (U < 0) U = GridCells-1;
                D = index(1)+1; if (D >= GridCells-1) D = 0;
                L = index(0)-1; if (L < 0) L = GridCells-1;
                R = index(0)+1; if (R >= GridCells-1) R = 0;
                auto& eR = hLayer(R,Y);
                auto& eL = hLayer(L,Y);
                auto& eU = hLayer(X,U);
                auto& eD = hLayer(X,D);
                V.x = -(eL - eR);
                V.z = -(eD - eU);
                V.y = 1.0;
                V = glm::normalize(V);
                auto& delta = deltaLayer(X,Y);
                auto& obstacle = obstacleLayer(X,Y);
// Pro modifikovane cells spocitej normaly a vyhodnot odchylku od oblohy.
// Pokud je odchylka vetsi nez ObstacleDelta, je to prilis kolme a je to prekazka
                if (vgrid[x][y].modified) {
                    float Q = std::acos(glm::dot(V,sky));
                    obstacle = -1.0; // Unknown
                    if (!std::isnan(Q)) {
                        if (Q > 1.0) Q = 1.0;
                        delta = Q;
                        if (Q > ObstacleDelta) {
                            obstacle = 1.0; // Obstacle
                        }
                        else {
                            obstacle = 0.0; // Free
                        }
                    }
                    else {
                        delta = 0.0;
                        obstacle = -1.0; // Unknown
                    }
                }
// Pokud je prislusna cell ve vgrid volna, vymaz pripadnou prekazku vypoctenou v predchozim kroku (proc?) - viz "laser scan" vyse
                if (vgrid[x][y].free) {
                    obstacle = 0.0; // Free
                }
                index(0) = index(0)+1; if (index(0) >= GridCells) index(0) = 0;
            }
            index(1) = index(1)+1; if (index(1) >= GridCells) index(1) = 0;
        }
// Get flat Grid do matice ObstacleGrid pro nafukovani prekazek a planovani cesty
        startindex = map.getStartIndex();
        index = startindex;
        for(int y=0;y<GridCells;y++) {
            //map.getPosition(index,position);
            index(0) = startindex(0);
            for(int x=0;x<GridCells;x++) {
                ObstacleGrid[x][y].obstacle = false; // Cell.obstacle se aktualizuje v Inflate()
                ObstacleGrid[x][y].path = false;
                float& obstacle = obstacleLayer(index(0), index(1));
                //float& pathitem = pathLayer(index(0), index(1));
                if (obstacle > 0.0) {
                    // Obstacle
                    ObstacleGrid[x][y].unknown = false;
                    ObstacleGrid[x][y].free = false;
                    ObstacleGrid[x][y].occupied = true;
                }
                else if (obstacle < 0.0) {
                    // Unknown
                    ObstacleGrid[x][y].unknown = true;
                    ObstacleGrid[x][y].free = false;
                    ObstacleGrid[x][y].occupied = false;
                }
                else {
                    // Free
                    ObstacleGrid[x][y].unknown = false;
                    ObstacleGrid[x][y].free = true;
                    ObstacleGrid[x][y].occupied = false;
                }
                index(0) = index(0)+1; if (index(0) >= GridCells) index(0) = 0;
            }
            index(1) = index(1)+1; if (index(1) >= GridCells) index(1) = 0;
        }
// Inflate
        int InflateRadius = 2;  // TODO Nacitat z configu // 5; // Number of grid cells. Radius in meters = InflateRadius * LocMapResM
        int r = InflateRadius;
        int nmin = r;
        int nmax = GridCells-r;

        for (int cy=nmin; cy<nmax; cy++) {
            for (int cx=nmin; cx<nmax; cx++) {
                TCell C = ObstacleGrid[cx][cy];
                if (C.occupied) {
                    int x = 0, y = r;
                    int d = 1 - r;
                    while (y >= x) {
                        for (int i = cx - x; i <= cx + x; i++) {
                            ObstacleGrid[i][cy + y].obstacle = true;
                            ObstacleGrid[i][cy - y].obstacle = true;
                        }
                        for (int i = cx - y; i <= cx + y; i++) {
                            ObstacleGrid[i][cy + x].obstacle = true;
                            ObstacleGrid[i][cy - x].obstacle = true;
                        }
                        if (d < 0) {
                            d += 2 * x + 3;
                        } else {
                            d += 2 * (x - y) + 5;
                            y--;
                        }
                        x++;
                    }
                }
            }
        }
        UpdateGridMap = false;
    }
}

// ----------------------------------------------------------------------------
void RunLocMap() {
    std::cout << "LocMap thread started." << std::endl;
    InitLocMap();
// ----------------------------------------------------------------------------    
    while (!ShutdownLocMap) {
        if (LocMap_lock.try_lock()) {
            UpdateLocMap();
            LocMap_lock.unlock();
        }
        usleep(100); // TODO test        
    }
    std::cout << "LocMap thread ended." << std::endl;
}

} // end namespace
