#include <iostream>
#include <stack>

#include "locmap.h"
#include "grid_map/include/grid_map_core/GridMap.hpp"

namespace locmap {

#define D455HStart 160

cv::Mat lmap_depth_image(D455H,D455W,CV_16UC1); // DEPRECATED - prevzit pointcloud z modulu vision

float ObstacleDelta = 0.27; // TODO Nacitat z configu
int InflateRadius = 2;  // TODO Nacitat z configu // 5; // Number of grid cells. Radius in meters = InflateRadius * LocMapResM

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

// Planner
std::vector<TPoint2DInt> Path;
std::vector<TPoint2DInt> AStarPath;
bool GoalValid = false;
glm::vec2 GoalPos;
TPoint2DInt GoalCellIdx;
TPoint2DInt PlanGoalIdx;

//void AStar(const TPoint2DInt& src, const TPoint2DInt& dest);
// bool isCellFree(int X, int Y);
// bool isValid(const std::pair<int, int>& point);
// bool isDestination(const std::pair<int, int>& position, const std::pair<int, int>& dest);
// double calculateHValue(const std::pair<int, int>& src, const std::pair<int, int>& dest);

// ============================================================================
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
                else if ((obstacle < 0.0) || std::isnan(obstacle)) {
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
//         int InflateRadius = 2;  // TODO Nacitat z configu // 5; // Number of grid cells. Radius in meters = InflateRadius * LocMapResM
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

// ============================================================================
struct planner_cell {
        // Row and Column index of its parent
        std::pair<int, int> parent;
        // f = g + h
        double f, g, h;
        planner_cell()
        : parent ( -1, -1 )
        , f ( -1 )
        , g ( -1 )
        , h ( -1 ) {
        }
    };

bool isCellFree(int X, int Y) {
    if (X < 0) X = 0;
    if (Y < 0) Y = 0;
    if (X > GridCells-1) X = GridCells-1;
    if (Y > GridCells-1) Y = GridCells-1;

    return (ObstacleGrid[X][Y].obstacle == false) && (ObstacleGrid[X][Y].free == true);
}

bool isValid(const std::pair<int, int>& point) {
    return ((point.first >= 0) && (point.first < GridCells) && (point.second >= 0) && (point.second < GridCells));
}

bool isDestination(const std::pair<int, int>& position, const std::pair<int, int>& dest) {
    return position == dest;
}

double calculateHValue(const std::pair<int, int>& src, const std::pair<int, int>& dest) {
    // h is estimated with the two points distance formula
    return sqrt(pow((src.first - dest.first),2.0) + pow((src.second - dest.second),2.0));
}

void AStar(const TPoint2DInt& src, const TPoint2DInt& dest) {
    AStarPath.clear();
    // Create a closed list and initialise it to false which
    // means that no cell has been included yet This closed
    // list is implemented as a boolean 2D array
    bool closedList[GridCells][GridCells];
    memset(closedList, false, sizeof(closedList));

    // Declare a 2D array of structure to hold the details
    // of that cell
    std::array<std::array<planner_cell,GridCells>,GridCells> cellDetails;

    int i, j;
    // Initialising the parameters of the starting node
    i = src.x, j = src.y;
    cellDetails[i][j].f = 0.0;
    cellDetails[i][j].g = 0.0;
    cellDetails[i][j].h = 0.0;
    cellDetails[i][j].parent = { i, j };

//          C reate an open list having information as <f, <i, j>>
//          where f = g + h, and i, j are the row and column index of that cell
//          Note that 0 <= i <= ROW-1 & 0 <= j <= COL-1
//          This open list is implenented as a set of tuple.
    std::priority_queue<std::tuple<double, int, int>, std::vector<std::tuple<double, int, int> >, std::greater<std::tuple<double, int, int>>> openList;
    // Put the starting cell on the open list and set its 'f' as 0
    openList.emplace(0.0, i, j);

    std::pair<int, int> dp(dest.x,dest.y);
    //std::cout << "DP: " << dp.first << "," << dp.second << std::endl;

    // We set this boolean value as false as initially
    // the destination is not reached.
    bool AStarPathFound = false;
    while (!openList.empty()) {
        const std::tuple<double, int, int>& p = openList.top();
        // Add this vertex to the closed list
        i = std::get<1>(p); // second element of tuple
        j = std::get<2>(p); // third element of tuple

        // Remove this vertex from the open list
        openList.pop();
        closedList[i][j] = true;
        /*
            *   		Generating all the 8 successor of this cell
            *   				N.W N N.E
            *   				\ | /
            *   				\ | /
            *   				W----Cell----E
            *   						/ | \
            *   				/ | \
            *   				S.W S S.E
            *
            *   		Cell-->Popped Cell (i, j)
            *   		N --> North	 (i-1, j)
            *   		S --> South	 (i+1, j)
            *   		E --> East	 (i, j+1)
            *   		W --> West		 (i, j-1)
            *   		N.E--> North-East (i-1, j+1)
            *   		N.W--> North-West (i-1, j-1)
            *   		S.E--> South-East (i+1, j+1)
            *   		S.W--> South-West (i+1, j-1)
            */
        for (int add_x = -1; add_x <= 1; add_x++) {
            for (int add_y = -1; add_y <= 1; add_y++) {
                std::pair<int, int> neighbour ( i + add_x, j + add_y );
                // Only process this cell if this is a valid one
                if (isValid(neighbour)) {
                    // If the destination cell is the same as the current successor
                    if (isDestination(neighbour,dp)) { // Set the Parent of the destination cell
                        cellDetails[neighbour.first][neighbour.second].parent = { i, j };
                        AStarPathFound = true;
                        break;
                    }
                    // If the successor is already on the closed list or if it is blocked, then ignore it. Else do the following
                    else if (!closedList[neighbour.first][neighbour.second] && isCellFree(neighbour.first,neighbour.second)) {
                        double gNew, hNew, fNew;
                        gNew = cellDetails[i][j].g + 1.0;
                        hNew = calculateHValue ( neighbour,dp );
                        fNew = gNew + hNew;

                        // If it isn’t on the open list, add it to the open list. Make the current square the parent of this square.
                        // Record the f, g, and h costs of the square cell
                        // OR
                        // If it is on the open list already, check to see if this path to that square is better, using 'f' cost as the measure.
                        if (cellDetails[neighbour.first][neighbour.second].f == -1 || cellDetails[neighbour.first][neighbour.second].f > fNew) {
                            openList.emplace(fNew, neighbour.first, neighbour.second);

                            // Update the details of this cell
                            cellDetails[neighbour.first][neighbour.second].g = gNew;
                            cellDetails[neighbour.first][neighbour.second].h = hNew;
                            cellDetails[neighbour.first][neighbour.second].f = fNew;
                            cellDetails[neighbour.first][neighbour.second].parent = { i, j };
                        }
                    }
                }
            }
            if (AStarPathFound) {
                break;
            }
        }
        if (AStarPathFound) {
            break;
        }
    }

    if (AStarPathFound) {
        // Backtrack path
        std::stack<std::pair<int, int>> iAStarPath;
        int row = dp.first; // y
        int col = dp.second; // x

        std::pair<int, int> next_node = cellDetails[row][col].parent;
        if ((next_node.first >= 0) && (next_node.second >= 0)) {
            do {
                iAStarPath.push(next_node);
                next_node = cellDetails[row][col].parent;
                row = next_node.first;
                col = next_node.second;
            } while (cellDetails[row][col].parent != next_node);

            TPoint2DInt p;
            p.x = GridCenter; p.y = GridCenter;
            //AStarPath.push_back(p); -- Nebudeme vkladat CenterCell
            while (!iAStarPath.empty()) {
                std::pair<int, int> pp = iAStarPath.top();
                iAStarPath.pop();
                p.x = pp.first; p.y = pp.second;
                AStarPath.push_back(p);
            }
            p.x = dp.first; p.y = dp.second;
            AStarPath.push_back(p);
        }
    }
}


// #define GridResolutionM 0.08f
// #define GridSizeM 8.08f
// #define GridCells 101 //(GridSizeM / GridResolutionM)
// #define GridCenter 51

// ----------------------------------------------------------------------------
void ClearPath() {
    for(int y=0;y<GridCells;y++) {
        for(int x=0;x<GridCells;x++) {
            ObstacleGrid[y][x].path = false;
        }
    }
}

void SetGoal(const int Xidx, const int Yidx) {
    GoalCellIdx.x = Xidx;
    GoalCellIdx.y = Yidx;
}

int GoalSearchAttempts = ((GridCells / 2) - 1);

void Plan() {
    TPoint2DInt startpt;
    std::vector<TPoint2DInt> pts;
    int radius,MinIdx;
    bool found;
    float MinDist,D;

    startpt.x = GridCenter; startpt.y = GridCenter;
    PlanGoalIdx = GoalCellIdx; // TODO bude vyhledani nejblizhi free cell - viz CircleLim nize
#if 1
    radius = 2;
    for (int attempts=0; attempts<GoalSearchAttempts; attempts++) {
        if (!ObstacleGrid[GoalCellIdx.x][GoalCellIdx.y].free) {
            found = false;
            PlanGoalIdx = GoalCellIdx;
            for (int i=0;i<50;i++) {
                pts.clear();
                CircleLim(PlanGoalIdx.x,PlanGoalIdx.y,radius,0,GridCells,0,GridCells,pts);
                MinDist = 1e9; MinIdx = 100000;
                for (int i=0;i<pts.size();i++) {
                    if ((ObstacleGrid[pts[i].x][pts[i].y].free) || (ObstacleGrid[pts[i].x][pts[i].y].unknown)) {
                        D = Distance2D(startpt.x,startpt.y,pts[i].x,pts[i].y);
                        if (D < MinDist) {
                            MinIdx = i;
                            MinDist = D;
                        }
                    }
                }
                if (MinIdx < 100000) {
                    PlanGoalIdx = pts[MinIdx];
                    break;
                }
                else
                    radius++;
            }
        }
        AStar(startpt,PlanGoalIdx);
        if (AStarPath.size() > 0) {
            break;
        }
        else {
            radius++;
        }
    }
#else
    AStar(startpt,PlanGoalIdx);
#endif

    if (AStarPath.size() > 0) {
        ClearPath();
        Path = AStarPath;
        for (int i=0;i<Path.size();i++) {
            ObstacleGrid[Path[i].x][Path[i].y].path = true;
        }
    }
}


} // end namespace
