#include <dynamicEDT3D/dynamicEDT3D.h>
#include <dynamicEDT3D/dynamicEDTOctomap.h>
#include <iostream>
#include <stdlib.h>

#include <boost/shared_ptr.hpp>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/objectives/StateCostIntegralObjective.h>
#include <ompl/base/objectives/MaximizeMinClearanceObjective.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/base/PlannerTerminationCondition.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>

#include <fstream>
#include <dynamicEDT3D/dynamicEDT3D.h>
#include <dynamicEDT3D/dynamicEDTOctomap.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Eigenvalues>
#include <eigen3/Eigen/Geometry>

#include <visualization_msgs/MarkerArray.h>
#include <ros/ros.h>
#include <ros/node_handle.h>


#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#include <octomap_ros/conversions.h>
#include <octomap_msgs/conversions.h>

using namespace Eigen;
using namespace std;

Eigen::MatrixXd EDT_Matrix;

template<typename to, typename from>
to lexical_cast(from const &x)
{
    std::stringstream os;
    to ret;
    os << x;
    os >> ret;
    return ret;
}


int createEDT( int argc , char** argv) {

    //we build a sample map
    int sizeX, sizeY, sizeZ;
    sizeX=100;
    sizeY=100;
    sizeZ=100;

    bool*** map;
    map = new bool**[sizeX];
    for(int x=0; x<sizeX; x++)
    {
        map[x] = new bool*[sizeY];
        for(int y=0; y<sizeY; y++)
        {
            map[x][y] = new bool[sizeZ];
            for(int z=0; z<sizeZ; z++)
            {
                if(x<2 || x > sizeX-3 || y < 2 || y > sizeY-3 || z<2 || z > sizeZ-3)
                        map[x][y][z] = 1;
                else
                    map[x][y][z] = 0;
            }
        }
    }

    map[51][45][67] = 1;
    map[50][50][68] = 1;

    // create the EDT object and initialize it with the map
    int maxDistInCells = 20;
    DynamicEDT3D distmap(maxDistInCells*maxDistInCells);
    distmap.initializeMap(sizeX, sizeY, sizeZ, map);



    //compute the distance map
    distmap.update();

    // now perform some updates with random obstacles
    int numPoints = 20;
    for (int frame=1; frame<=10; frame++)
    {
        std::cout<<"\n\nthis is frame #"<<frame<<std::endl;
        std::vector<IntPoint3D> newObstacles;
        for (int i=0; i<numPoints; i++)
        {
            double x = 2+rand()/(double)RAND_MAX*(sizeX-4);
            double y = 2+rand()/(double)RAND_MAX*(sizeY-4);
            double z = 2+rand()/(double)RAND_MAX*(sizeZ-4);
            newObstacles.push_back(IntPoint3D(x,y,z));
        }

        // register the new obstacles (old ones will be removed)
        distmap.exchangeObstacles(newObstacles);

        //update the distance map
        distmap.update();

        //retrieve distance at a point
        float dist = distmap.getDistance(30,67,33);
        int distSquared = distmap.getSQCellDistance(30,67,33);
        std::cout<<"distance at  30,67,33: "<< dist << " squared: "<< distSquared << std::endl;
        if(distSquared == maxDistInCells*maxDistInCells)
            std::cout<<"we hit a cell with d = dmax, distance value is clamped."<<std::endl;

        //retrieve closest occupied cell at a point
        IntPoint3D closest = distmap.getClosestObstacle(30,67,33);
        if(closest.x == DynamicEDT3D::invalidObstData)
            std::cout<<"we hit a cell with d = dmax, no information about closest occupied cell."<<std::endl;
        else
        std::cout<<"closest occupied cell to 30,67,33: "<< closest.x<<","<<closest.y<<","<<closest.z<<std::endl;

    }


  std::cout<<"\n\nthis is the last frame"<<std::endl;

  // now remove all random obstacles again.
  std::vector<IntPoint3D> empty;
  distmap.exchangeObstacles(empty);
  distmap.update();


  //retrieve distance at a point
  float dist = distmap.getDistance(30,67,33);
  int distSquared = distmap.getSQCellDistance(30,67,33);
  std::cout<<"distance at  30,67,33: "<< dist << " squared: "<< distSquared << std::endl;
  if(distSquared == maxDistInCells*maxDistInCells)
  	std::cout<<"we hit a cell with d = dmax, distance value is clamped."<<std::endl;

  //retrieve closest occupied cell at a point
  IntPoint3D closest = distmap.getClosestObstacle(30,67,33);
  if(closest.x == DynamicEDT3D::invalidObstData)
  	std::cout<<"we hit a cell with d = dmax, no information about closest occupied cell."<<std::endl;
  else
  	std::cout<<"closest occupied cell to 30,67,33: "<< closest.x<<","<<closest.y<<","<<closest.z<<std::endl;

  return 0;
}


DynamicEDTOctomap createEDTfromOctomaptree(octomap::OcTree *tree )
{
//    if(argc<=1)
//    {
//        std::cout<<"usage: "<<argv[0]<<" <octoMap.bt>"<<std::endl;
//        exit(0);
//    }

//    octomap::OcTree *tree = NULL;
//    tree = new octomap::OcTree(0.01);

//    //read in octotree
//    tree->readBinary(argv[1]);

    std::cout<<"read in tree, "<<tree->getNumLeafNodes()<<" leaves "<<std::endl;

    double x,y,z;
    tree->getMetricMin(x,y,z);
    octomap::point3d min(x,y,z);
    std::cout<<"Metric min: "<<x<<","<<y<<","<<z<<std::endl;
    tree->getMetricMax(x,y,z);
    octomap::point3d max(x,y,z);
    std::cout<<"Metric max: "<<x<<","<<y<<","<<z<<std::endl;

    bool unknownAsOccupied = true;
    unknownAsOccupied = false;
    float maxDist = 1.0;
    //- the first argument ist the max distance at which distance computations are clamped
    //- the second argument is the octomap
    //- arguments 3 and 4 can be used to restrict the distance map to a subarea
    //- argument 5 defines whether unknown space is treated as occupied or free
    //The constructor copies data but does not yet compute the distance map
    DynamicEDTOctomap distmap(maxDist, tree, min, max, unknownAsOccupied);

    //This computes the distance map
    distmap.update();

    //This is how you can query the map
    octomap::point3d p(0.0,0.0,0.0);
    //As we don't know what the dimension of the loaded map are, we modify this point
//    p.x() = min.x() + 0.3 * (max.x() - min.x());
//    p.y() = min.y() + 0.6 * (max.y() - min.y());
//    p.z() = min.z() + 0.5 * (max.z() - min.z());

    octomap::point3d closestObst;
    float distance;

    distmap.getDistanceAndClosestObstacle(p, distance, closestObst);

    std::cout<<"\n\ndistance at point "<<p.x()<<","<<p.y()<<","<<p.z()<<" is "<<distance<<std::endl;
    if(distance < distmap.getMaxDist())
    std::cout<<"closest obstacle to "<<p.x()<<","<<p.y()<<","<<p.z()<<" is at "<<closestObst.x()<<","<<closestObst.y()<<","<<closestObst.z()<<std::endl;

    //if you modify the octree via tree->insertScan() or tree->updateNode()
    //just call distmap.update() again to adapt the distance map to the changes made

    delete tree;

    return distmap;
}


void twodMapExample()
{
    //we build a sample map
    int sizeX, sizeY, sizeZ;
    sizeX=100;
    sizeY=100;
    sizeZ=100;

    bool*** map;
    map = new bool**[sizeX];
    for(int x=0; x<sizeX; x++)
    {
        map[x] = new bool*[sizeY];
        for(int y=0; y<sizeY; y++)
        {
            map[x][y] = new bool[sizeZ];
            for(int z=0; z<sizeZ; z++)
            {
                map[x][y][z] = 0;
            }
        }
    }

/*

0,0 ........................ 0,65     0,75 .......0,99
.
.
.
.
.
.
.
.... 40,35    40,45.........
.
.
.                            60,65     60,75 ..... 60,99
.
.
.
.
.
.
.
.
... 99,35     99,45................................99,99




*/


    EDT_Matrix.resize(100,100);
    EDT_Matrix=MatrixXd::Zero(100, 100);
    for(int x=0; x<60; x++)
    {

        for(int y=65; y<75; y++)
        {

            for(int z=0; z<1; z++)
            {
                map[x][y][z] = 1;
                EDT_Matrix(x,y)=1;
            }
        }
    }



    for(int x=40; x<99; x++)
    {

        for(int y=35; y<45; y++)
        {

            for(int z=0; z<1; z++)
            {
                map[x][y][z] = 1;
                EDT_Matrix(x,y)=1;
            }
        }
    }





    // create the EDT object and initialize it with the map
    int maxDistInCells = 100;
    DynamicEDT3D distmap(maxDistInCells*maxDistInCells);
    distmap.initializeMap(sizeX, sizeY, sizeZ, map);
    distmap.update();





    for(int x=0; x<sizeX; x++)
    {
        for(int y=0; y<sizeY; y++)
        {
            for(int z=0; z<1; z++)
            {
                std::cout<<distmap.getDistance(x,y,z);

                if(sizeY!=y+1)
                    std::cout<<" , ";


            }
        }
        std::cout<<"\n";
    }


}


void AddOctomapForOptmisationPlanning(boost::shared_ptr<DynamicEDTOctomap> &distmap, octomap::OcTree  &tree, bool unknownAsOccupied, float maxDist )
{
    double x,y,z;
    tree.getMetricMin(x,y,z);
    octomap::point3d min(x,y,z);
//    std::cout<<"Metric min: "<<x<<","<<y<<","<<z<<std::endl;
    tree.getMetricMax(x,y,z);
    octomap::point3d max(x,y,z);
//    std::cout<<"Metric max: "<<x<<","<<y<<","<<z<<std::endl;
    distmap.reset(new DynamicEDTOctomap(maxDist,&tree,min,max,unknownAsOccupied));
    distmap->update();
}




namespace ob = ompl::base;
namespace og = ompl::geometric;


void populate_EDT_Matrix()
{
    EDT_Matrix.resize(100,100);
    std::string line;
    std::string delimiter = ",";
    std::ifstream myfile ("/home/behnam/sample_based_optimisation_based_path_planner_sw/devel/lib/sample_based_optimisation_based_path_planner/map.txt");
    int i,j;
    i=0;
    j=0;
    if (myfile.is_open())
    {
        while ( getline (myfile,line) )
        {

            size_t pos = 0;
            std::string token;
            j=0;
            while ((pos = line.find(delimiter)) != std::string::npos)
            {

                token = line.substr(0, pos);
                std::cout << i<<"," <<j << std::endl;

                EDT_Matrix(i,j)=   lexical_cast<double>(token);
//                j++;

                line.erase(0, pos + delimiter.length());
            }
            std::cout << i<<"," <<j << std::endl;
            EDT_Matrix(i,j)=   lexical_cast<double>(line);
//            i++;
        }
        myfile.close();
    }

    else
    {
        std::cout << "Unable to open file...";
    }

    std::cout<< EDT_Matrix<<std::endl;
}


#include <SDL/SDL.h>
#include <math.h>



#define WIDTH  100
#define HEIGHT 100

struct Point
{
    int dx, dy;

    int DistSq() const { return dx*dx + dy*dy; }
};

struct Grid
{
    Point grid[HEIGHT][WIDTH];
};

Point inside = { 0, 0 };
Point empty = { 9999, 9999 };
Grid grid1, grid2;

Point Get( Grid &g, int x, int y )
{
    // OPTIMIZATION: you can skip the edge check code if you make your grid
    // have a 1-pixel gutter.
    if ( x >= 0 && y >= 0 && x < WIDTH && y < HEIGHT )
        return g.grid[y][x];
    else
        return empty;
}

void Put( Grid &g, int x, int y, const Point &p )
{
    g.grid[y][x] = p;
}

void Compare( Grid &g, Point &p, int x, int y, int offsetx, int offsety )
{
    Point other = Get( g, x+offsetx, y+offsety );
    other.dx += offsetx;
    other.dy += offsety;

    if (other.DistSq() < p.DistSq())
        p = other;
}

void GenerateSDF( Grid &g )
{
    // Pass 0
    for (int y=0;y<HEIGHT;y++)
    {
        for (int x=0;x<WIDTH;x++)
        {
            Point p = Get( g, x, y );
            Compare( g, p, x, y, -1,  0 );
            Compare( g, p, x, y,  0, -1 );
            Compare( g, p, x, y, -1, -1 );
            Compare( g, p, x, y,  1, -1 );
            Put( g, x, y, p );
        }

        for (int x=WIDTH-1;x>=0;x--)
        {
            Point p = Get( g, x, y );
            Compare( g, p, x, y, 1, 0 );
            Put( g, x, y, p );
        }
    }

    // Pass 1
    for (int y=HEIGHT-1;y>=0;y--)
    {
        for (int x=WIDTH-1;x>=0;x--)
        {
            Point p = Get( g, x, y );
            Compare( g, p, x, y,  1,  0 );
            Compare( g, p, x, y,  0,  1 );
            Compare( g, p, x, y, -1,  1 );
            Compare( g, p, x, y,  1,  1 );
            Put( g, x, y, p );
        }

        for (int x=0;x<WIDTH;x++)
        {
            Point p = Get( g, x, y );
            Compare( g, p, x, y, -1, 0 );
            Put( g, x, y, p );
        }
    }
}

int Signed_Distance_Fields_test(SDL_Surface *temp   )
{
    if ( SDL_Init( SDL_INIT_VIDEO ) == -1 )
        return 1;

    SDL_Surface *screen = SDL_SetVideoMode( WIDTH, HEIGHT, 32, SDL_SWSURFACE );
    if ( !screen )
        return 1;



    temp = SDL_ConvertSurface( temp, screen->format, SDL_SWSURFACE );
    SDL_LockSurface( temp );
    for( int y=0;y<HEIGHT;y++ )
    {
        for ( int x=0;x<WIDTH;x++ )
        {
            Uint8 r,g,b;
            Uint32 *src = ( (Uint32 *)( (Uint8 *)temp->pixels + y*temp->pitch ) ) + x;
            SDL_GetRGB( *src, temp->format, &r, &g, &b );

            // Points inside get marked with a dx/dy of zero.
            // Points outside get marked with an infinitely large distance.
            if ( g < 128 )
            {
                Put( grid1, x, y, inside );
                Put( grid2, x, y, empty );
            } else {
                Put( grid2, x, y, inside );
                Put( grid1, x, y, empty );
            }
        }
    }
    SDL_UnlockSurface( temp );

    // Generate the SDF.
    GenerateSDF( grid1 );
    GenerateSDF( grid2 );

    // Render out the results.
    SDL_LockSurface( screen );
    for( int y=0;y<HEIGHT;y++ )
    {
        for ( int x=0;x<WIDTH;x++ )
        {
            // Calculate the actual distance from the dx/dy
            int dist1 = (int)( sqrt( (double)Get( grid1, x, y ).DistSq() ) );
            int dist2 = (int)( sqrt( (double)Get( grid2, x, y ).DistSq() ) );
            int dist = dist1 - dist2;
            std::cout<<dist;

            if(x!=WIDTH-1)
            {
                std::cout<<" , ";
            }

            // Clamp and scale it, just for display purposes.
            int c = dist*3 + 128;
            if ( c < 0 ) c = 0;
            if ( c > 255 ) c = 255;

            Uint32 *dest = ( (Uint32 *)( (Uint8 *)screen->pixels + y*screen->pitch ) ) + x;
            *dest = SDL_MapRGB( screen->format, c, c, c );
        }
        std::cout<<"\n";
    }
    SDL_UnlockSurface( screen );
    SDL_Flip( screen );
    SDL_SaveBMP(screen,"edt.bmp");

    // Wait for a keypress
    SDL_Event event;
    while( true )
    {
        if ( SDL_PollEvent( &event ) )
        switch( event.type )
        {
        case SDL_QUIT:
        case SDL_KEYDOWN:
            return true;
        }
    }

    return 0;
}



////////////////////////////////////////////////////////////////////////////////////////////////////



//#define LOG_CLEARANCE
//#define LOG_STATECOST
/**/
class ValidityChecker : public ob::StateValidityChecker
{
public: ValidityChecker(const ob::SpaceInformationPtr& si) : ob::StateValidityChecker(si) {}

    bool isValid(const ob::State* state) const
    {
        const ob::RealVectorStateSpace::StateType* state2D =state->as<ob::RealVectorStateSpace::StateType>();

        int x= nearbyint(state2D->values[0]) ;
        int y = nearbyint( state2D->values[1]);
        int M,N;

        M=y;
        N=x;

        return EDT_Matrix(M,N) > 0.0;
    }

    double clearance(const ob::State* state) const
    {
        const ob::RealVectorStateSpace::StateType* state2D =state->as<ob::RealVectorStateSpace::StateType>();

        int x= nearbyint(state2D->values[0]) ;
        int y = nearbyint( state2D->values[1]);

        int M,N;

        M=y;
        N=x;

        #ifdef LOG_CLEARANCE
        std::cout<<"****************** computing clearance (distance from closest obstacle, using edt matrix) *******************"<<std::endl;

        std::cout<<"x,y: "<<x  <<" , " <<y  <<std::endl;
        std::cout<<"position of the point in the Matrix, M:N "<<M  <<", " <<N  <<std::endl;
        std::cout<<"Distance to the closest obstacle(extracted from edt matrix): "<<EDT_Matrix(M,N)   <<std::endl;

        std::cout<<"********************************************"<<std::endl;
        #endif


        return EDT_Matrix(M,N);


    }
};

ob::OptimizationObjectivePtr getPathLengthObjective(const ob::SpaceInformationPtr& si)
{
    std::cout<<"getPathLengthObjective called"<<std::endl;

    return ob::OptimizationObjectivePtr(new ob::PathLengthOptimizationObjective(si));
}

ob::OptimizationObjectivePtr getThresholdPathLengthObj(const ob::SpaceInformationPtr& si)
{
    std::cout<<"getThresholdPathLengthObj called"<<std::endl;
    ob::OptimizationObjectivePtr obj(new ob::PathLengthOptimizationObjective(si));
    obj->setCostThreshold(ob::Cost(1.51));
    return obj;
}

class ClearanceObjective : public ob::StateCostIntegralObjective
{
public: ClearanceObjective(const ob::SpaceInformationPtr& si) : ob::StateCostIntegralObjective(si, true)
    {
    }

    ob::Cost stateCost(const ob::State* s) const
    {
        ompl::base::RealVectorStateSpace::StateType* point_to_be_checked=(ompl::base::RealVectorStateSpace::StateType*)s ;
        double x,y;
        x=point_to_be_checked->values[0];
        y=point_to_be_checked->values[1];
        #ifdef LOG_STATECOST
        std::cout<<"-------------------calling stateCost-------------------"<<std::endl;
        std::cout << "x,y " << x <<" , " <<y <<std::endl;
        std::cout<<"cost of state(these costs will be summed for whole the path, the lower better path ): " <<ob::Cost(1 / si_->getStateValidityChecker()->clearance(s)) <<std::endl;
        std::cout<<"---------------------------------------------"<<std::endl;
        #endif
        return ob::Cost(1 / si_->getStateValidityChecker()->clearance(s));

    }
};

ob::OptimizationObjectivePtr getClearanceObjective(const ob::SpaceInformationPtr& si)
{
    return ob::OptimizationObjectivePtr(new ClearanceObjective(si));
}

ob::OptimizationObjectivePtr shortrisky(const ob::SpaceInformationPtr& si)
{
    ob::OptimizationObjectivePtr lengthObj(new ob::PathLengthOptimizationObjective(si));
    ob::OptimizationObjectivePtr clearObj(new ClearanceObjective(si));
    return 100.0*lengthObj + 1.0*clearObj;
}

ob::OptimizationObjectivePtr longsafe(const ob::SpaceInformationPtr& si)
{
    ob::OptimizationObjectivePtr lengthObj(new ob::PathLengthOptimizationObjective(si));
    ob::OptimizationObjectivePtr clearObj(new ClearanceObjective(si));

    return 1.0*lengthObj + 1000.0*clearObj;
}



void optimal_palnning_without_setting_path(int argc, char** argv)
{
    ob::StateSpacePtr space(new ob::RealVectorStateSpace(2));
    space->as<ob::RealVectorStateSpace>()->setBounds(0.0, 99.0);
    ob::SpaceInformationPtr si(new ob::SpaceInformation(space));
    si->setStateValidityChecker(ob::StateValidityCheckerPtr(new ValidityChecker(si)));
    si->setup();
    ob::ScopedState<> start(space);
    start->as<ob::RealVectorStateSpace::StateType>()->values[0] = 99;
    start->as<ob::RealVectorStateSpace::StateType>()->values[1] = 0;
    ob::ScopedState<> goal(space);
    goal->as<ob::RealVectorStateSpace::StateType>()->values[0] = 0;
    goal->as<ob::RealVectorStateSpace::StateType>()->values[1] = 99;
    ob::ProblemDefinitionPtr pdef(new ob::ProblemDefinition(si));



    pdef->setStartAndGoalStates(start, goal);





// this will turn our planner into optimisation based:
/*
// only one of these should be enabled:
//    pdef->setOptimizationObjective(shortrisky(si));
//    pdef->setOptimizationObjective(getClearanceObjective(si));
    pdef->setOptimizationObjective(longsafe(si));
    ob::PlannerPtr optimizingPlanner(new og::RRTstar(si));

    optimizingPlanner->setProblemDefinition(pdef);
    optimizingPlanner->setup();

    ompl::base::ParamSet param_set= optimizingPlanner->params();
    std::map<std::string, std::string> planner_specific_parameters_kv;
    planner_specific_parameters_kv["range"]="1";
    param_set.setParams(planner_specific_parameters_kv);


    ob::PlannerStatus solved = optimizingPlanner->solve(15.0);
*/

// this will turn our planner into optimisation based:

    ob::PlannerPtr RRTConnectPlanner(new og::RRTConnect(si));

    RRTConnectPlanner->setProblemDefinition(pdef);
    RRTConnectPlanner->setup();


    ompl::base::ParamSet param_set= RRTConnectPlanner->params();
    std::map<std::string, std::string> planner_specific_parameters_kv;
    planner_specific_parameters_kv["range"]="1";
    param_set.setParams(planner_specific_parameters_kv);


    ob::PlannerStatus solved = RRTConnectPlanner->solve(15.0);





    if (solved)
    {
        // Output the length of the path found
        std::cout << "Found solution of path length " << pdef->getSolutionPath()->length() << std::endl;
        if (argc > 1)
        {
            std::ofstream outFile(argv[1]);
            boost::static_pointer_cast<og::PathGeometric>(pdef->getSolutionPath())->printAsMatrix(outFile);
            outFile.close();
        }
        for(std::size_t i=0;i<boost::static_pointer_cast<og::PathGeometric>(pdef->getSolutionPath())->getStates().size();i++)
        {
            boost::static_pointer_cast<og::PathGeometric>(pdef->getSolutionPath())->getState(i);
        }
    }
    else
        std::cout << "No solution found." << std::endl;

}

void populate_EDT_Matrix_old()
{
    EDT_Matrix.resize(100,100);
    std::string line;
    std::string delimiter = ",";
    std::ifstream myfile ("/home/behnam/optimisation_based_path_planner_ws/devel/lib/sample_based_optimisation_based_path_planner/mat2.txt");
    int i,j;
    i=0;
    j=0;
    if (myfile.is_open())
    {
        while ( getline (myfile,line) )
        {

            size_t pos = 0;
            std::string token;
            j=0;
            while ((pos = line.find(delimiter)) != std::string::npos)
            {

                token = line.substr(0, pos);
//                std::cout << i<<"," <<j << std::endl;

                EDT_Matrix(i,j)=   lexical_cast<double>(token);
                j++;

                line.erase(0, pos + delimiter.length());
            }
//            std::cout << i<<"," <<j << std::endl;
            EDT_Matrix(i,j)=   lexical_cast<double>(line);
            i++;
        }
        myfile.close();
    }

    else
    {
        std::cout << "Unable to open file...";
    }

    //std::cout<< EDT_Matrix<<std::endl;
}

//octomap::OcTree *tree = NULL;
//tree = new octomap::OcTree(0.01);
////read in octotree
//tree->readBinary(argv[1]);
//std::cout<<"read in tree, "<<tree->getNumLeafNodes()<<" leaves "<<std::endl;


//void  creatEDTfromOctomap( boost::shared_ptr<octomap::OcTree> tree_ptr ,boost::shared_ptr<DynamicEDTOctomap> distmap_ptr)
//{
//    double x,y,z;
//    tree_ptr->getMetricMin(x,y,z);
//    octomap::point3d min(x,y,z);
//    std::cout<<"Metric min: "<<x<<","<<y<<","<<z<<std::endl;
//    tree_ptr->getMetricMax(x,y,z);
//    octomap::point3d max(x,y,z);
//    std::cout<<"Metric max: "<<x<<","<<y<<","<<z<<std::endl;

//    bool unknownAsOccupied = true;
//    unknownAsOccupied = false;
//    float maxDist = 1.0;
//    //- the first argument ist the max distance at which distance computations are clamped
//    //- the second argument is the octomap
//    //- arguments 3 and 4 can be used to restrict the distance map to a subarea
//    //- argument 5 defines whether unknown space is treated as occupied or free
//    //The constructor copies data but does not yet compute the distance map
//    DynamicEDTOctomap distmap(maxDist, tree_ptr.get(), min, max, unknownAsOccupied);

//    //This computes the distance map
//    distmap.update();

//    //This is how you can query the map
//    octomap::point3d p(0.0,0.0,0.0);
//    //As we don't know what the dimension of the loaded map are, we modify this point
////    p.x() = min.x() + 0.3 * (max.x() - min.x());
////    p.y() = min.y() + 0.6 * (max.y() - min.y());
////    p.z() = min.z() + 0.5 * (max.z() - min.z());

//    octomap::point3d closestObst;
//    float distance;

//    distmap.getDistanceAndClosestObstacle(p, distance, closestObst);

//    std::cout<<"\n\ndistance at point "<<p.x()<<","<<p.y()<<","<<p.z()<<" is "<<distance<<std::endl;
//    if(distance < distmap.getMaxDist())
//    std::cout<<"closest obstacle to "<<p.x()<<","<<p.y()<<","<<p.z()<<" is at "<<closestObst.x()<<","<<closestObst.y()<<","<<closestObst.z()<<std::endl;



//    return;
//}



#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>

template <class PointT>
void convertPCLpointcloudToOctomapPointCloud(pcl::PointCloud<PointT>& pclCloud, octomap::Pointcloud& octomapCloud)
{
    octomapCloud.reserve(pclCloud.points.size());
    typename   pcl::PointCloud<PointT>::const_iterator it;
    for (it = pclCloud.begin(); it != pclCloud.end(); ++it)
    {
        // Check if the point is invalid
        if (!std::isnan (it->x) && !std::isnan (it->y) && !std::isnan (it->z))
        octomapCloud.push_back(it->x, it->y, it->z);
    }
}

void convertOctomapPointCloudToOctomaptree(octomap::Pointcloud &octomapCloud, octomap::OcTree &octomap_Octree )
{
    octomap::point3d origin (0.0f, 0.0f, 0.0f);
    octomap_Octree.insertPointCloud(octomapCloud, origin);
}


#include <opencv/cv.h>
#include <opencv2/contrib/contrib.hpp>
#include <opencv2/highgui/highgui.hpp>


void threeDimMatrix()
{
    int out[3];
    out[0] = 2;
    out[1] = 2;
    out[2] = 2;

    //Alternative 1:
    cv::Mat M(3, out, CV_32FC1, cv::Scalar(0));
    M.at<float>(0,0,0) = 1.2;
    M.at<float>(0,0,1) = 2.4;
    M.at<float>(0,1,0) = 3.9;
    M.at<float>(0,1,1) = 3.9;
    M.at<float>(1,0,0) = 1.7;



    std::cout<<M.at<float>(0,0,0) <<std::endl;
    std::cout<<M.at<float>(1,0,0) <<std::endl;


    std::cout<<"M.dims: " <<M.dims <<std::endl;
    std::cout<<"M.rows: " <<M.rows <<std::endl;
    std::cout<<"M.cols: " <<M.cols <<std::endl;

    boost::shared_ptr<DynamicEDTOctomap>  distmap_ptr;
    //This is how you can query the map

    //As we don't know what the dimension of the loaded map are, we modify this point
//    p.x() = min.x() + 0.3 * (max.x() - min.x());
//    p.y() = min.y() + 0.6 * (max.y() - min.y());
//    p.z() = min.z() + 0.5 * (max.z() - min.z());



    cv::Mat cm_img0;
    std::cout<<"cm_img0.dims: " <<cm_img0.dims <<std::endl;
    // Apply the colormap:

//    M.reshape(2,4);
    cv::applyColorMap(M, cm_img0, cv::COLORMAP_JET);
    // Show the result:
//    cv::imshow("cm_img0", cm_img0);
//    cv::imwrite("cm_img0.png", cm_img0);




//    std::cout<<"cm_img0.rows: "  <<cm_img0.rows <<std::endl;
//    std::cout<<"cm_img0.cols: " <<cm_img0.cols <<std::endl;


    octomap::point3d closestObst;
    float distance;

    octomap::point3d p;
/*
    float x_step_size,y_step_size,z_step_size;
    float x_upper_limit,x_lower_limit,y_upper_limit,y_lower_limit,z_upper_limit,z_lower_limit;
    for(float x=x_lower_limit;x<x_upper_limit;x=x+x_step_size)
    {
        for(float y=y_lower_limit;y<y_upper_limit;y=y+y_step_size)
        {

            for(float z=z_lower_limit;z<z_upper_limit;z=z+z_step_size)
            {
                p.x()=x;
                p.x()=y;
                p.x()=z;
                distmap_ptr->getDistanceAndClosestObstacle(p, distance, closestObst);
            }
        }

    }

*/

}



ros::Publisher vis_pub;


template< class PointT>
void createPCLHorizontalBAR(pcl::PointCloud<PointT>& pclCloud, double delta=0)
{
    double x,y,z, lenght, width,height, step_size;

    x=0.50;
    y=-01.20;
    z=0.30 ;
    lenght=0.40;
    width=0.30;
    height=0.60;
    step_size=0.01;

    pcl::PointXYZ pcl_point;
    double epsilon =0.012;

    for(double i=x+epsilon;i<x+lenght+epsilon;i=i+step_size)//x
    {
        for(double j=y+epsilon;j<y+width+epsilon;j=j+step_size)//y
        {
            for(double k=z+epsilon;k<z+height+epsilon;k=k+step_size)//z
            {
                pcl_point.x=i+delta;
                pcl_point.y=j+delta;
                pcl_point.z=k+delta;
                pclCloud.points.push_back(pcl_point);
            }
        }
    }

}




void markers_visulizer(ros::NodeHandle &node_handle, visualization_msgs::MarkerArray &marker_array)
{
    ros::Publisher vis_pub = node_handle.advertise<visualization_msgs::MarkerArray>( "visualization_marker_array", 0 );
    ros::WallDuration sleep_t(0.4);
    sleep_t.sleep();
    vis_pub.publish( marker_array );
    sleep_t.sleep();
    return;
}

void markers_visulizer(ros::NodeHandle &node_handle, std::vector<visualization_msgs::Marker> &markers)
{
    visualization_msgs::MarkerArray marker_array;
    ros::Publisher vis_pub = node_handle.advertise<visualization_msgs::MarkerArray>( "visualization_marker_array", 0 );

    for(int i=0;i<markers.size();i++)
    {
        marker_array.markers.push_back(markers.at(i));
    }
    ros::WallDuration sleep_t(0.4);
    sleep_t.sleep();
    vis_pub.publish( marker_array );
    sleep_t.sleep();
    return;
}

void wipe_marker(ros::NodeHandle &node_handle, std::vector<visualization_msgs::Marker> &markers)
{
    for(std::size_t i=0;i<markers.size();i++)
    {
        markers.at(i).action = visualization_msgs::Marker::DELETE;
    }
    markers_visulizer(node_handle, markers);
    markers.clear();
}

template <class PointT>
void publish_pclPointCloud(ros::NodeHandle &node_handle, pcl::PointCloud<PointT> &cloud)
{

    ros::Publisher  pointcloud2_publisher = node_handle.advertise<sensor_msgs::PointCloud2>("pointcloud2", 1, true);
    sensor_msgs::PointCloud2 pointcould_msg;
    pcl::toROSMsg<PointT> (cloud,pointcould_msg);

    std::string link_name="base_link";
    pointcould_msg.header.frame_id=link_name;
    ros::WallDuration sleep_time(2.0);
//    sleep_time.sleep();
    pointcloud2_publisher.publish(pointcould_msg);
    sleep_time.sleep();

}

void publish_octree(ros::NodeHandle &node_handle, octomap::OcTree &octomap_ocTree)
{

    ros::Publisher  octomap_ocTree_Publisher = node_handle.advertise<octomap_msgs::Octomap>("octomapbinary", 1, true);
    std::string link_name="base_link";
    octomap_msgs::Octomap octomap_msg;
    octomap_msg.header.frame_id = link_name;
    octomap_msg.header.stamp = ros::Time::now();
    if (octomap_msgs::binaryMapToMsg(octomap_ocTree,octomap_msg))
    {
        ros::WallDuration sleep_time(4.0);
        sleep_time.sleep();
        octomap_ocTree_Publisher.publish(octomap_msg);
        sleep_time.sleep();
    }
    else
    {
        std::cout<<"Error serializing OctoMap"<<std::endl;
    }
}

void test_visulaizing_markers(int argc , char** argv)
{
    ros::init(argc,argv,"test_planning_collision_aila_dynamic_environment_replanning",1);
    ros::NodeHandle node_handle;
    vis_pub = node_handle.advertise<visualization_msgs::MarkerArray>( "visualization_marker_array", 0 );
    ros::WallDuration sleep_time(0.05);
    std::string link_name="base_link";
    int number_of_markers=10;

//    visualization_msgs::MarkerArray marker_array;
//    visualization_msgs::Marker marker;


















    pcl::PointCloud<pcl::PointXYZ> bar_cloud;
    createPCLHorizontalBAR<pcl::PointXYZ>(bar_cloud);

    //pointcloud2
    publish_pclPointCloud(node_handle,bar_cloud);


    octomap::Pointcloud octomapCloud_bar;
    convertPCLpointcloudToOctomapPointCloud<pcl::PointXYZ>( bar_cloud   , octomapCloud_bar);
    octomap::OcTree octomap_Octree(0.01);
    convertOctomapPointCloudToOctomaptree(octomapCloud_bar, octomap_Octree );






    double x,y,z;
    octomap_Octree.getMetricMin(x,y,z);
    octomap::point3d min(x,y,z);
    std::cout<<"Metric min: "<<x<<","<<y<<","<<z<<std::endl;
    octomap_Octree.getMetricMax(x,y,z);
    octomap::point3d max(x,y,z);
    std::cout<<"Metric max: "<<x<<","<<y<<","<<z<<std::endl;

    bool unknownAsOccupied = true;
    unknownAsOccupied = false;
    float maxDist = 2.0;
    //- the first argument ist the max distance at which distance computations are clamped
    //- the second argument is the octomap
    //- arguments 3 and 4 can be used to restrict the distance map to a subarea
    //- argument 5 defines whether unknown space is treated as occupied or free
    //The constructor copies data but does not yet compute the distance map


    min.x()=-2;
    min.y()=-2;
    min.z()=-2;


    max.x()=2;
    max.y()=2;
    max.z()=2;
    DynamicEDTOctomap distmap(maxDist, &octomap_Octree, min, max, unknownAsOccupied);

    //This computes the distance map
    distmap.update();

    //This is how you can query the map
    octomap::point3d p;


    octomap::point3d closestObst;
    float distance;






    float x_step_size,y_step_size,z_step_size;
    float x_upper_limit,x_lower_limit,y_upper_limit,y_lower_limit,z_upper_limit,z_lower_limit;


    float scale_x, scale_y,scale_z;

    scale_x=0.2;
    scale_y=0.2;
    scale_z=0.2;



    x_step_size=0.2;
    y_step_size=0.2;
    z_step_size=0.2;


    x_upper_limit=1;
    x_lower_limit=-1;
    y_upper_limit=1;
    y_lower_limit=-1;
    z_upper_limit=1;
    z_lower_limit=-1;


    int distance_image_matrix_rows,distance_image_matrix_cols;

    distance_image_matrix_rows=(ceil((x_upper_limit-x_lower_limit)/x_step_size )+1)  * (ceil((y_upper_limit-y_lower_limit)/y_step_size ) +1) * (ceil( (z_upper_limit-z_lower_limit)/z_step_size )+1)     ;
    distance_image_matrix_cols=1;

    cv::Mat distance_image_matrix(distance_image_matrix_rows, distance_image_matrix_cols, CV_32FC1,cvScalar(0.));

    std::cout<< distance_image_matrix.rows  <<std::endl;
    std::cout<< distance_image_matrix.cols  <<std::endl;
/*

    srand (time(NULL));
    int i=0;

    int j=0;
    int k=0;

    for(float x=x_lower_limit;x<x_upper_limit;x=x+x_step_size)
    {
        j=0;
        for(float y=y_lower_limit;y<y_upper_limit;y=y+y_step_size)
        {
            k=0;
            for(float z=z_lower_limit;z<z_upper_limit;z=z+z_step_size)
            {
                p.x()=x;
                p.y()=y;
                p.z()=z;




//                std::cout<< "p.x(): " << p.x() <<std::endl;
//                std::cout<< "p.y(): " << p.y() <<std::endl;
//                std::cout<< "p.z(): " << p.z() <<std::endl;


                distmap.getDistanceAndClosestObstacle(p, distance, closestObst);

//                std::cout<< x << "," <<y<<"," <<z <<std::endl;
//                std::cout<<i <<std::endl;

                distance_image_matrix.at<float>(i,0) = distance;




//                std::cout<< "Distance: " << distance <<std::endl;
//                std::cout<<"closestObst.x(): " <<closestObst.x() <<std::endl;
//                std::cout<<"closestObst.y(): " <<closestObst.y() <<std::endl;
//                std::cout<<"closestObst.z(): " <<closestObst.z() <<std::endl;

                marker.header.frame_id = link_name;
                marker.header.stamp = ros::Time();
                marker.id = i;
                marker.color.a = 1.0;
        //        marker.ns=name_space;
                marker.lifetime = ros::Duration();
                marker.color.r = (1.0*(rand() % 100) )/100;
                marker.color.g = (1.0*(rand() % 100) )/100;
                marker.color.b = (1.0*(rand() % 100) )/100;


//                marker.color.r = 120;
//                marker.color.g = 0;
//                marker.color.b = 0;


                marker.action = visualization_msgs::Marker::ADD;
                marker.pose.position.x=x;
                marker.pose.position.y=y;
                marker.pose.position.z=z;

                marker.pose.orientation.w= 1;
                marker.pose.orientation.x=0;
                marker.pose.orientation.y=0;
                marker.pose.orientation.z=0;
                marker.text="visual_array.at(i)->group_name";
                marker.type = visualization_msgs::Marker::CUBE;
//                marker.scale.x=x_step_size;
//                marker.scale.y=y_step_size;
//                marker.scale.z=z_step_size;

                marker.scale.x=scale_x;
                marker.scale.y=scale_y;
                marker.scale.z=scale_z;

                marker_array.markers.push_back(marker);
                i++;
                k++;

            }
//            std::cout<<"k: "<<k <<std::endl;
//            j++;
        }
//        std::cout<<"j: "<<j <<std::endl;
    }
//     std::cout<<"i: "<<i <<std::endl;


    cv::Mat cm_img0,distance_image_matrix_normalized;

    cv::normalize(distance_image_matrix, distance_image_matrix_normalized, 255, 0,cv::NORM_MINMAX);

    cv::Mat m2(distance_image_matrix_normalized.rows, distance_image_matrix_normalized.cols,CV_8UC1,distance_image_matrix_normalized.data);
//, cv::CV_8UC4, distance_image_matrix_normalized.data


//    cv::Mat M=m2.reshape(1,11*11);

    cv::applyColorMap(m2, cm_img0, cv::COLORMAP_HOT);
    cv::imwrite("cm_img0.bmp",cm_img0);


    std::cout<<"channels: "<<cm_img0.channels() <<std::endl;
    std::cout<<"rows: "<<cm_img0.rows <<std::endl;
    std::cout<<"cols: "<<cm_img0.cols <<std::endl;


    cv::Mat cm_img0_resized;
    cv::Size size(10*cm_img0.cols,10*cm_img0.rows);//the dst image size,e.g.100x100
//    Mat dst;//dst image
//    Mat src;//src image
    cv::resize(cm_img0,cm_img0_resized,size);//resize image
    cv::imwrite("cm_img0_resized.png", cm_img0_resized);

//    cv::imshow("cm_img0", cm_img0_resized);



    cv::waitKey(0);


    std::vector<cv::Mat> bgr_planes;
    cv::split( cm_img0, bgr_planes );

    cv::Mat blue, green, red;

    blue=bgr_planes.at(0);
    green=bgr_planes.at(1);
    red=bgr_planes.at(2);




    cv::Mat cm_img0_normalized;
    cv::normalize(cm_img0, cm_img0_normalized, 1, 0,cv::NORM_MINMAX);

    std::cout<<"cm_img0.type(): "<<cm_img0.type()<<std::endl;

    std::cout<<"cm_img0_normalized.type(): "<<cm_img0_normalized.type()<<std::endl;


    for(std::size_t i=0;i<marker_array.markers.size();i++)
    {
//        marker_array.markers.at(i).color.b = (1.0 * cm_img0.at<cv::Vec3b>(i,0)[0]) /255.0;
//        marker_array.markers.at(i).color.g = (1.0 *cm_img0.at<cv::Vec3b>(i,0)[1])/255.0;
//        marker_array.markers.at(i).color.r =(1.0 * cm_img0.at<cv::Vec3b>(i,0)[2] )/255.0;



        marker_array.markers.at(i).color.r=(red.at<uchar>(i,0)*1.0 )/255.0;
        marker_array.markers.at(i).color.g=(green.at<uchar>(i,0)*1.0) /255.0;
        marker_array.markers.at(i).color.b=(blue.at<uchar>(i,0)*1.0 )/255.0 ;
    }

//    markers_visulizer(node_handle, marker_array);




*/
//----------------------------------------------------------------------------------------------------------



    // the topic is octomapbinary
//    publish_octree(node_handle, octomap_Octree);

    std::string filename="/home/behnam/optimisation_based_path_planner_ws/src/sample_based_optimisation_based_path_planner/files/mesh_files/Dude.binvox.bt";
    octomap::OcTree* octree = new octomap::OcTree(filename);
    publish_octree(node_handle, *octree);


//    filename="/home/behnam/optimisation_based_path_planner_ws/src/sample_based_optimisation_based_path_planner/files/mesh_files/room.binvox.bt";
//    octree = new octomap::OcTree(filename);
//    publish_octree(node_handle, *octree);



   double x_min,y_min,z_min, x_max,y_max,z_max;

    x_min=-2;
    y_min=-2;
    z_min=0;

    x_max=2;
    y_max=2;
    z_max=2;

    std::vector<float> distances_vector;


     double step_size=0.2;

     for(double x=x_min;x<x_max;x=x+step_size)
     {
         for(double y=y_min;y<y_max;y=y+step_size)
         {
             for(double z=z_min;z<z_max;z=z+step_size)
             {
                 octomap::point3d p(x,y,z);
                 distances_vector.push_back(distmap.getDistance(p));
             }
         }
     }



    cv::Mat mat_raw_distances_vector(distances_vector);




    cv::Mat mat_normilized_distances_value(mat_raw_distances_vector.rows,mat_raw_distances_vector.cols ,CV_8UC1);
    cv::normalize( mat_raw_distances_vector, mat_normilized_distances_value, 0, 255, cv::NORM_MINMAX, CV_8UC1, cv::Mat() );

    cv::Mat cm_img0;

//cv::COLORMAP_AUTUMN
//    cv:0:COLORMAP_BONE

    cv::applyColorMap(mat_normilized_distances_value, cm_img0,cv::COLORMAP_HOT);



    std::cout<<"cm_img0.rows: " <<cm_img0.rows <<std::endl;
    std::cout<<"cm_img0.cols: " <<cm_img0.cols <<std::endl;





    std::vector<cv::Mat> bgr_planes;
    cv::split( cm_img0, bgr_planes );

    cv::Mat blue, green, red;

    blue=bgr_planes.at(0);
    green=bgr_planes.at(1);
    red=bgr_planes.at(2);



    std::cout<<"blue.rows: " <<blue.rows <<std::endl;
    std::cout<<"blue.cols: " <<blue.cols <<std::endl;



    std::string frame_id="base_link";

    ros::Publisher vis_pub = node_handle.advertise<visualization_msgs::Marker >( "visualization_marker", 0 );


    visualization_msgs::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = ros::Time();
    marker.ns = "my_namespace";
    marker.id = lexical_cast<int>(1);
    marker.type = visualization_msgs::Marker::CUBE_LIST;
//    marker.type = visualization_msgs::Marker::SPHERE_LIST;
    marker.action = visualization_msgs::Marker::ADD;



//    std::string file_name;
//    pcl::PolygonMesh mesh;
//    pcl::io::loadPolygonFileSTL(file_name,mesh);





     int i=0;
     for(double x=x_min;x<x_max;x=x+step_size)
     {
         for(double y=y_min;y<y_max;y=y+step_size)
         {
             for(double z=z_min;z<z_max;z=z+step_size)
             {
                 //std::cout<<"i: " << i<<std::endl;
                 geometry_msgs::Point point;
                 std_msgs::ColorRGBA color;

                 color.a=0.03;
                 point.x=x;
                 point.y=y;
                 point.z=z;

                 color.r=(red.at<uchar>(i,0)*1.0 )/255.0;
                 color.g=(green.at<uchar>(i,0)*1.0) /255.0;
                 color.b=(blue.at<uchar>(i,0)*1.0 )/255.0 ;

                 marker.points.push_back(point);
                 marker.colors.push_back(color);
                 marker.scale.x=0.2;
                 marker.scale.y=0.2;
                 marker.scale.z=0.2;
                 i++;
             }
         }
     }




    std::string tmp_keyboard_press = "";

    ros::WallDuration sleep_t(2);
    sleep_t.sleep();
    ros::spinOnce();
    std::cout << "Please enter to start publishing :"<<  std::endl;
    getline(std::cin, tmp_keyboard_press);


    vis_pub.publish( marker );
    sleep_t.sleep();
    ros::spinOnce();















    return;

}

using namespace cv;

void colormapExample(int argc , char** argv)
{
    // Get the path to the image, if it was given
    // if no arguments were given.
    string filename;
    if (argc > 1) {
        filename = string(argv[1]);
    }
    // The following lines show how to apply a colormap on a given image
    // and show it with cv::imshow example with an image. An exception is
    // thrown if the path to the image is invalid.
    if(!filename.empty())
    {
//        Mat img0 = imread(filename);

//        cv::Mat distance_image_matrix(distance_image_matrix_rows, distance_image_matrix_cols, CV_32FC1,cvScalar(0.));
//        cv::Mat img = cv::imread(imagePath, CV_LOAD_IMAGE_ANYCOLOR | CV_LOAD_IMAGE_ANYDEPTH);


        Mat img0 = imread( argv[1], CV_LOAD_IMAGE_GRAYSCALE );

        std::cout<<"channels: "<<img0.channels() <<std::endl;
        std::cout<<"rows: "<<img0.rows <<std::endl;
        std::cout<<"cols: "<<img0.cols <<std::endl;


        // Throw an exception, if the image can't be read:
        if(img0.empty()) {
            CV_Error(CV_StsBadArg, "Sample image is empty. Please adjust your path, so it points to a valid input image!");
        }
        // Holds the colormap version of the image:
        Mat cm_img0;
        // Apply the colormap:
        applyColorMap(img0, cm_img0, COLORMAP_JET);


        std::cout<<"channels: "<<cm_img0.channels() <<std::endl;
        std::cout<<"rows: "<<cm_img0.rows <<std::endl;
        std::cout<<"cols: "<<cm_img0.cols <<std::endl;


        // Show the result:
        imshow("cm_img0", cm_img0);
        waitKey(0);
    }


}

int main( int argc , char** argv)
{

//    colormapExample(argc , argv);

    test_visulaizing_markers(argc ,argv);

//    Initialize the grid from the BMP file.

/*
    SDL_Surface *grid_from__BMP_file = SDL_LoadBMP( "../../../src/sample_based_optimisation_based_path_planner/images/problem2.bmp" );
    Signed_Distance_Fields_test(grid_from__BMP_file);
*/
/*
    populate_EDT_Matrix_old();
    optimal_palnning_without_setting_path(argc,  argv);
*/
//    threeDimMatrix();
}
