#include "lmcma.hpp"
#include <stdio.h>
#include <vector>
#include <iostream>
#include <cmath>

#include <iostream>
#include <fstream>


#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

using namespace Eigen;



void cost_function(double x, double y, double &z)
{

//python cost function:    Z=4*np.exp(-(X-4)**2 - (Y-4)**2)+2*np.exp(-(X-2)**2 - (Y-2)**2)
//if we don't limit the upper and lower bound the answer is answer x,y=4.4 z=40, if we limit the other pick is a x=2,y=2
    z=-( 4* exp(- ( pow(x-4,2)+ pow(y-4,2)   )  )
         +2* exp(- ( pow(x-2,2)+ pow(y-2,2)   )  )
        );
}

void test_lmcma(int argc , char **argv)
{
    const int numParams = 2;
    double params_hiBounds[numParams];
    double params_loBounds[numParams];
    double params[numParams];


    params_hiBounds[0]=15;
    params_hiBounds[1]=15;
    params_loBounds[0]=-2;
    params_loBounds[1]=-2;


    double f;
    std::vector<double> x,y,z;

    params[0] = 0.0;
    params[1] = 0.0;
    LMCMA opt(params, -1,params_loBounds,params_hiBounds);
    opt.init(numParams);
    for(int i = 0; i < 1000; i++)
    {
        opt.getNextParameterVector(params, numParams);

        cost_function(params[0], params[1], f);
        opt.setEvaluationFeedback(&f, 1);


        x.push_back(params[0] );
        y.push_back(params[1] );
        z.push_back(f);
    }

    std::cout<< "the optimum point is:" << x.at(x.size()-1) <<","<<y.at(y.size()-1) <<std::endl;

    std::ofstream path_to_max;
    path_to_max.open ("path_to_min.csv");
    path_to_max << "x,y,z,\n";

    for(std::size_t i=0;i<x.size();i++)
    {
        path_to_max<< x.at(i) <<" , "<<y.at(i)<<" , " << z.at(i)<<"\n";
    }
    path_to_max.close();



}

void test_lmcma_using_cov(int argc , char **argv)
{
    const int numParams = 2;
    double params_hiBounds[numParams];
    double params_loBounds[numParams];
    double params[numParams];


    params_hiBounds[0]=15;
    params_hiBounds[1]=15;
    params_loBounds[0]=-2;
    params_loBounds[1]=-2;

    double f;
    std::vector<double> x,y,z;

    params[0] = 0.0;
    params[1] = 0.0;

    double sigma=1;

    double cov[numParams*numParams];
    covariance(numParams, 1, cov);

    LMCMA opt(params, -1,params_loBounds,params_hiBounds,sigma, cov);

    opt.init(numParams);
    for(int i = 0; i < 1000; i++)
    {
        opt.getNextParameterVector(params, numParams);
        cost_function(params[0], params[1], f);
        opt.setEvaluationFeedback(&f, 1);
        x.push_back(params[0] );
        y.push_back(params[1] );
        z.push_back(f);
    }
    std::cout<< "the optimum point is:" << x.at(x.size()-1) <<","<<y.at(y.size()-1) <<std::endl;

    std::ofstream path_to_max;
    path_to_max.open ("path_to_max_using_cov.csv");
    path_to_max << "x,y,z,\n";

    for(std::size_t i=0;i<x.size();i++)
    {
        path_to_max<< x.at(i) <<" , "<<y.at(i)<<" , " << z.at(i)<<"\n";
    }
    path_to_max.close();


}


int main(int argc, char ** argv)
{
    test_lmcma(argc , argv);
    test_lmcma_using_cov(argc , argv);
}

