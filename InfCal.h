#pragma once
#ifndef  INFCAL_H_INCLUDE
#define INFCAL_H_INCLUDE

#include "VecT.h"
#include <vector>
#include <iostream>
using namespace std;

double DTimeCal(double num_area, double v_UAV, double RunLens);//期望耗时计算
double DUAVNum(double num_area, double mintime, double v_UAV, double RunLens);//最少无人机计算
//double RealTimeCal(const vector<VecPathPlan>& VPP, double* StartPosition, double* PathPlanPosition, double v_UAV);//无人机实际耗时计算



double DTimeCal(double num_area, double v_UAV, double RunLens)
{
	double dtime;
	return dtime = (num_area / (v_UAV * RunLens));
}
double DUAVNum(double num_area, double mintime, double v_UAV, double RunLens)
{
	double dUAVnum;
	return dUAVnum = ceil((num_area / (v_UAV * RunLens)) / mintime);
}

//double RealTimeCal(const vector<VecPathPlan>& VPP, double* StartPosition, double* PathPlanPosition, double v_UAV)//无人机实际耗时计算
//{
//
//}

#endif // ! INFCAL_H_INCLUDE
