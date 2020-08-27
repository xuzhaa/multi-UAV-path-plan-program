#pragma once
#ifndef  INFCAL_H_INCLUDE
#define INFCAL_H_INCLUDE

#include "VecT.h"
#include <vector>
#include <iostream>
using namespace std;

double DTimeCal(double num_area, double v_UAV, double RunLens);//������ʱ����
double DUAVNum(double num_area, double mintime, double v_UAV, double RunLens);//�������˻�����
//double RealTimeCal(const vector<VecPathPlan>& VPP, double* StartPosition, double* PathPlanPosition, double v_UAV);//���˻�ʵ�ʺ�ʱ����



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

//double RealTimeCal(const vector<VecPathPlan>& VPP, double* StartPosition, double* PathPlanPosition, double v_UAV)//���˻�ʵ�ʺ�ʱ����
//{
//
//}

#endif // ! INFCAL_H_INCLUDE
