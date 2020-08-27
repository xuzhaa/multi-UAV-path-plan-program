#pragma once
#ifndef MAM_H_INCLUDE
#define MAM_H_INCLUDE

#include "VecT.h"

#include <vector>


using namespace std;

double getMaxAndMin(const vector<Vec2d>& POL, double arr_M[]);//计算大小值
double getMaxAndMinPathPoint(const vector<Vec_tcp>& CTPOL, double arr_MPP[]);//计算大小值


double getMaxAndMin(const vector<Vec2d>& POL, double arr_M[])//计算大小值
{
    //static double  arr_M[4];
    double POL_max_x = 0, POL_max_y = 0;//定义最大值
    double POL_min_x = 0, POL_min_y = 0;//定义最小值
    double POL_max_x_index=0, POL_max_y_index=0;//坐标最大值下标
    double POL_min_x_index=0, POL_min_y_index=0;//坐标最小值下标
    POL_max_x = POL[0].x; POL_max_y = POL[0].y;//置零
    POL_min_x = POL[0].x; POL_min_y = POL[0].y;
    for (int i = 0; i < POL.size() - 1; i++)
    {
        if (POL_max_x < POL[i].x)//比较x轴最大值
        {
            POL_max_x = POL[i].x;
            POL_max_x_index = i;
        }
        if (POL_max_y < POL[i].y)//比较y轴最大值
        {
            POL_max_y = POL[i].y;
            POL_max_y_index = i;
        }
        if (POL_min_x > POL[i].x)//比较x轴最小值
        {
            POL_min_x = POL[i].x;
            POL_min_x_index = i;
        }
        if (POL_min_y > POL[i].y)//比较y轴最小值
        {
            POL_min_y = POL[i].y;
            POL_min_y_index = i;
        }
    }
    arr_M[0] = POL_min_x;//赋值给arr_M数组
    arr_M[1] = POL_max_x;
    arr_M[2] = POL_min_y;
    arr_M[3] = POL_max_y;
    arr_M[4] = POL_min_x_index;
    arr_M[5] = POL_max_x_index;
    arr_M[6] = POL_min_y_index;
    arr_M[7] = POL_max_y_index;

    return *arr_M;//返回指针
}

double getMaxAndMinPathPoint(const vector<Vec_tcp>& CTPOL, double arr_MPP[])//计算大小值
{
    //static double  arr_M[4];
    double CTPOL_max_x = 0, CTPOL_max_y = 0;//定义最大值
    double CTPOL_min_x = 0, CTPOL_min_y = 0;//定义最小值
    double CTPOL_max_x_index = 0, CTPOL_max_y_index = 0;//坐标最大值下标
    double CTPOL_min_x_index = 0, CTPOL_min_y_index = 0;//坐标最小值下标
    CTPOL_max_x = CTPOL[0].x; CTPOL_max_y = CTPOL[0].y;//置零
    CTPOL_min_x = CTPOL[0].x; CTPOL_min_y = CTPOL[0].y;

    for (int i = 0; i < CTPOL.size() - 1; i++)
    {
        if (CTPOL_max_x < CTPOL[i].x)//比较x轴最大值
        {
            CTPOL_max_x = CTPOL[i].x;
            CTPOL_max_x_index = i;
        }
        if (CTPOL_max_y < CTPOL[i].y)//比较y轴最大值
        {
            CTPOL_max_y = CTPOL[i].y;
            CTPOL_max_y_index = i;
            //cout << "CTPOL_max_y=" << CTPOL_max_y << endl;
        }
        if (CTPOL_min_x >= CTPOL[i].x)//比较x轴最小值
        {
            CTPOL_min_x = CTPOL[i].x;
            CTPOL_min_x_index = i;
        }
        if (CTPOL_min_y >= CTPOL[i].y)//比较y轴最小值
        {
            CTPOL_min_y = CTPOL[i].y;
            CTPOL_min_y_index = i;
        }
    }
    arr_MPP[0] = CTPOL_min_x;//赋值给arr_M数组
    arr_MPP[1] = CTPOL_max_x;
    arr_MPP[2] = CTPOL_min_y;
    arr_MPP[3] = CTPOL_max_y;
    arr_MPP[4] = CTPOL_min_x_index;
    arr_MPP[5] = CTPOL_max_x_index;
    arr_MPP[6] = CTPOL_min_y_index;
    arr_MPP[7] = CTPOL_max_y_index;
    //cout <<"MAM="<< CTPOL_min_y <<"-"<< CTPOL_max_y << endl;

    return *arr_MPP;//返回指针
}




#endif // !MAM_H_INCLUDE
