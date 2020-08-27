#pragma once
#ifndef AC_H_INCLUDE
#define AC_H_INCLUDE

#include "VecT.h"
#include <vector>
using namespace std;

double vec_ac(const vector<Vec2d>& POL);//计算切分面积
double vec_ac2(const vector<Vec_tcp>& CTPOL);

double vec_ac(const vector<Vec2d>& POL)//计算切分面积
{
    double sum2 = 0;
    for (int i = 0; i < POL.size() - 1; i++)
    {
        sum2 += (POL[i].x * POL[i + 1].y - POL[i + 1].x * POL[i].y);
    }
    sum2 = -(sum2 / 2);
    return sum2;
}//计算面积

double vec_ac2(const vector<Vec_tcp>& CTPOL)//计算子切分面积
{
    double sum2_2 = 0;
    for (int i = 0; i < CTPOL.size() - 1; i++)
    {
        sum2_2 += (CTPOL[i].x * CTPOL[i + 1].y - CTPOL[i + 1].x * CTPOL[i].y);
    }
    sum2_2 = -(sum2_2 / 2);
    return sum2_2;
}//计算面积



#endif // !AC_H_INCLUDE
