#pragma once
#ifndef PJ_H_INCLUDE
#define PJ_H_INCLUDE

#include "VecT.h"
#include <vector>
using namespace std;

void getLinePara(double& x1, double& y1, double& x2, double& y2, LinePara& LP);//获取直线参数
double DisCal(double dx1, double dy1, double dx2, double dy2);//计算距离

void getLinePara(double& x1, double& y1, double& x2, double& y2, LinePara& LP)//获取直线参数
{
    double m = 0;

    // 计算分子  
    m = x2 - x1;

    if (0 == m)
    {
        LP.k = 10000.0;
        LP.b = y1 - LP.k * x1;
    }
    else
    {
        LP.k = (y2 - y1) / (x2 - x1);
        LP.b = y1 - LP.k * x1;
    }
}

void getLineLocation(double x1, double y1, double x2, double y2, double target_l, double axis_flag, LineLocation& LL)//输出坐标点
{
    double m = 0;

    // 计算分子  
    m = x2 - x1;
    double k, b;

    if (0 == m)
    {
        k = 10000.0;
        b = y1 - k * x1;
    }
    else
    {
        k = (y2 - y1) / (x2 - x1);
        b = y1 - k * x1;
    }


    if (axis_flag == 0)
    {
        LL.x = target_l;
        LL.y = k * target_l + b;

    }
    else if (axis_flag == 1)
    {
        LL.y = target_l;

        if (x1 == x2)
        {
            LL.x = x1;
        }
        else
        {
            LL.x = (target_l - b) / k;
        } 
        
    }
    //cout << "x1=" << x1 << "-y1=" << y1 << endl;
    //cout << "x2=" << x2 << "-y2=" << y2 << endl;
    //cout << "target_l=" << target_l << endl;
    //cout << "k=" << k << endl;
    //cout << "b=" << b << endl;
}


double DisCal(double dx1, double dy1, double dx2, double dy2)//计算距离
{
    double rd = 0.0;
    rd = sqrt(pow((dx1 - dx2), 2) + pow((dy1 - dy2), 2));
    return rd;
}

#endif // !PJ_H_INCLUDE

