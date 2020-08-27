#pragma once
#ifndef VECT_H_INCLUDE
#define VECT_H_INCLUDE

//二维double矢量--输入坐标的
struct  Vec2d
{
    double x, y;

    Vec2d()
    {
        x = 0.0;
        y = 0.0;
    }
    Vec2d(double dx, double dy)
    {
        x = dx;
        y = dy;
    }
    void Set(double dx, double dy)
    {
        x = dx;
        y = dy;
    }
};

//切点
struct  Vec_cp
{
    double x, y;

    Vec_cp()
    {
        x = 0.0;
        y = 0.0;
    }
    Vec_cp(double dx, double dy)
    {
        x = dx;
        y = dy;
    }
    Vec_cp Set(double dx, double dy)
    {
        x = dx;
        y = dy;
    }
};

//路径规划点
struct  VecPathPlan
{
    double x, y;

    VecPathPlan()
    {
        x = 0.0;
        y = 0.0;
    }
    VecPathPlan(double dx, double dy)
    {
        x = dx;
        y = dy;
    }
    VecPathPlan Set(double dx, double dy)
    {
        x = dx;
        y = dy;
    }
};

//结算切点面积
struct  Vec_tcp
{
    double x, y;

    Vec_tcp()
    {
        x = 0.0;
        y = 0.0;
    }
    Vec_tcp(double dx, double dy)
    {
        x = dx;
        y = dy;
    }
    Vec_tcp Set(double dx, double dy)
    {
        x = dx;
        y = dy;
    }
};

//避障点
struct  VecOA
{
    double x, y;

    VecOA()
    {
        x = 0.0;
        y = 0.0;
    }
    VecOA(double dx, double dy)
    {
        x = dx;
        y = dy;
    }
    VecOA Set(double dx, double dy)
    {
        x = dx;
        y = dy;
    }
};

//三维矢量--切点
struct  Vec3_cp
{
    double x, y, z;

    Vec3_cp()
    {
        x = 0.0;
        y = 0.0;
        z = 0.0;
    }
    Vec3_cp(double dx, double dy, double dz)
    {
        x = dx;
        y = dy;
        z = dz;
    }
    Vec3_cp Set(double dx, double dy, double dz)
    {
        x = dx;
        y = dy;
        z = dz;
    }
};

// 定义直线参数结构体  
struct LinePara
{
    double k;
    double b;

};

// 定义直线参数结构体  
struct LineLocation
{
    double x;
    double y;

};

//偏移量
struct Offset_SP
{
    double offset1;
    int index_num;
};

//起始点相关信息
struct PPStartPoint
{
    double st_x;//起始点x
    double st_y;//起始点y
    int vector_flag;//起点期望规划方向 0-上 1-下 2-右 3-左
    int index_flag;//长边另外一点 0---i->i+1  1---i+1->i
    int index_axis;//靠近坐标轴哪一点

};


#endif // !AC_H_INCLUDE
#pragma once
