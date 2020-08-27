#pragma once
#ifndef VECT_H_INCLUDE
#define VECT_H_INCLUDE

//��άdoubleʸ��--���������
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

//�е�
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

//·���滮��
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

//�����е����
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

//���ϵ�
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

//��άʸ��--�е�
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

// ����ֱ�߲����ṹ��  
struct LinePara
{
    double k;
    double b;

};

// ����ֱ�߲����ṹ��  
struct LineLocation
{
    double x;
    double y;

};

//ƫ����
struct Offset_SP
{
    double offset1;
    int index_num;
};

//��ʼ�������Ϣ
struct PPStartPoint
{
    double st_x;//��ʼ��x
    double st_y;//��ʼ��y
    int vector_flag;//��������滮���� 0-�� 1-�� 2-�� 3-��
    int index_flag;//��������һ�� 0---i->i+1  1---i+1->i
    int index_axis;//������������һ��

};


#endif // !AC_H_INCLUDE
#pragma once
