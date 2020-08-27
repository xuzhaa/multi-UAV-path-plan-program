#pragma once
#ifndef SPP_H_INCLUDE
#define SPP_H_INCLUDE

#include "VecT.h"
#include "MAM.h"
#include "PJ.h"
#include <vector>
#include "PathPlan.h"
#include "PathPlan2.h"

const double lon_nom3 = 102834.74258026089786013677476285;// 经度
const double lat_nom3 = 111712.69150641055729984301412873;//纬度

using namespace std;

void PathPlanStartPoint(const vector<Vec_tcp>& CTPOL, int axis_flag, double* StartPosition, double RunLens);//单机路径规划起点
/*
CTPOL         -单机区域顶点
axis_flag     -区域划分标志 基于x轴ory轴
StartPosition -无人机起飞点
RunLens       -扫描长度
*/

void SPL(double arr_MM[], double dx1, double dy1, double axis_flag, double Rlens, Offset_SP& OSP);//计算起点坐标偏移

void PathPlanStartPoint(const vector<Vec_tcp>& CTPOL, int axis_flag, double* StartPosition, double RunLens)
{
	double Min_dis, Min_dis_next, Path_x1, Path_y1, Min_dis_index;//最小距离，距离坐标，坐标下标
	double CrDis, Min_CrDis_x, Min_CrDis_y, Min_CrDis_index;//相对距离
	double PathLineStart_x=0, PathLineStart_y=0;//起飞点
	double *ptr_CrDis=new double[CTPOL.size() - 1];//存放区域内边长大小
	double* ptr_Min_dis = new double[CTPOL.size() - 1];//存放区域顶点与起飞点距离
	double arr_MPP[8];//传回的xy大小值
	double o_index;


	//初始化-将第一个点作为最小值点
	double temp_dis = DisCal(CTPOL[0].x, CTPOL[0].y, StartPosition[0], StartPosition[1]);//将第一个点作为最小值点

	//初始化-将第一个点作为相对位置最小值点  相对位置为该点与无人机起飞点距离
	double temp_CrDis = 0;//将第一个点作为最小值点
	double temp_MinCrDis = INT_MAX;//最小相对位置点

	getMaxAndMinPathPoint(CTPOL, arr_MPP);//获取POL中的最小和最大值
	double ptr_MMP[8];//获取最大最小值以及极值点
	for (int i = 0; i < 8; i++)
	{
		ptr_MMP[i] = arr_MPP[i];
	}


	PPStartPoint PPSP;//结构体
	
	//查找距离StartPosition最近的点及区域内相邻两点之间相对距离
	for (int i = 0; i < CTPOL.size() - 1; i++){
		//计算i点以及i+1点距离起飞点的距离
		Min_dis = DisCal(CTPOL[i].x, CTPOL[i].y, StartPosition[0], StartPosition[1]);//计算起飞点与划分区域各顶点的距离
		Min_dis_next = DisCal(CTPOL[i + 1].x, CTPOL[i + 1].y, StartPosition[0], StartPosition[1]);//i+1这个顶点与起飞点距离

		//计算i点以及i+1点之间的距离
		CrDis = DisCal(CTPOL[i].x, CTPOL[i].y, CTPOL[i + 1].x, CTPOL[i + 1].y);//计算区域内各顶点之间的相对距离

		//x轴为起点
		if (axis_flag == 0){
			if ((CrDis >= temp_CrDis) && ((CTPOL[i].x - CTPOL[i + 1].x) == 0)&& ((Min_dis <= temp_MinCrDis) || (Min_dis_next <= temp_MinCrDis))){//垂直于x轴的最大起飞点
				temp_CrDis = CrDis;//实小距离
				if (Min_dis > Min_dis_next){//i+1点更近
					temp_MinCrDis = Min_dis_next;
					Offset_SP OSP;
					SPL(ptr_MMP, CTPOL[i + 1].x, CTPOL[i + 1].y, axis_flag, RunLens,OSP);//路径起点以之为基准
					Path_x1 = OSP.offset1;
					o_index = OSP.index_num;

					LineLocation paraL1;
					getLineLocation(CTPOL[i + 1].x, CTPOL[i + 1].y, CTPOL[i + 2].x, CTPOL[i + 2].y, Path_x1, axis_flag, paraL1);//获取起始点

					PPSP.st_x = paraL1.x;
					PPSP.st_y = paraL1.y;
					PPSP.index_axis = i + 1;
					PPSP.index_flag = 1;

					/*确定方向i+1>i*/
					if ((CTPOL[i+1].y - CTPOL[i].y)>0){//i+1在上面i在下 朝下飞
						PPSP.vector_flag = 1;
					}
					else{
						PPSP.vector_flag = 0;
					}

					PathLineStart_x = PPSP.st_x;
					PathLineStart_y = PPSP.st_y;
				}
				else{//i点更近
					temp_MinCrDis = Min_dis;
					Offset_SP OSP;
					SPL(ptr_MMP, CTPOL[i].x, CTPOL[i].y, axis_flag, RunLens, OSP);//路径起点以之为基准
					Path_x1=OSP.offset1;
					o_index = OSP.index_num;

					LineLocation paraL1;
					if (i == 0){//i=0时由第一个点和最后一个点的切线确定
						getLineLocation(CTPOL[i].x, CTPOL[i].y, CTPOL[CTPOL.size() - 2].x, CTPOL[CTPOL.size() - 2].y, Path_x1, 0, paraL1);//获取起始点
					}
					else{
						getLineLocation(CTPOL[i].x, CTPOL[i].y, CTPOL[i - 1].x, CTPOL[i - 1].y, Path_x1, axis_flag, paraL1);//获取起始点
					}
					PPSP.st_x = paraL1.x;
					PPSP.st_y = paraL1.y;
					//PPSP.index_axis = o_index;
					PPSP.index_axis = i;
					PPSP.index_flag = 0;

					//确定方向i+1>i
					if ((CTPOL[i].y - CTPOL[i+1].y) > 0){//i在上面i+1在下 朝下飞
						PPSP.vector_flag = 1;
					}
					else{
						PPSP.vector_flag = 0;
					}

					PathLineStart_x = PPSP.st_x;
					PathLineStart_y = PPSP.st_y;
				}
			}
		}
		else{ //axis_flag==1 y轴上
				
			if ((CrDis >= temp_CrDis) && ((CTPOL[i].y - CTPOL[i + 1].y) == 0)&&((Min_dis<= temp_MinCrDis)||(Min_dis_next <= temp_MinCrDis))){//垂直于y轴的最大起飞点
				temp_CrDis = CrDis;
			
				if (Min_dis > Min_dis_next){//i+1点更近
					temp_MinCrDis = Min_dis_next;
					Offset_SP OSP;
					SPL(ptr_MMP, CTPOL[i + 1].x, CTPOL[i + 1].y, axis_flag, RunLens, OSP);//路径起点以之为基准
					Path_y1 = OSP.offset1;
					o_index = OSP.index_num;

					cout << "Path_y1" << Path_y1 << endl;
					LineLocation paraL1;
					getLineLocation(CTPOL[i + 1].x, CTPOL[i + 1].y, CTPOL[i + 2].x, CTPOL[i + 2].y, Path_y1, axis_flag, paraL1);//获取起始点
					PPSP.st_x = paraL1.x;
					PPSP.st_y = paraL1.y;
					PPSP.index_axis = i + 1;
					PPSP.index_flag = 1;

					/*确定方向i+1>i*/
					if ((CTPOL[i + 1].x - CTPOL[i].x) > 0){//i+1在右i在左 朝左飞
						PPSP.vector_flag = 3;//向左规划
					}
					else{
						PPSP.vector_flag = 2;//向右规划
					}

					PathLineStart_x = PPSP.st_x;
					PathLineStart_y = PPSP.st_y;
				}
				else{//i点更近		
					temp_MinCrDis = Min_dis;
					Offset_SP OSP;
					SPL(ptr_MMP, CTPOL[i].x, CTPOL[i].y, axis_flag, RunLens, OSP);//路径起点以之为基准
					Path_y1 = OSP.offset1;
					o_index = OSP.index_num;

					LineLocation paraL1;
					if (i == 0){
						getLineLocation(CTPOL[i].x, CTPOL[i].y, CTPOL[CTPOL.size() - 2].x, CTPOL[CTPOL.size() - 2].y, Path_y1, axis_flag, paraL1);//获取起始点
					}
					else{
						getLineLocation(CTPOL[i].x, CTPOL[i].y, CTPOL[i - 1].x, CTPOL[i - 1].y, Path_y1, axis_flag, paraL1);//获取起始点
					}
					PPSP.st_x = paraL1.x;
					PPSP.st_y = paraL1.y;
					PPSP.index_axis = i;
					PPSP.index_flag = 0;

					/*确定方向i->i+1*/
					if ((CTPOL[i].x - CTPOL[i+1].x) > 0){//i+1在右i在左 朝左飞
						PPSP.vector_flag = 3;
					}
					else{
						PPSP.vector_flag = 2;
					}
					PathLineStart_x = PPSP.st_x;
					PathLineStart_y = PPSP.st_y;
				}
			}
		}

	}

	
	cout << "Start path plan" << endl;
	//SinglePathPlan(CTPOL, arr_MPP, axis_flag, RunLens, PPSP);
	SinglePathPlan2(CTPOL, arr_MPP, axis_flag, RunLens, PPSP);
	cout << "PathLineStart_x=" << (PathLineStart_x/lat_nom3) << "-" << "PathLineStart_y=" << (PathLineStart_y/lon_nom3) << endl;

}

	void SPL(double arr_MM[], double dx1, double dy1,double axis_flag, double Rlens, Offset_SP& OSP){ //计算起点坐标偏移
		double Offset_1;
		if (axis_flag == 0){//x轴为基准时
			if (dx1 == arr_MM[0]){//最小值点
				OSP.offset1= dx1 + (Rlens / 2);
				OSP.index_num = 0;
				//return Offset_1 = dx1 + (Rlens / 2);
			}
			else if (dx1 == arr_MM[1]){//最大值点
				OSP.offset1 = dx1 - (Rlens / 2);;
				OSP.index_num = 1;
				//return Offset_1 = dx1 - (Rlens / 2);
			}
		}
		else{
			if (dy1 == arr_MM[2]){//最小值点
				OSP.offset1 = dy1 + (Rlens / 2);;
				OSP.index_num = 2;
				//return Offset_1 = dy1 + (Rlens / 2);
			}
			else if (dy1 == arr_MM[3]){//最大值点
				OSP.offset1= dy1 - (Rlens / 2);
				OSP.index_num = 3;
				//return Offset_1 = dy1 - (Rlens / 2);
			}
		}
	}

#endif // !SPP_H_INCLUDE
