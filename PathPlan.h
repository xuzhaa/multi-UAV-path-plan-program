#pragma once
#ifndef PATHPLAN_H_INCLUDE
#define PATHPLAN_H_INCLUDE

#include <iostream>
#include "VecT.h"
#include "MAM.h"
#include "PJ.h"
#include <vector>

using namespace std;

 void SinglePathPlan(const vector<Vec_tcp>& CTPOL, double *arr_MPP, int axis_flag,double RunLens, PPStartPoint& PPSP);//生成路径点
 /*
 CTPOL--切分子区域特征点
  arr_MPP--切分子区域最大范围
  axis_flag--切分坐标轴
  Runlens--扫描区域(直径)
  PPSP--路径规划起始点
 */

 void SinglePathPlan(const vector<Vec_tcp>& CTPOL, double* arr_MPP, int axis_flag, double RunLens, PPStartPoint& PPSP)//生成路径点
 {
	 vector<VecPathPlan> VPP;//创建一个位置vector
	 VPP.push_back(VecPathPlan(PPSP.st_x, PPSP.st_y));//填充路径起点


	 double Plan_length = 1;//规划步长
	 double flag_HV = 0;//横纵切换
	 int flag_pathtime = 0;//规划次数
	 double vector_flag_2;//方向2   0-上 1-下 2-右 3-左
	 double axis_min_x, axis_max_x, axis_min_y, axis_max_y;//获取划分区域极限范围，也即划分子区域
	 double temp_path_x, temp_path_y;//临时路径规划点
	 double flag_origin_vrc;
	 int num_Z;
	 axis_min_x = arr_MPP[0];
	 axis_max_x = arr_MPP[1];
	 axis_min_y = arr_MPP[2];
	 axis_max_y = arr_MPP[3];

	 
	 //计算总折返次数
	num_Z = ceil((axis_max_x - axis_min_x) / RunLens);
	cout <<"RS"<<RunLens<<"minx"<<axis_min_x<<"maxx"<<axis_max_x<< "num_Z=" << num_Z << endl;

	//初始位置为起点
	temp_path_x = PPSP.st_x;
	temp_path_y = PPSP.st_y; 
	//cout << "temp_path_x=" << temp_path_x << endl;

	flag_origin_vrc = PPSP.index_flag;//0-指向i 1-指向i+1

	 for (int i = 0; i < num_Z; i++)//生成每一个折返点 
	 {
		 if (axis_flag == 0)//切分轴在x轴上，则以y轴衍生为主
		 {
			 //cout << "axis_flag=" << axis_flag << endl;

			 if (CTPOL[PPSP.index_axis].x == arr_MPP[0])//在最小值点，需要往最大值点走
			 {
				 vector_flag_2 = 2;//y轴向右走
			 }
			 else if (CTPOL[PPSP.index_axis].x == arr_MPP[1])//在最大值点，需要往最小值点走
			 {
				 vector_flag_2 = 3;//y轴向左走
			 }
			 //cout << "vector_flag_2=" << vector_flag_2 << endl;

			 if (PPSP.vector_flag == 0)//往上规划
			 {

				 cout << "num_Z=" << i << endl;
				 LineLocation paraO1;
				 if (PPSP.index_flag == 1)//i+1->i
				 {
					 cout << "PPSP.index_flag=" << PPSP.index_flag << endl;
					 /*temp_path_x = temp_path_x+ flag_pathtime* RunLens;*/
					 //if (PPSP.index_axis == 0)
					 //{
						// getLineLocation(CTPOL[PPSP.index_axis].x, CTPOL[PPSP.index_axis].y, CTPOL[CTPOL.size()-2].x, CTPOL[CTPOL.size() - 2].y, temp_path_x, axis_flag, paraO1);//获取交点
					 //}
					 //else
					 //{
					 getLineLocation(CTPOL[PPSP.index_axis - 1].x, CTPOL[PPSP.index_axis - 1].y, CTPOL[PPSP.index_axis - 2].x, CTPOL[PPSP.index_axis - 2].y, temp_path_x, axis_flag, paraO1);//获取交点
				 /*}*/
					 cout << "parao1=" << paraO1.x << "-" << paraO1.y << endl;
					VPP.push_back(VecPathPlan(paraO1.x, paraO1.y));//填充纵向路径点

					 PPSP.vector_flag = 1;
					 PPSP.index_flag = 0;
					 //cout << "PPSP.index_flag =" << PPSP.index_flag << endl;
					 //cout << "temp_path_x=" << temp_path_x << endl;
					 if ((temp_path_x >= (axis_min_x )) && (temp_path_x <= axis_max_x ))
					 {
						
						 //flag_pathtime = flag_pathtime + 1;
						 if (vector_flag_2 == 2)//向右填充一个值
						 {
							 temp_path_x = temp_path_x +   RunLens;
						 }
						 else if (vector_flag_2 == 3)//向左填充一个值
						 {
							 temp_path_x = temp_path_x -   RunLens;
						 }
						 getLineLocation(CTPOL[PPSP.index_axis-1].x, CTPOL[PPSP.index_axis-1].y, CTPOL[PPSP.index_axis - 2].x, CTPOL[PPSP.index_axis - 2].y, temp_path_x, axis_flag, paraO1);//获取交点
						 cout << "parao1=" << paraO1.x <<"-"<< paraO1.y << endl;
						 temp_path_x = paraO1.x;
						 temp_path_y = paraO1.y;
						 VPP.push_back(VecPathPlan(paraO1.x, paraO1.y));//填充横向路径点
					 }
					 else
					 {
						 PPSP.vector_flag = 2;
						 //cout << "PPSP.vector_flag=" << PPSP.vector_flag << endl;
					 }
				 }
				 else if (PPSP.index_flag == 0)//i->i+1
				 {
					 cout << "PPSP.index_flag=" << PPSP.index_flag << endl;
					 cout << "PPSP.index_axis=" << PPSP.index_axis << endl;
					 getLineLocation(CTPOL[PPSP.index_axis+1 ].x, CTPOL[PPSP.index_axis+1 ].y, CTPOL[PPSP.index_axis + 2].x, CTPOL[PPSP.index_axis + 2].y, temp_path_x, axis_flag, paraO1);//获取交点
					 VPP.push_back(VecPathPlan(paraO1.x, paraO1.y));//填充纵向路径点
					cout << "parao1=" << paraO1.x << "-" << paraO1.y << endl;
					 PPSP.vector_flag = 1;
					 PPSP.index_flag = 1;
					 if ((temp_path_x >= (axis_min_x)) && (temp_path_x <= axis_max_x))
					 {
						
						 //flag_pathtime = flag_pathtime + 1;
						 if (vector_flag_2 == 2)//向右填充一个值
						 {
							 temp_path_x = temp_path_x +  RunLens;
						 }
						 else if (vector_flag_2 == 3)//向左填充一个值
						 {
							 temp_path_x = temp_path_x - RunLens;
						 }
						 getLineLocation(CTPOL[PPSP.index_axis + 1].x, CTPOL[PPSP.index_axis + 1].y, CTPOL[PPSP.index_axis + 2].x, CTPOL[PPSP.index_axis + 2].y, temp_path_x, axis_flag, paraO1);//获取交点
						
						 temp_path_x = paraO1.x;
						 temp_path_y = paraO1.y;
						 VPP.push_back(VecPathPlan(paraO1.x, paraO1.y));//填充横向路径点
						 cout << "parao1=" << paraO1.x << "-" << paraO1.y << endl;
					 }
					 else
					 {
						 PPSP.vector_flag = 2;
					 }
				 }
			 }
			 else if (PPSP.vector_flag == 1)//往下规划
			 {
				 cout << "num_Z=" << i << endl;
				 LineLocation paraO1;
				 if (PPSP.index_flag == 0)//i->i+1
				 {
					 getLineLocation(CTPOL[PPSP.index_axis ].x, CTPOL[PPSP.index_axis ].y, CTPOL[PPSP.index_axis + 1].x, CTPOL[PPSP.index_axis +1].y, temp_path_x, axis_flag, paraO1);//获取交点
					 VPP.push_back(VecPathPlan(paraO1.x, paraO1.y));//填充路径起点
					 
					 cout << "paraO1.x=" << paraO1.x << "-" << paraO1.y << endl;
					 PPSP.vector_flag = 0;
					 PPSP.index_flag = 1;
					 //cout << "PPSP.vector_flag=" << PPSP.vector_flag << endl;
					 cout << " PPSP.index_flag=" << PPSP.index_flag << endl;
					 if ((temp_path_x >= (axis_min_x)) && (temp_path_x <= axis_max_x))
					 {
						/* cout << "temp_path=" << temp_path_x<<"-"<< temp_path_y << endl;*/

						
						 //flag_pathtime = flag_pathtime + 1;
						 if (vector_flag_2 == 2)
						 {
							 temp_path_x = temp_path_x + RunLens;
						 }
						 else if (vector_flag_2 == 3)
						 {
							 temp_path_x = temp_path_x -   RunLens;
							 cout << "temp_path=" << temp_path_x << "-" << temp_path_y << endl;
						 }
						 getLineLocation(CTPOL[PPSP.index_axis].x, CTPOL[PPSP.index_axis].y, CTPOL[PPSP.index_axis + 1].x, CTPOL[PPSP.index_axis + 1].y, temp_path_x, axis_flag, paraO1);//获取交点
						 temp_path_x = paraO1.x;
						temp_path_y = paraO1.y;
						VPP.push_back(VecPathPlan(temp_path_x, temp_path_y));//填充横向路径点
						 cout << "paraO1.x=" << paraO1.x << "-" << paraO1.y << endl;
						
					 }
					 else
					 {
						 PPSP.vector_flag = 2;
					 }
				 }
				 else if (PPSP.index_flag == 1)//i+1->i
				 {
					 cout << "PPSP.index_flag=" << PPSP.index_flag << endl;
					 //cout << "temp_path=" << temp_path_x << "-" << temp_path_y << endl;
					 //if ((PPSP.index_axis-1) == 0)
					 //{
						// getLineLocation(CTPOL[PPSP.index_axis-1].x, CTPOL[PPSP.index_axis-1].y, CTPOL[CTPOL.size()-2].x, CTPOL[CTPOL.size() - 2].y, temp_path_x, axis_flag, paraO1);//获取交点
					 //}
					 //else
					 //{
						// getLineLocation(CTPOL[PPSP.index_axis - 1].x, CTPOL[PPSP.index_axis - 1].y, CTPOL[PPSP.index_axis - 2].x, CTPOL[PPSP.index_axis - 2].y, temp_path_x, axis_flag, paraO1);//获取交点
					 //}

					getLineLocation(CTPOL[PPSP.index_axis ].x, CTPOL[PPSP.index_axis].y, CTPOL[CTPOL.size() - 2].x, CTPOL[CTPOL.size() - 2].y, temp_path_x, axis_flag, paraO1);//获取交点
					cout << "paraO1.x=" << paraO1.x << "-" << paraO1.y << endl;
					VPP.push_back(VecPathPlan(paraO1.x, paraO1.y));//填充路径起点
					 //cout << "paraO1.x=" << paraO1.x << "-" << paraO1.y << endl;
					 PPSP.vector_flag = 0;
					 PPSP.index_flag = 0;
					 if ((temp_path_x >= (axis_min_x)) && (temp_path_x <= axis_max_x))
					 {

						 //flag_pathtime = flag_pathtime + 1;
						 if (vector_flag_2 == 2)
						 {
							 temp_path_x = temp_path_x +   RunLens;
						 }
						 else if (vector_flag_2 == 3)
						 {
							 temp_path_x = temp_path_x -   RunLens;
						 }
						 getLineLocation(CTPOL[PPSP.index_axis].x, CTPOL[PPSP.index_axis].y, CTPOL[CTPOL.size() - 2].x, CTPOL[CTPOL.size() - 2].y, temp_path_x, axis_flag, paraO1);//获取交点

						 temp_path_x = paraO1.x;
						 temp_path_y = paraO1.y;
						 VPP.push_back(VecPathPlan(paraO1.x, paraO1.y));//填充横向路径点
					 }
					 else
					 {
						 PPSP.vector_flag = 2;
					 }
				 }
			 }
		 }
		 else if (axis_flag == 1)//切分轴在y轴上，则以x轴衍生为主
		 {
		 //cout << "axis_flag=" << axis_flag << endl;
		 if (CTPOL[PPSP.index_axis].y == arr_MPP[2])//在最小值点，需要往最大值点走
		 {
			 vector_flag_2 = 0;//y轴向右走
		 }
		 else if (CTPOL[PPSP.index_axis].y == arr_MPP[3])//在最大值点，需要往最小值点走
		 {
			 vector_flag_2 = 1;//y轴向左走
		 }
		 //cout << "vector_flag_2=" << vector_flag_2 << endl;


		 if (PPSP.vector_flag == 2)//往右规划
		 {
			 cout << "num_Z=" << i << endl;
			 cout << "向右规划-i+1->i" << endl;
			 LineLocation paraO1;
			 if (PPSP.index_flag == 1)//i+1->i
			 {
				 cout << "PPSP.index_flag=" << PPSP.index_flag << endl;
				 cout << "PPSP.index_axis=" << PPSP.index_axis << endl;
				 if (flag_origin_vrc == 0)//指向i
				 {
					 if (PPSP.index_axis == 0) {
						 getLineLocation(CTPOL[PPSP.index_axis].x, CTPOL[PPSP.index_axis].y, CTPOL[CTPOL.size() - 2].x, CTPOL[CTPOL.size() - 2].y, temp_path_y, axis_flag, paraO1);//获取交点
					 }
					 else {
						 getLineLocation(CTPOL[PPSP.index_axis].x, CTPOL[PPSP.index_axis].y, CTPOL[PPSP.index_axis - 1].x, CTPOL[PPSP.index_axis - 1].y, temp_path_y, axis_flag, paraO1);//获取交点

					 }
				 }
				 else if (flag_origin_vrc == 1)//指向i+1
				 {
					 if (PPSP.index_axis == 0)
					 {
						 getLineLocation(CTPOL[PPSP.index_axis].x, CTPOL[PPSP.index_axis].y, CTPOL[CTPOL.size() - 2].x, CTPOL[CTPOL.size() - 2].y, temp_path_y, axis_flag, paraO1);//获取交点
					 }
					 else {
						 getLineLocation(CTPOL[PPSP.index_axis - 1].x, CTPOL[PPSP.index_axis - 1].y, CTPOL[PPSP.index_axis - 2].x, CTPOL[PPSP.index_axis - 2].y, temp_path_y, axis_flag, paraO1);//获取交点
					 }
				 }

				 cout << "parao1=" << paraO1.x << "-" << paraO1.y << endl;
				 VPP.push_back(VecPathPlan(paraO1.x, paraO1.y));//填充纵向路径点

				 PPSP.vector_flag = 3;
				 PPSP.index_flag = 0;
				 cout << "PPSP.index_flag =" << PPSP.index_flag << endl;
				 cout << "temp_path_y=" << temp_path_y << endl;

				 if ((temp_path_y >= (axis_min_y)) && (temp_path_y <= axis_max_y))
				 {
					 if (vector_flag_2 == 0)//向上填充一个值
					 {
						 temp_path_y = temp_path_y + RunLens;
					 }
					 else if (vector_flag_2 == 1)//向下填充一个值
					 {
						 temp_path_y = temp_path_y - RunLens;
					 }
					 
					 if (flag_origin_vrc == 0)//指向i
					 {
						 if (PPSP.index_axis == 0) {
							 getLineLocation(CTPOL[PPSP.index_axis].x, CTPOL[PPSP.index_axis].y, CTPOL[CTPOL.size() - 2].x, CTPOL[CTPOL.size() - 2].y, temp_path_y, axis_flag, paraO1);//获取交点
						 }
						 else {
							 getLineLocation(CTPOL[PPSP.index_axis].x, CTPOL[PPSP.index_axis].y, CTPOL[PPSP.index_axis - 1].x, CTPOL[PPSP.index_axis - 1].y, temp_path_y, axis_flag, paraO1);//获取交点

						 }
					 }
					 else if (flag_origin_vrc == 1)//指向i+1
					 {
						 if (PPSP.index_axis == 0)
						 {
							 getLineLocation(CTPOL[PPSP.index_axis].x, CTPOL[PPSP.index_axis].y, CTPOL[CTPOL.size() - 2].x, CTPOL[CTPOL.size() - 2].y, temp_path_y, axis_flag, paraO1);//获取交点
						 }
						 else {
							 getLineLocation(CTPOL[PPSP.index_axis - 1].x, CTPOL[PPSP.index_axis - 1].y, CTPOL[PPSP.index_axis - 2].x, CTPOL[PPSP.index_axis - 2].y, temp_path_y, axis_flag, paraO1);//获取交点
						 }
					 }

					 cout << "parao1=" << paraO1.x << "-" << paraO1.y << endl;
					 temp_path_x = paraO1.x;
					 temp_path_y = paraO1.y;
					 VPP.push_back(VecPathPlan(paraO1.x, paraO1.y));//填充横向路径点
				 }
				 else
				 {
					 PPSP.vector_flag = 5;
					 cout << "向右规划-i+1->i  PPSP.vector_flag=" << PPSP.vector_flag << endl;
				 }
			 }
			 else if (PPSP.index_flag == 0)//i->i+1
			 {
				 cout << "向右规划-i->i+1" << endl;
				 cout << "PPSP.index_flag=" << PPSP.index_flag << endl;
				 cout << "PPSP.index_axis=" << PPSP.index_axis << endl;

				 if (flag_origin_vrc == 0)//指向i
				 {
					 if (PPSP.index_axis == (CTPOL.size() - 2)) {
						 getLineLocation(CTPOL[0].x, CTPOL[0].y, CTPOL[CTPOL.size() - 2].x, CTPOL[CTPOL.size() - 2].y, temp_path_y, axis_flag, paraO1);//获取交点
					 }
					 else {
						 getLineLocation(CTPOL[PPSP.index_axis + 1].x, CTPOL[PPSP.index_axis + 1].y, CTPOL[PPSP.index_axis + 2].x, CTPOL[PPSP.index_axis + 2].y, temp_path_y, axis_flag, paraO1);//获取交点
					 }
				 }
				 else if (flag_origin_vrc == 1)//指向i+1
				 {
					 if (PPSP.index_axis == (CTPOL.size() - 2)){
						 getLineLocation(CTPOL[1].x, CTPOL[1].y, CTPOL[CTPOL.size() - 2].x, CTPOL[CTPOL.size() - 2].y, temp_path_y, axis_flag, paraO1);//获取交点
					 }
					 else {
						 getLineLocation(CTPOL[PPSP.index_axis ].x, CTPOL[PPSP.index_axis ].y, CTPOL[PPSP.index_axis +1].x, CTPOL[PPSP.index_axis +1].y, temp_path_y, axis_flag, paraO1);//获取交点
					 }
				 }

				 //getLineLocation(CTPOL[PPSP.index_axis + 1].x, CTPOL[PPSP.index_axis + 1].y, CTPOL[PPSP.index_axis + 2].x, CTPOL[PPSP.index_axis + 2].y, temp_path_y, axis_flag, paraO1);//获取交点
				 VPP.push_back(VecPathPlan(paraO1.x, paraO1.y));//填充纵向路径点
				 cout << "parao1=" << paraO1.x << "-" << paraO1.y << endl;
				 PPSP.vector_flag = 3;
				 PPSP.index_flag = 1;
				 if ((temp_path_y >= (axis_min_y)) && (temp_path_y <= axis_max_y))
				 {
					 if (vector_flag_2 == 0)//向上填充一个值
					 {
						 temp_path_y = temp_path_y + RunLens;
					 }
					 else if (vector_flag_2 == 1)//向下填充一个值
					 {
						 temp_path_y = temp_path_y - RunLens;
					 }

					 if (flag_origin_vrc == 0)//指向i
					 {
						 if (PPSP.index_axis == (CTPOL.size() - 2)) {
							 getLineLocation(CTPOL[0].x, CTPOL[0].y, CTPOL[CTPOL.size() - 2].x, CTPOL[CTPOL.size() - 2].y, temp_path_y, axis_flag, paraO1);//获取交点
						 }
						 else {
							 getLineLocation(CTPOL[PPSP.index_axis + 1].x, CTPOL[PPSP.index_axis + 1].y, CTPOL[PPSP.index_axis + 2].x, CTPOL[PPSP.index_axis + 2].y, temp_path_y, axis_flag, paraO1);//获取交点
						 }
					 }
					 else if (flag_origin_vrc == 1)//指向i+1
					 {
						 if (PPSP.index_axis == (CTPOL.size() - 2)) {
							 getLineLocation(CTPOL[1].x, CTPOL[1].y, CTPOL[CTPOL.size() - 2].x, CTPOL[CTPOL.size() - 2].y, temp_path_y, axis_flag, paraO1);//获取交点
						 }
						 else {
							 getLineLocation(CTPOL[PPSP.index_axis].x, CTPOL[PPSP.index_axis].y, CTPOL[PPSP.index_axis + 1].x, CTPOL[PPSP.index_axis + 1].y, temp_path_y, axis_flag, paraO1);//获取交点
						 }
					 }

					 //getLineLocation(CTPOL[PPSP.index_axis + 1].x, CTPOL[PPSP.index_axis + 1].y, CTPOL[PPSP.index_axis + 2].x, CTPOL[PPSP.index_axis + 2].y, temp_path_y, axis_flag, paraO1);//获取交点

					 temp_path_x = paraO1.x;
					 temp_path_y = paraO1.y;
					 VPP.push_back(VecPathPlan(paraO1.x, paraO1.y));//填充横向路径点
					 cout << "parao1=" << paraO1.x << "-" << paraO1.y << endl;
				 }
				 else
				 {
					 PPSP.vector_flag = 5;
				 }
			 }
		 }
		 else if (PPSP.vector_flag == 3)//往左规划
		 {

			 cout << "num_Z=" << i << endl;
			 LineLocation paraO1;
			 if (PPSP.index_flag == 0)//i->i+1
			 {
				 cout << "向左规划-i->i+1" << endl;
				 cout << "PPSP.index_axis" << PPSP.index_axis << endl;

				 if (flag_origin_vrc == 0)//指向i
				 {
					 if (PPSP.index_axis == (CTPOL.size() - 2)){
						 getLineLocation(CTPOL[0].x, CTPOL[0].y, CTPOL[1].x, CTPOL[1].y, temp_path_y, axis_flag, paraO1);//获取交点
					 }
					 else{
						 getLineLocation(CTPOL[PPSP.index_axis + 1].x, CTPOL[PPSP.index_axis + 1].y, CTPOL[PPSP.index_axis + 2].x, CTPOL[PPSP.index_axis + 2].y, temp_path_y, axis_flag, paraO1);//获取交点
					 }
				 }
				 else if (flag_origin_vrc == 1)//指向i+1
				 {
					 if (PPSP.index_axis == (CTPOL.size() - 2)) {
						 getLineLocation(CTPOL[1].x, CTPOL[1].y, CTPOL[CTPOL.size() - 2].x, CTPOL[CTPOL.size() - 2].y, temp_path_y, axis_flag, paraO1);//获取交点
					 }
					 else {
						 getLineLocation(CTPOL[PPSP.index_axis].x, CTPOL[PPSP.index_axis].y, CTPOL[PPSP.index_axis + 1].x, CTPOL[PPSP.index_axis + 1].y, temp_path_y, axis_flag, paraO1);//获取交点
					 }
				 }

				 //if (PPSP.index_axis == (CTPOL.size() - 2))
				 //{

					// getLineLocation(CTPOL[0].x, CTPOL[0].y, CTPOL[1].x, CTPOL[1].y, temp_path_y, axis_flag, paraO1);//获取交点
				 //}
				 //else
				 //{
					// getLineLocation(CTPOL[PPSP.index_axis+1].x, CTPOL[PPSP.index_axis+1].y, CTPOL[PPSP.index_axis + 2].x, CTPOL[PPSP.index_axis +2].y, temp_path_y, axis_flag, paraO1);//获取交点
				 //}

				 
				 VPP.push_back(VecPathPlan(paraO1.x, paraO1.y));//填充路径起点

				 cout << "paraO1.x=" << paraO1.x << "-" << paraO1.y << endl;
				 PPSP.vector_flag = 2;
				 PPSP.index_flag = 1;
				 //cout << "PPSP.vector_flag=" << PPSP.vector_flag << endl;
				 cout << " PPSP.index_flag=" << PPSP.index_flag << endl;
				 if ((temp_path_y >= (axis_min_y)) && (temp_path_y <= axis_max_y))
				 {
					 if (vector_flag_2 == 0)
					 {
						 temp_path_y = temp_path_y + RunLens;
					 }
					 else if (vector_flag_2 == 1)
					 {
						 temp_path_y = temp_path_y - RunLens;
						 cout << "temp_path=" << temp_path_x << "-" << temp_path_y << endl;
					 }

					 if (flag_origin_vrc == 0)//指向i
					 {
						 if (PPSP.index_axis == (CTPOL.size() - 2)) {
							 getLineLocation(CTPOL[0].x, CTPOL[0].y, CTPOL[1].x, CTPOL[1].y, temp_path_y, axis_flag, paraO1);//获取交点
						 }
						 else {
							 getLineLocation(CTPOL[PPSP.index_axis + 1].x, CTPOL[PPSP.index_axis + 1].y, CTPOL[PPSP.index_axis + 2].x, CTPOL[PPSP.index_axis + 2].y, temp_path_y, axis_flag, paraO1);//获取交点
						 }
					 }
					 else if (flag_origin_vrc == 1)//指向i+1
					 {
						 if (PPSP.index_axis == (CTPOL.size() - 2)) {
							 getLineLocation(CTPOL[1].x, CTPOL[1].y, CTPOL[CTPOL.size() - 2].x, CTPOL[CTPOL.size() - 2].y, temp_path_y, axis_flag, paraO1);//获取交点
						 }
						 else {
							 getLineLocation(CTPOL[PPSP.index_axis].x, CTPOL[PPSP.index_axis].y, CTPOL[PPSP.index_axis + 1].x, CTPOL[PPSP.index_axis + 1].y, temp_path_y, axis_flag, paraO1);//获取交点
						 }
					 }

					 //if (PPSP.index_axis == (CTPOL.size() - 2))
					 //{

						// getLineLocation(CTPOL[0].x, CTPOL[0].y, CTPOL[1].x, CTPOL[1].y, temp_path_y, axis_flag, paraO1);//获取交点
					 //}
					 //else
					 //{
						// getLineLocation(CTPOL[PPSP.index_axis + 1].x, CTPOL[PPSP.index_axis + 1].y, CTPOL[PPSP.index_axis + 2].x, CTPOL[PPSP.index_axis + 2].y, temp_path_y, axis_flag, paraO1);//获取交点
					 //}
					temp_path_x = paraO1.x;
					 temp_path_y = paraO1.y;
					 cout << "11temp_path=" << temp_path_x << "-" << temp_path_y << endl;
					 VPP.push_back(VecPathPlan(temp_path_x, temp_path_y));//填充横向路径点
					 cout << "11paraO1.x=" << paraO1.x << "-" << paraO1.y << endl;

				 }
				 else
				 {
					 PPSP.vector_flag = 5;
				 }
			 }
			 else if (PPSP.index_flag == 1)//i+1->i
			 {
				 cout << "向左规划-i+1->i" << endl;

				 cout << "PPSP.index_flag=" << PPSP.index_flag << endl;

				 if (flag_origin_vrc == 0)//指向i
				 {
					 if (PPSP.index_axis == (CTPOL.size() - 2)) {
						 getLineLocation(CTPOL[PPSP.index_axis].x, CTPOL[PPSP.index_axis].y, CTPOL[PPSP.index_axis - 1].x, CTPOL[PPSP.index_axis - 1].y, temp_path_y, axis_flag, paraO1);//获取交点
					 }
					 else {
						 getLineLocation(CTPOL[PPSP.index_axis].x, CTPOL[PPSP.index_axis].y, CTPOL[PPSP.index_axis + 1].x, CTPOL[PPSP.index_axis + 1].y, temp_path_y, axis_flag, paraO1);//获取交点
					 }
				 }
				 else if (flag_origin_vrc == 1)//指向i+1
				 {
					 if (PPSP.index_axis == (CTPOL.size() - 2)) {
						 getLineLocation(CTPOL[1].x, CTPOL[1].y, CTPOL[CTPOL.size() - 2].x, CTPOL[CTPOL.size() - 2].y, temp_path_y, axis_flag, paraO1);//获取交点
					 }
					 else {
						 getLineLocation(CTPOL[PPSP.index_axis].x, CTPOL[PPSP.index_axis].y, CTPOL[PPSP.index_axis + 1].x, CTPOL[PPSP.index_axis + 1].y, temp_path_y, axis_flag, paraO1);//获取交点
					 }
				 }


				 //getLineLocation(CTPOL[PPSP.index_axis].x, CTPOL[PPSP.index_axis].y, CTPOL[CTPOL.size() - 2].x, CTPOL[CTPOL.size() - 2].y, temp_path_y, axis_flag, paraO1);//获取交点
				 cout << "paraO1.x=" << paraO1.x << "-" << paraO1.y << endl;
				 VPP.push_back(VecPathPlan(paraO1.x, paraO1.y));//填充路径起点
				 //cout << "paraO1.x=" << paraO1.x << "-" << paraO1.y << endl;

				 PPSP.vector_flag = 2;
				 PPSP.index_flag = 0;
				 if ((temp_path_y >= (axis_min_y)) && (temp_path_y <= axis_max_y))
				 {
					 if (vector_flag_2 == 2)
					 {
						 temp_path_y = temp_path_y + RunLens;
					 }
					 else if (vector_flag_2 == 3)
					 {
						 temp_path_y = temp_path_y - RunLens;
					 }

					 if (flag_origin_vrc == 0)//指向i
					 {
						 if (PPSP.index_axis == (CTPOL.size() - 2)) {
							 getLineLocation(CTPOL[PPSP.index_axis].x, CTPOL[PPSP.index_axis].y, CTPOL[PPSP.index_axis - 1].x, CTPOL[PPSP.index_axis - 1].y, temp_path_y, axis_flag, paraO1);//获取交点
						 }
						 else {
							 getLineLocation(CTPOL[PPSP.index_axis].x, CTPOL[PPSP.index_axis].y, CTPOL[PPSP.index_axis + 1].x, CTPOL[PPSP.index_axis + 1].y, temp_path_y, axis_flag, paraO1);//获取交点
						 }
					 }
					 else if (flag_origin_vrc == 1)//指向i+1
					 {
						 if (PPSP.index_axis == (CTPOL.size() - 2)) {
							 getLineLocation(CTPOL[1].x, CTPOL[1].y, CTPOL[CTPOL.size() - 2].x, CTPOL[CTPOL.size() - 2].y, temp_path_y, axis_flag, paraO1);//获取交点
						 }
						 else {
							 getLineLocation(CTPOL[PPSP.index_axis].x, CTPOL[PPSP.index_axis].y, CTPOL[PPSP.index_axis + 1].x, CTPOL[PPSP.index_axis + 1].y, temp_path_y, axis_flag, paraO1);//获取交点
						 }
					 }

					 //getLineLocation(CTPOL[PPSP.index_axis].x, CTPOL[PPSP.index_axis].y, CTPOL[CTPOL.size() - 2].x, CTPOL[CTPOL.size() - 2].y, temp_path_y, axis_flag, paraO1);//获取交点

					 temp_path_x = paraO1.x;
					 temp_path_y = paraO1.y;
					 VPP.push_back(VecPathPlan(paraO1.x, paraO1.y));//填充横向路径点
				 }
				 else
				 {
					 PPSP.vector_flag = 5;
				 }
			 }
		 }
				 }
	 }

	 cout << VPP.size() << endl;
	 //填充点
		double new_x, new_y;
	 for (int j = 0; j < VPP.size(); j++)
	 {
		 //if (j < (VPP.size() - 1))
		 //{
			// double dis;
			// dis=DisCal(VPP[j].x, VPP[j].y, VPP[j+1].x, VPP[j + 1].y);
			// if (dis >= (2 * Plan_length))
			// {
			//	 
			//	 if (VPP[j].x == VPP[j +1 ].x)
			//	 {
			//		 if (VPP[j].y >= VPP[j + 1].y)
			//		 {
			//			 new_x = VPP[j].x;
			//			 new_y = VPP[j].y - Plan_length;
			//		 }
			//		 else
			//		 {
			//			 new_x = VPP[j].x;
			//			 new_y = VPP[j].y + Plan_length;
			//		 }
			//	 }
			//	 else if (VPP[j].y == VPP[j + 1].y)
			//	 {
			//		 if (VPP[j].x >= VPP[j + 1].x)
			//		 {
			//			 new_x = VPP[j].x - Plan_length;
			//			 new_y = VPP[j].y;
			//		 }
			//		 else
			//		 {
			//			 new_x = VPP[j].x + Plan_length;
			//			 new_y = VPP[j].y;
			//		 }
			//	 }	
			//	 VPP.insert(VPP.begin()+j+1,VecPathPlan(new_x, new_y));//填充纵向路径点
			// }

		 //}
		 
		 cout << "第" << j << "个VPP=" << VPP[j].x << "-" << VPP[j].y << endl;
	 }







 }



#endif // ! PATHPLAN_H_INCLUDE
