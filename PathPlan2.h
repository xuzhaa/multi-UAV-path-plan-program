#pragma once
#pragma once
#ifndef PATHPLAN2_H_INCLUDE
#define PATHPLAN2_H_INCLUDE

#include <iostream>
#include "VecT.h"
#include "MAM.h"
#include "PJ.h"
#include <vector>
#include <math.h>
const double lon_nom2 = 102834.74258026089786013677476285;// 经度
const double lat_nom2 = 111712.69150641055729984301412873;//纬度

using namespace std;

void SinglePathPlan2(const vector<Vec_tcp>& CTPOL1, double* arr_MPP, int axis_flag, double RunLens, PPStartPoint& PPSP);//生成路径点
/*
CTPOL--切分子区域特征点
 arr_MPP--切分子区域最大范围
 axis_flag--切分坐标轴
 Runlens--扫描区域(直径)
 PPSP--路径规划起始点
*/

//void ObsatcleAvoidance(double* PPL, double ox1, double oy1, double duo, vector<VecOA>& VOAPOL);//障碍物规避
/*
PPL--规划路径信息
ox1,oy1--障碍物信息
duo--障碍物垂直于规划路径的距离
VOAPOL--障碍物规避路径点信息
*/

//double** M_Circle(double cx1, double cy1, double cr, double CNums);
/*
cx1,cy1--圆心
cr--半径
CNums--规划点信息
MCP--规划点信息指针
*/

void SinglePathPlan2(const vector<Vec_tcp>& CTPOL1, double* arr_MPP, int axis_flag, double RunLens, PPStartPoint& PPSP)//生成路径点
{
	cout << "SinglePathPlan2_start" << endl;
	vector<VecPathPlan> VPP;//创建一个位置vector
	vector<Vec_cp> SCPOL;//创建一个单机切分位置vector
	//VPP.push_back(VecPathPlan(PPSP.st_x, PPSP.st_y));//填充路径起点


	double Plan_length = 5;//规划步长
	double flag_HV = 0;//横纵切换
	int flag_pathtime = 0;//规划次数
	double vector_flag_2;//方向2   0-上 1-下 2-右 3-左
	double axis_min_x, axis_max_x, axis_min_y, axis_max_y;//获取划分区域极限范围，也即划分子区域
	double tsx1, tsy1, tsx2, tsy2;//临时变量
	double temp_cut_sx, temp_cut_sy;//临时切分点
	double temp_path_x, temp_path_y;//临时路径规划点
	double flag_origin_vrc;
	int num_Z;//无人机单次规划折返次数
	double sc_start=0;//起始坐标轴切分点
	axis_min_x = arr_MPP[0];
	axis_max_x = arr_MPP[1];
	axis_min_y = arr_MPP[2];
	axis_max_y = arr_MPP[3];





	//计算总折返次数
	if (axis_flag == 0) {
		/*sc_start= arr_MPP[0];*/
		num_Z = ceil((axis_max_x - axis_min_x) / RunLens);
		cout << "RS-" << RunLens << "minx-" << axis_min_x << "maxx-" << axis_max_x << "-num_Z=" << num_Z << endl;
	}
	else {
		/*sc_start = arr_MPP[2];*/
		num_Z = ceil((axis_max_y - axis_min_y) / RunLens);
		cout << "RS-" << RunLens << "miny-" << axis_min_y << "maxy-" << axis_max_y << "-num_Z=" << num_Z << endl;
	}
	

	//初始位置为起点
	temp_path_x = PPSP.st_x;
	temp_path_y = PPSP.st_y;

	//计算转换走向
	for (int i = 0; i < num_Z; i++){
		if (axis_flag == 0) {//切分轴在x轴上，则以y轴衍生为主
			if (CTPOL1[PPSP.index_axis].x == arr_MPP[0]){//在最小值点，需要往最大值点走
				sc_start = arr_MPP[0];
				vector_flag_2 = 2;//y轴向右走
			}
			else if (CTPOL1[PPSP.index_axis].x == arr_MPP[1]){//在最大值点，需要往最小值点走
				sc_start = arr_MPP[0];
				vector_flag_2 = 3;//y轴向左走
			}
		}
		else {
			if (CTPOL1[PPSP.index_axis].y == arr_MPP[2]){//在最小值点，需要往最大值点走
				sc_start = arr_MPP[2];
				vector_flag_2 = 0;//x轴向右走
			}
			else if (CTPOL1[PPSP.index_axis].y == arr_MPP[3]){//在最大值点，需要往最小值点走
				sc_start = arr_MPP[2];
				vector_flag_2 = 1;//x轴向左走
			}
		}
	}

	//PPSP.vector_flag  起点期望规划方向 0 - 上 1 - 下 2 - 右 3 - 左
	//vector_flag_2  切分轴方向0-上 1-下 2-右 3-左


	//进行路径规划切分 切分数量为：num_Z
	double* ptr_sca = new double[(int)num_Z];//创建一个包含切分点坐标的数组
	for (int i1 = 0; i1 < num_Z; i1++)//计算切分点坐标  num_c1--切分数量
	{
		ptr_sca[i1] = sc_start + (double)i1 * RunLens + (RunLens / 2);  //c_start--切分点起点  num_sc1-- 单位切分长度

		if (i1 == (num_Z - 1)) {
			if (axis_flag == 0) {
				if ((arr_MPP[1] - ptr_sca[i1 - 1]) < RunLens) {
					ptr_sca[i1] = arr_MPP[1] - (RunLens / 2);  //c_start--切分点起点  num_sc1-- 单位切分长度
				}
			}
			else {
				if ((arr_MPP[3] - ptr_sca[i1 - 1]) < RunLens) {
					ptr_sca[i1] = arr_MPP[3] - (RunLens / 2);  //c_start--切分点起点  num_sc1-- 单位切分长度
				}
			}
			
		}
		//cout << "ptr_sca=" << ptr_sca[i1] << endl;


		//if ((sc_start == arr_MPP[0]) || (sc_start == arr_MPP[2])) {
		//	ptr_sca[i1] = sc_start + (double)i1 * RunLens + (RunLens / 2);  //c_start--切分点起点  num_sc1-- 单位切分长度
		//}
		//else{
		//	ptr_sca[i1] = sc_start - (double)i1 * RunLens - (RunLens / 2);  //c_start--切分点起点  num_sc1-- 单位切分长度
		//}
	}


	//计算由切分点生成的坐标点--保存到Vec_cp中
	for (int i1 = 0; i1 < num_Z; i1++) {//遍历每个切点 num_Z--路径转换次数
		
		for (int i2 = 0; i2 < CTPOL1.size()-1; i2++) {//遍历每个顶点      
			
			if (axis_flag == 0) {//若为x轴    
				//cout << "CTPOL1[i2].x=" << CTPOL1[i2].x << endl;
				//cout << "CTPOL1[i2+1].x=" << CTPOL1[i2+1].x << endl;
				//cout << "ptr_sca[i1]=" << ptr_sca[i1];
				if (((CTPOL1[i2].x <= ptr_sca[i1]) && (CTPOL1[i2 + 1].x > ptr_sca[i1])) || ((CTPOL1[i2].x >= ptr_sca[i1]) && (CTPOL1[i2 + 1].x < ptr_sca[i1]))) {//在切点两个点的x坐标之间
					LinePara para1;
					tsx1 = CTPOL1[i2].x; tsy1 = CTPOL1[i2].y; tsx2 = CTPOL1[i2 + 1].x; tsy2 = CTPOL1[i2 + 1].y;
					//cout << "tsx1" <<tsx1;
					getLinePara(tsx1, tsy1, tsx2, tsy2, para1);//计算切点直线的k和b
					temp_cut_sx = ptr_sca[i1]; //确定切点的x轴
					temp_cut_sy = para1.k * ptr_sca[i1] + para1.b; //确定y轴
					SCPOL.push_back(Vec_cp(temp_cut_sx, temp_cut_sy)); //保存切点
					cout << "切点" << temp_cut_sx << "-" << temp_cut_sy << endl;
				}
			}
			else {//y轴          
				if (((CTPOL1[i2].y <= ptr_sca[i1]) && (CTPOL1[i2 + 1].y > ptr_sca[i1])) || ((CTPOL1[i2].y >= ptr_sca[i1]) && (CTPOL1[i2 + 1].y < ptr_sca[i1]))) {//在两个点之间     
					LinePara para1;
					tsx1 = CTPOL1[i2].x; tsy1 = CTPOL1[i2].y; tsx2 = CTPOL1[i2 + 1].x; tsy2 = CTPOL1[i2 + 1].y;
					getLinePara(tsx1, tsy1, tsx2, tsy2, para1);//计算切点直线的k和b
					if (CTPOL1[i2].x == CTPOL1[i2 + 1].x) {//出现两个点垂直的情况时
						temp_cut_sy = ptr_sca[i1]; //切点y轴
						temp_cut_sx = CTPOL1[i2].x; //切点x轴为这两点的x轴坐标
					}
					else { //不垂直时，按常规计算   
						temp_cut_sy = ptr_sca[i1];//确定切点的y轴
						temp_cut_sx = (ptr_sca[i1] - para1.b) / para1.k;
					}
					SCPOL.push_back(Vec_cp(temp_cut_sx, temp_cut_sy)); // 保存切点
					//cout << "切点" << temp_cut_sx << "-" << temp_cut_sy << endl;                      
				}
			}
		}
	}

	//cout << SCPOL.size() << endl;
	//for (int i = 0; i < SCPOL.size(); i++)
	//{
	//	cout << "SCPOL=" << SCPOL[i].x << "-" << SCPOL[i].y << endl;
	//}
	//cout << "num_Z=" << num_Z << endl;


	
	//生成单机路径
	if (axis_flag == 0) {//x轴
		cout << "单机路径生成" << endl;
		cout << "vector_flag=" << PPSP.vector_flag << endl;
		cout << "vector_flag_2=" << vector_flag_2 << endl;
		if ((PPSP.vector_flag == 0) && (vector_flag_2 == 3)) {//右下起点
			for (int j = 0; j < num_Z; j++){
				if ((j % 2) == 0){//奇数
					cout << (num_Z - 1 - j) + 1 << endl;
					VPP.push_back(VecPathPlan(SCPOL[2 * (num_Z - 1 - j) + 1].x, SCPOL[2 * (num_Z - 1 - j) + 1].y));
					VPP.push_back(VecPathPlan(SCPOL[2 * (num_Z - 1 - j)].x, SCPOL[2 * (num_Z - 1 - j)].y));
				}
				else {//偶数
					VPP.push_back(VecPathPlan(SCPOL[2 * (num_Z - 1 - j)].x, SCPOL[2 * (num_Z - 1 - j)].y));
					VPP.push_back(VecPathPlan(SCPOL[2 * (num_Z - 1 - j) + 1].x, SCPOL[2 * (num_Z - 1 - j) + 1].y));
				}
			}
		}
		else if ((PPSP.vector_flag == 0) && (vector_flag_2 == 2)) {//左下起点
			for (int j = 0; j < num_Z; j++) {
				if ((j % 2) == 0) {//奇数
					VPP.push_back(VecPathPlan(SCPOL[2 * (j) + 1].x, SCPOL[2 * (j) + 1].y));
					VPP.push_back(VecPathPlan(SCPOL[2 * (j)].x, SCPOL[2 * (j)].y));
				}
				else {//偶数
					VPP.push_back(VecPathPlan(SCPOL[2 * j].x, SCPOL[2 * j].y));
					VPP.push_back(VecPathPlan(SCPOL[2 * j + 1].x, SCPOL[2 * j + 1].y));
				}
			}
		}
		else if ((PPSP.vector_flag == 1) && (vector_flag_2 == 3)) {//右上起点
			for (int j = 0; j < num_Z; j++) {
				if ((j % 2) == 0) {//奇数
					VPP.push_back(VecPathPlan(SCPOL[2 * (num_Z - 1 - j)].x, SCPOL[2 * (num_Z - 1 - j)].y));
					VPP.push_back(VecPathPlan(SCPOL[2 * (num_Z - 1 - j) + 1].x, SCPOL[2 * (num_Z - 1 - j) + 1].y));
				}
				else {//偶数
					VPP.push_back(VecPathPlan(SCPOL[2 * (num_Z - 1 - j) + 1].x, SCPOL[2 * (num_Z - 1 - j) + 1].y));
					VPP.push_back(VecPathPlan(SCPOL[2 * (num_Z - 1 - j)].x, SCPOL[2 * (num_Z - 1 - j)].y));
				}
			}
		}
		else if ((PPSP.vector_flag == 1) && (vector_flag_2 == 2)) {//左上起点
			cout << "numZ=" << num_Z << "SCPOL.size()=" << SCPOL.size() << endl;
		/*	cout << SCPOL[11].x << endl;*/
			for (int j = 0; j < num_Z; j++) {
				if ((j % 2) == 0) {//奇数
					VPP.push_back(VecPathPlan(SCPOL[2 * (j)].x, SCPOL[2 * (j)].y));
					VPP.push_back(VecPathPlan(SCPOL[2 * (j)+1].x, SCPOL[2 * (j)+1].y));
				}
				else {//偶数
					cout << "j=" <<j<< endl;
					cout<<j <<":"<< "SCPOL" << SCPOL[2 * j + 1].x << "-" << SCPOL[2 * j + 1].y << endl;

					VPP.push_back(VecPathPlan(SCPOL[2 * j + 1].x, SCPOL[2 * j + 1].y));
					VPP.push_back(VecPathPlan(SCPOL[2 * j].x, SCPOL[2 * j].y));
				}
			}
		}
	}
	else {//y轴
		if ((PPSP.vector_flag == 3) && (vector_flag_2 == 0)) {//右下起点
			for (int j = 0; j < num_Z; j++) {
				if ((j % 2) == 0) {//奇数
					VPP.push_back(VecPathPlan(SCPOL[2*j + 1].x, SCPOL[2 * j + 1].y));
					VPP.push_back(VecPathPlan(SCPOL[2 * j].x, SCPOL[2 * j].y));
				}
				else {//偶数
					VPP.push_back(VecPathPlan(SCPOL[2 * j].x, SCPOL[2 * j].y));
					VPP.push_back(VecPathPlan(SCPOL[2 * j + 1].x, SCPOL[2 * j + 1].y));
				}
			}
		}
		else if ((PPSP.vector_flag == 2) && (vector_flag_2 == 0)) {//左下起点
			for (int j = 0; j < num_Z; j++) {
				if ((j % 2) == 0) {//奇数
					VPP.push_back(VecPathPlan(SCPOL[2 * j].x, SCPOL[2 * j].y));
					VPP.push_back(VecPathPlan(SCPOL[2 * j+1].x, SCPOL[2* j+1].y));
				}
				else {//偶数
					VPP.push_back(VecPathPlan(SCPOL[2 * j+1].x, SCPOL[2 * j+1].y));
					VPP.push_back(VecPathPlan(SCPOL[2 * j ].x, SCPOL[2 * j ].y));
				}
			}
		}
		else if ((PPSP.vector_flag == 3) && (vector_flag_2 == 1)) {//右上起点
			for (int j = 0; j < num_Z; j++) {
				if ((j % 2) == 0) {//奇数
					VPP.push_back(VecPathPlan(SCPOL[2 * (num_Z - 1 - j) + 1].x, SCPOL[2 * (num_Z - 1 - j) + 1].y));
					VPP.push_back(VecPathPlan(SCPOL[2 * (num_Z - 1 - j)].x, SCPOL[2 * (num_Z - 1 - j)].y));
				}
				else {//偶数
					VPP.push_back(VecPathPlan(SCPOL[2 * (num_Z - 1 - j) ].x, SCPOL[2 * (num_Z - 1 - j)].y));
					VPP.push_back(VecPathPlan(SCPOL[2 * (num_Z - 1 - j) + 1].x, SCPOL[2 * (num_Z - 1 - j) + 1].y));
				}
			}
		}
		else if ((PPSP.vector_flag == 2) && (vector_flag_2 == 1)) {//左上起点
			for (int j = 0; j < num_Z; j++) {
				if ((j % 2) == 0) {//奇数
					VPP.push_back(VecPathPlan(SCPOL[2 * (num_Z - 1 - j) ].x, SCPOL[2 * (num_Z - 1 - j)].y));
					VPP.push_back(VecPathPlan(SCPOL[2 * (num_Z - 1 - j) + 1].x, SCPOL[2 * (num_Z - 1 - j) + 1].y));
				}
				else {//偶数
					VPP.push_back(VecPathPlan(SCPOL[2 * (num_Z - 1 - j) + 1].x, SCPOL[2 * (num_Z - 1 - j) + 1].y));
					VPP.push_back(VecPathPlan(SCPOL[2 * (num_Z - 1 - j)].x, SCPOL[2 * (num_Z - 1 - j)].y));
				}
			}
		}
	}




	////计算避障程序
	//vector<VecOA> VOAPOL;
	//double* PPL[6];
	//double ux1, uy1, ux1_next, uy1_next, vec_path_flag, vec_axis_flag;

	for (int j = 0; j < VPP.size(); j++) {
		//PPL[0]=

		//	ux1 = PPL[0];//i点特征信息
		//double uy1 = PPL[1];
		//double ux1_next = PPL[2];//i+1点特征信息
		//double uy1_next = PPL[3];
		//double vec_path_flag = PPL[4]; // 当前i点移动方向 0 - 上 1 - 下 2 - 右 3 - 左
		//double vec_axis_flag = PPL[5]; //起点期望规划方向 0 - 上 1 - 下 2 - 右 3 - 左


		//ObsatcleAvoidance(PPL,  axis,  ox1,  oy1,  oraidus,  duo, vector<VecOA>& VOAPOL);

	}

	




	//cout << VPP.size() << endl;
	//填充点
	double new_x, new_y;
	for (int j = 0; j < VPP.size(); j++)
	{
		//if (j < (VPP.size() - 1))
		//{
		//    double dis;
		//    dis=DisCal(VPP[j].x, VPP[j].y, VPP[j+1].x, VPP[j + 1].y);
		//    if (dis >= (2 * Plan_length))
		//    {
		//   		 if (VPP[j].x == VPP[j +1 ].x)
		//   		 {
		//   			 if (VPP[j].y >= VPP[j + 1].y)
		//   			 {
		//   				 new_x = VPP[j].x;
		//   				 new_y = VPP[j].y - Plan_length;
		//   			 }
		//   			 else
		//   			 {
		//   				 new_x = VPP[j].x;
		//   				 new_y = VPP[j].y + Plan_length;
		//   			 }
		//   		 }
		//   		 else if (VPP[j].y == VPP[j + 1].y)
		//   		 {
		//   			 if (VPP[j].x >= VPP[j + 1].x)
		//   			 {
		//   				 new_x = VPP[j].x - Plan_length;
		//   				 new_y = VPP[j].y;
		//   			 }
		//   			 else
		//   			 {
		//   				 new_x = VPP[j].x + Plan_length;
		//   				 new_y = VPP[j].y;
		//   			 }
		//   		 }	
		//   	 VPP.insert(VPP.begin()+j+1,VecPathPlan(new_x, new_y));//填充纵向路径点
		//    }

		//}

		//cout << "第" << j << "个VPP=" << VPP[j].x << "-" << VPP[j].y << endl;


	

		cout <<fixed << setprecision(8) << (VPP[j].x/ lat_nom2 )<<",";

	}
	cout << endl;

	for (int j = 0; j < VPP.size(); j++)
	{
		cout << fixed << setprecision(8) << (VPP[j].y/lon_nom2) << ",";

	}
	cout << endl;



	delete[] ptr_sca;//删除new的对象


}



//void ObsatcleAvoidance(double *PPL,double axis, double ox1, double oy1, double oraidus, double duo, vector<VecOA>& VOAPOL)
//{
//	double ux1 = PPL[0];//i点特征信息
//	double uy1 = PPL[1];
//	double ux1_next = PPL[2];//i+1点特征信息
//	double uy1_next = PPL[3];
//	double vec_path_flag = PPL[4]; // 当前i点移动方向 0 - 上 1 - 下 2 - 右 3 - 左
//	double vec_axis_flag=PPL[5]; //起点期望规划方向 0 - 上 1 - 下 2 - 右 3 - 左
//
//	double MinSafeRadius = oraidus + 1;//规避最小安全距离
//	int CNums = 8;//切分点
//
//	double** MCP = M_Circle(ox1, oy1, MinSafeRadius, CNums);//获取坐标
//
//	//避障信息获取
//	if (axis == 0) {//x轴
//		if (vec_axis_flag == 3) {//x轴、向左方向扫描
//			for (int j = 0; j < CNums; j++) {
//				if (MCP[0][j] <= ux1) {//将在该线左边的全部赋值
//					if (vec_path_flag == 0) {//向上移动
//						VOAPOL.push_back(VecOA(MCP[0][CNums - 1 - j], MCP[1][CNums - 1 - j]));//将在该线左边的点从下到上全部赋值
//					}
//					else {//向下移动
//						VOAPOL.push_back(VecOA(MCP[0][j], MCP[1][j]));//将在该线左边的点从上到下全部赋值
//					}
//				}
//			}
//		}
//		else if(vec_axis_flag == 2) {//x轴、向右方向扫描
//			for (int j = 0; j < CNums; j++) {
//				if (MCP[0][j] >= ux1) {//将在该线左边的全部赋值
//					if (vec_path_flag == 0) {//向上移动
//						VOAPOL.push_back(VecOA(MCP[0][CNums - 1 - j], MCP[1][CNums - 1 - j]));//将在该线左边的点从下到上全部赋值
//					}
//					else {//向下移动
//						VOAPOL.push_back(VecOA(MCP[0][j], MCP[1][j]));//将在该线左边的点从上到下全部赋值
//					}
//				}
//			}
//
//		}
//	}
//	//else {//y轴
//
//	//}
//
//
//	//内存清空
//	for (int i = 0; i < 2; i++)
//	free(MCP[i]);
//	free(MCP);
//}





//double** M_Circle(double cx1, double cy1, double cr, double CNums)
//{
//	double** data_c;
//	double thi = (2 * MATH_PI) / CNums;
//	data_c = (double **)malloc(2 * sizeof(int*));
//	for (int i = 0; i < 2; i++)
//		data_c[i] = (double*)malloc(CNums * sizeof(double));
//	for (int i = 0; i < CNums; i++){
//		data_c[0][i] = cr * cos(i*thi);//x
//		data_c[1][i]= cr * cos(i * thi);//y
//	}
//	return data_c;
//}




#endif // ! PATHPLAN2_H_INCLUDE
