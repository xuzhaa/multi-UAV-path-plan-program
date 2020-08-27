#pragma once
#ifndef PATHPLAN_H_INCLUDE
#define PATHPLAN_H_INCLUDE

#include <iostream>
#include "VecT.h"
#include "MAM.h"
#include "PJ.h"
#include <vector>

using namespace std;

 void SinglePathPlan(const vector<Vec_tcp>& CTPOL, double *arr_MPP, int axis_flag,double RunLens, PPStartPoint& PPSP);//����·����
 /*
 CTPOL--�з�������������
  arr_MPP--�з����������Χ
  axis_flag--�з�������
  Runlens--ɨ������(ֱ��)
  PPSP--·���滮��ʼ��
 */

 void SinglePathPlan(const vector<Vec_tcp>& CTPOL, double* arr_MPP, int axis_flag, double RunLens, PPStartPoint& PPSP)//����·����
 {
	 vector<VecPathPlan> VPP;//����һ��λ��vector
	 VPP.push_back(VecPathPlan(PPSP.st_x, PPSP.st_y));//���·�����


	 double Plan_length = 1;//�滮����
	 double flag_HV = 0;//�����л�
	 int flag_pathtime = 0;//�滮����
	 double vector_flag_2;//����2   0-�� 1-�� 2-�� 3-��
	 double axis_min_x, axis_max_x, axis_min_y, axis_max_y;//��ȡ���������޷�Χ��Ҳ������������
	 double temp_path_x, temp_path_y;//��ʱ·���滮��
	 double flag_origin_vrc;
	 int num_Z;
	 axis_min_x = arr_MPP[0];
	 axis_max_x = arr_MPP[1];
	 axis_min_y = arr_MPP[2];
	 axis_max_y = arr_MPP[3];

	 
	 //�������۷�����
	num_Z = ceil((axis_max_x - axis_min_x) / RunLens);
	cout <<"RS"<<RunLens<<"minx"<<axis_min_x<<"maxx"<<axis_max_x<< "num_Z=" << num_Z << endl;

	//��ʼλ��Ϊ���
	temp_path_x = PPSP.st_x;
	temp_path_y = PPSP.st_y; 
	//cout << "temp_path_x=" << temp_path_x << endl;

	flag_origin_vrc = PPSP.index_flag;//0-ָ��i 1-ָ��i+1

	 for (int i = 0; i < num_Z; i++)//����ÿһ���۷��� 
	 {
		 if (axis_flag == 0)//�з�����x���ϣ�����y������Ϊ��
		 {
			 //cout << "axis_flag=" << axis_flag << endl;

			 if (CTPOL[PPSP.index_axis].x == arr_MPP[0])//����Сֵ�㣬��Ҫ�����ֵ����
			 {
				 vector_flag_2 = 2;//y��������
			 }
			 else if (CTPOL[PPSP.index_axis].x == arr_MPP[1])//�����ֵ�㣬��Ҫ����Сֵ����
			 {
				 vector_flag_2 = 3;//y��������
			 }
			 //cout << "vector_flag_2=" << vector_flag_2 << endl;

			 if (PPSP.vector_flag == 0)//���Ϲ滮
			 {

				 cout << "num_Z=" << i << endl;
				 LineLocation paraO1;
				 if (PPSP.index_flag == 1)//i+1->i
				 {
					 cout << "PPSP.index_flag=" << PPSP.index_flag << endl;
					 /*temp_path_x = temp_path_x+ flag_pathtime* RunLens;*/
					 //if (PPSP.index_axis == 0)
					 //{
						// getLineLocation(CTPOL[PPSP.index_axis].x, CTPOL[PPSP.index_axis].y, CTPOL[CTPOL.size()-2].x, CTPOL[CTPOL.size() - 2].y, temp_path_x, axis_flag, paraO1);//��ȡ����
					 //}
					 //else
					 //{
					 getLineLocation(CTPOL[PPSP.index_axis - 1].x, CTPOL[PPSP.index_axis - 1].y, CTPOL[PPSP.index_axis - 2].x, CTPOL[PPSP.index_axis - 2].y, temp_path_x, axis_flag, paraO1);//��ȡ����
				 /*}*/
					 cout << "parao1=" << paraO1.x << "-" << paraO1.y << endl;
					VPP.push_back(VecPathPlan(paraO1.x, paraO1.y));//�������·����

					 PPSP.vector_flag = 1;
					 PPSP.index_flag = 0;
					 //cout << "PPSP.index_flag =" << PPSP.index_flag << endl;
					 //cout << "temp_path_x=" << temp_path_x << endl;
					 if ((temp_path_x >= (axis_min_x )) && (temp_path_x <= axis_max_x ))
					 {
						
						 //flag_pathtime = flag_pathtime + 1;
						 if (vector_flag_2 == 2)//�������һ��ֵ
						 {
							 temp_path_x = temp_path_x +   RunLens;
						 }
						 else if (vector_flag_2 == 3)//�������һ��ֵ
						 {
							 temp_path_x = temp_path_x -   RunLens;
						 }
						 getLineLocation(CTPOL[PPSP.index_axis-1].x, CTPOL[PPSP.index_axis-1].y, CTPOL[PPSP.index_axis - 2].x, CTPOL[PPSP.index_axis - 2].y, temp_path_x, axis_flag, paraO1);//��ȡ����
						 cout << "parao1=" << paraO1.x <<"-"<< paraO1.y << endl;
						 temp_path_x = paraO1.x;
						 temp_path_y = paraO1.y;
						 VPP.push_back(VecPathPlan(paraO1.x, paraO1.y));//������·����
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
					 getLineLocation(CTPOL[PPSP.index_axis+1 ].x, CTPOL[PPSP.index_axis+1 ].y, CTPOL[PPSP.index_axis + 2].x, CTPOL[PPSP.index_axis + 2].y, temp_path_x, axis_flag, paraO1);//��ȡ����
					 VPP.push_back(VecPathPlan(paraO1.x, paraO1.y));//�������·����
					cout << "parao1=" << paraO1.x << "-" << paraO1.y << endl;
					 PPSP.vector_flag = 1;
					 PPSP.index_flag = 1;
					 if ((temp_path_x >= (axis_min_x)) && (temp_path_x <= axis_max_x))
					 {
						
						 //flag_pathtime = flag_pathtime + 1;
						 if (vector_flag_2 == 2)//�������һ��ֵ
						 {
							 temp_path_x = temp_path_x +  RunLens;
						 }
						 else if (vector_flag_2 == 3)//�������һ��ֵ
						 {
							 temp_path_x = temp_path_x - RunLens;
						 }
						 getLineLocation(CTPOL[PPSP.index_axis + 1].x, CTPOL[PPSP.index_axis + 1].y, CTPOL[PPSP.index_axis + 2].x, CTPOL[PPSP.index_axis + 2].y, temp_path_x, axis_flag, paraO1);//��ȡ����
						
						 temp_path_x = paraO1.x;
						 temp_path_y = paraO1.y;
						 VPP.push_back(VecPathPlan(paraO1.x, paraO1.y));//������·����
						 cout << "parao1=" << paraO1.x << "-" << paraO1.y << endl;
					 }
					 else
					 {
						 PPSP.vector_flag = 2;
					 }
				 }
			 }
			 else if (PPSP.vector_flag == 1)//���¹滮
			 {
				 cout << "num_Z=" << i << endl;
				 LineLocation paraO1;
				 if (PPSP.index_flag == 0)//i->i+1
				 {
					 getLineLocation(CTPOL[PPSP.index_axis ].x, CTPOL[PPSP.index_axis ].y, CTPOL[PPSP.index_axis + 1].x, CTPOL[PPSP.index_axis +1].y, temp_path_x, axis_flag, paraO1);//��ȡ����
					 VPP.push_back(VecPathPlan(paraO1.x, paraO1.y));//���·�����
					 
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
						 getLineLocation(CTPOL[PPSP.index_axis].x, CTPOL[PPSP.index_axis].y, CTPOL[PPSP.index_axis + 1].x, CTPOL[PPSP.index_axis + 1].y, temp_path_x, axis_flag, paraO1);//��ȡ����
						 temp_path_x = paraO1.x;
						temp_path_y = paraO1.y;
						VPP.push_back(VecPathPlan(temp_path_x, temp_path_y));//������·����
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
						// getLineLocation(CTPOL[PPSP.index_axis-1].x, CTPOL[PPSP.index_axis-1].y, CTPOL[CTPOL.size()-2].x, CTPOL[CTPOL.size() - 2].y, temp_path_x, axis_flag, paraO1);//��ȡ����
					 //}
					 //else
					 //{
						// getLineLocation(CTPOL[PPSP.index_axis - 1].x, CTPOL[PPSP.index_axis - 1].y, CTPOL[PPSP.index_axis - 2].x, CTPOL[PPSP.index_axis - 2].y, temp_path_x, axis_flag, paraO1);//��ȡ����
					 //}

					getLineLocation(CTPOL[PPSP.index_axis ].x, CTPOL[PPSP.index_axis].y, CTPOL[CTPOL.size() - 2].x, CTPOL[CTPOL.size() - 2].y, temp_path_x, axis_flag, paraO1);//��ȡ����
					cout << "paraO1.x=" << paraO1.x << "-" << paraO1.y << endl;
					VPP.push_back(VecPathPlan(paraO1.x, paraO1.y));//���·�����
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
						 getLineLocation(CTPOL[PPSP.index_axis].x, CTPOL[PPSP.index_axis].y, CTPOL[CTPOL.size() - 2].x, CTPOL[CTPOL.size() - 2].y, temp_path_x, axis_flag, paraO1);//��ȡ����

						 temp_path_x = paraO1.x;
						 temp_path_y = paraO1.y;
						 VPP.push_back(VecPathPlan(paraO1.x, paraO1.y));//������·����
					 }
					 else
					 {
						 PPSP.vector_flag = 2;
					 }
				 }
			 }
		 }
		 else if (axis_flag == 1)//�з�����y���ϣ�����x������Ϊ��
		 {
		 //cout << "axis_flag=" << axis_flag << endl;
		 if (CTPOL[PPSP.index_axis].y == arr_MPP[2])//����Сֵ�㣬��Ҫ�����ֵ����
		 {
			 vector_flag_2 = 0;//y��������
		 }
		 else if (CTPOL[PPSP.index_axis].y == arr_MPP[3])//�����ֵ�㣬��Ҫ����Сֵ����
		 {
			 vector_flag_2 = 1;//y��������
		 }
		 //cout << "vector_flag_2=" << vector_flag_2 << endl;


		 if (PPSP.vector_flag == 2)//���ҹ滮
		 {
			 cout << "num_Z=" << i << endl;
			 cout << "���ҹ滮-i+1->i" << endl;
			 LineLocation paraO1;
			 if (PPSP.index_flag == 1)//i+1->i
			 {
				 cout << "PPSP.index_flag=" << PPSP.index_flag << endl;
				 cout << "PPSP.index_axis=" << PPSP.index_axis << endl;
				 if (flag_origin_vrc == 0)//ָ��i
				 {
					 if (PPSP.index_axis == 0) {
						 getLineLocation(CTPOL[PPSP.index_axis].x, CTPOL[PPSP.index_axis].y, CTPOL[CTPOL.size() - 2].x, CTPOL[CTPOL.size() - 2].y, temp_path_y, axis_flag, paraO1);//��ȡ����
					 }
					 else {
						 getLineLocation(CTPOL[PPSP.index_axis].x, CTPOL[PPSP.index_axis].y, CTPOL[PPSP.index_axis - 1].x, CTPOL[PPSP.index_axis - 1].y, temp_path_y, axis_flag, paraO1);//��ȡ����

					 }
				 }
				 else if (flag_origin_vrc == 1)//ָ��i+1
				 {
					 if (PPSP.index_axis == 0)
					 {
						 getLineLocation(CTPOL[PPSP.index_axis].x, CTPOL[PPSP.index_axis].y, CTPOL[CTPOL.size() - 2].x, CTPOL[CTPOL.size() - 2].y, temp_path_y, axis_flag, paraO1);//��ȡ����
					 }
					 else {
						 getLineLocation(CTPOL[PPSP.index_axis - 1].x, CTPOL[PPSP.index_axis - 1].y, CTPOL[PPSP.index_axis - 2].x, CTPOL[PPSP.index_axis - 2].y, temp_path_y, axis_flag, paraO1);//��ȡ����
					 }
				 }

				 cout << "parao1=" << paraO1.x << "-" << paraO1.y << endl;
				 VPP.push_back(VecPathPlan(paraO1.x, paraO1.y));//�������·����

				 PPSP.vector_flag = 3;
				 PPSP.index_flag = 0;
				 cout << "PPSP.index_flag =" << PPSP.index_flag << endl;
				 cout << "temp_path_y=" << temp_path_y << endl;

				 if ((temp_path_y >= (axis_min_y)) && (temp_path_y <= axis_max_y))
				 {
					 if (vector_flag_2 == 0)//�������һ��ֵ
					 {
						 temp_path_y = temp_path_y + RunLens;
					 }
					 else if (vector_flag_2 == 1)//�������һ��ֵ
					 {
						 temp_path_y = temp_path_y - RunLens;
					 }
					 
					 if (flag_origin_vrc == 0)//ָ��i
					 {
						 if (PPSP.index_axis == 0) {
							 getLineLocation(CTPOL[PPSP.index_axis].x, CTPOL[PPSP.index_axis].y, CTPOL[CTPOL.size() - 2].x, CTPOL[CTPOL.size() - 2].y, temp_path_y, axis_flag, paraO1);//��ȡ����
						 }
						 else {
							 getLineLocation(CTPOL[PPSP.index_axis].x, CTPOL[PPSP.index_axis].y, CTPOL[PPSP.index_axis - 1].x, CTPOL[PPSP.index_axis - 1].y, temp_path_y, axis_flag, paraO1);//��ȡ����

						 }
					 }
					 else if (flag_origin_vrc == 1)//ָ��i+1
					 {
						 if (PPSP.index_axis == 0)
						 {
							 getLineLocation(CTPOL[PPSP.index_axis].x, CTPOL[PPSP.index_axis].y, CTPOL[CTPOL.size() - 2].x, CTPOL[CTPOL.size() - 2].y, temp_path_y, axis_flag, paraO1);//��ȡ����
						 }
						 else {
							 getLineLocation(CTPOL[PPSP.index_axis - 1].x, CTPOL[PPSP.index_axis - 1].y, CTPOL[PPSP.index_axis - 2].x, CTPOL[PPSP.index_axis - 2].y, temp_path_y, axis_flag, paraO1);//��ȡ����
						 }
					 }

					 cout << "parao1=" << paraO1.x << "-" << paraO1.y << endl;
					 temp_path_x = paraO1.x;
					 temp_path_y = paraO1.y;
					 VPP.push_back(VecPathPlan(paraO1.x, paraO1.y));//������·����
				 }
				 else
				 {
					 PPSP.vector_flag = 5;
					 cout << "���ҹ滮-i+1->i  PPSP.vector_flag=" << PPSP.vector_flag << endl;
				 }
			 }
			 else if (PPSP.index_flag == 0)//i->i+1
			 {
				 cout << "���ҹ滮-i->i+1" << endl;
				 cout << "PPSP.index_flag=" << PPSP.index_flag << endl;
				 cout << "PPSP.index_axis=" << PPSP.index_axis << endl;

				 if (flag_origin_vrc == 0)//ָ��i
				 {
					 if (PPSP.index_axis == (CTPOL.size() - 2)) {
						 getLineLocation(CTPOL[0].x, CTPOL[0].y, CTPOL[CTPOL.size() - 2].x, CTPOL[CTPOL.size() - 2].y, temp_path_y, axis_flag, paraO1);//��ȡ����
					 }
					 else {
						 getLineLocation(CTPOL[PPSP.index_axis + 1].x, CTPOL[PPSP.index_axis + 1].y, CTPOL[PPSP.index_axis + 2].x, CTPOL[PPSP.index_axis + 2].y, temp_path_y, axis_flag, paraO1);//��ȡ����
					 }
				 }
				 else if (flag_origin_vrc == 1)//ָ��i+1
				 {
					 if (PPSP.index_axis == (CTPOL.size() - 2)){
						 getLineLocation(CTPOL[1].x, CTPOL[1].y, CTPOL[CTPOL.size() - 2].x, CTPOL[CTPOL.size() - 2].y, temp_path_y, axis_flag, paraO1);//��ȡ����
					 }
					 else {
						 getLineLocation(CTPOL[PPSP.index_axis ].x, CTPOL[PPSP.index_axis ].y, CTPOL[PPSP.index_axis +1].x, CTPOL[PPSP.index_axis +1].y, temp_path_y, axis_flag, paraO1);//��ȡ����
					 }
				 }

				 //getLineLocation(CTPOL[PPSP.index_axis + 1].x, CTPOL[PPSP.index_axis + 1].y, CTPOL[PPSP.index_axis + 2].x, CTPOL[PPSP.index_axis + 2].y, temp_path_y, axis_flag, paraO1);//��ȡ����
				 VPP.push_back(VecPathPlan(paraO1.x, paraO1.y));//�������·����
				 cout << "parao1=" << paraO1.x << "-" << paraO1.y << endl;
				 PPSP.vector_flag = 3;
				 PPSP.index_flag = 1;
				 if ((temp_path_y >= (axis_min_y)) && (temp_path_y <= axis_max_y))
				 {
					 if (vector_flag_2 == 0)//�������һ��ֵ
					 {
						 temp_path_y = temp_path_y + RunLens;
					 }
					 else if (vector_flag_2 == 1)//�������һ��ֵ
					 {
						 temp_path_y = temp_path_y - RunLens;
					 }

					 if (flag_origin_vrc == 0)//ָ��i
					 {
						 if (PPSP.index_axis == (CTPOL.size() - 2)) {
							 getLineLocation(CTPOL[0].x, CTPOL[0].y, CTPOL[CTPOL.size() - 2].x, CTPOL[CTPOL.size() - 2].y, temp_path_y, axis_flag, paraO1);//��ȡ����
						 }
						 else {
							 getLineLocation(CTPOL[PPSP.index_axis + 1].x, CTPOL[PPSP.index_axis + 1].y, CTPOL[PPSP.index_axis + 2].x, CTPOL[PPSP.index_axis + 2].y, temp_path_y, axis_flag, paraO1);//��ȡ����
						 }
					 }
					 else if (flag_origin_vrc == 1)//ָ��i+1
					 {
						 if (PPSP.index_axis == (CTPOL.size() - 2)) {
							 getLineLocation(CTPOL[1].x, CTPOL[1].y, CTPOL[CTPOL.size() - 2].x, CTPOL[CTPOL.size() - 2].y, temp_path_y, axis_flag, paraO1);//��ȡ����
						 }
						 else {
							 getLineLocation(CTPOL[PPSP.index_axis].x, CTPOL[PPSP.index_axis].y, CTPOL[PPSP.index_axis + 1].x, CTPOL[PPSP.index_axis + 1].y, temp_path_y, axis_flag, paraO1);//��ȡ����
						 }
					 }

					 //getLineLocation(CTPOL[PPSP.index_axis + 1].x, CTPOL[PPSP.index_axis + 1].y, CTPOL[PPSP.index_axis + 2].x, CTPOL[PPSP.index_axis + 2].y, temp_path_y, axis_flag, paraO1);//��ȡ����

					 temp_path_x = paraO1.x;
					 temp_path_y = paraO1.y;
					 VPP.push_back(VecPathPlan(paraO1.x, paraO1.y));//������·����
					 cout << "parao1=" << paraO1.x << "-" << paraO1.y << endl;
				 }
				 else
				 {
					 PPSP.vector_flag = 5;
				 }
			 }
		 }
		 else if (PPSP.vector_flag == 3)//����滮
		 {

			 cout << "num_Z=" << i << endl;
			 LineLocation paraO1;
			 if (PPSP.index_flag == 0)//i->i+1
			 {
				 cout << "����滮-i->i+1" << endl;
				 cout << "PPSP.index_axis" << PPSP.index_axis << endl;

				 if (flag_origin_vrc == 0)//ָ��i
				 {
					 if (PPSP.index_axis == (CTPOL.size() - 2)){
						 getLineLocation(CTPOL[0].x, CTPOL[0].y, CTPOL[1].x, CTPOL[1].y, temp_path_y, axis_flag, paraO1);//��ȡ����
					 }
					 else{
						 getLineLocation(CTPOL[PPSP.index_axis + 1].x, CTPOL[PPSP.index_axis + 1].y, CTPOL[PPSP.index_axis + 2].x, CTPOL[PPSP.index_axis + 2].y, temp_path_y, axis_flag, paraO1);//��ȡ����
					 }
				 }
				 else if (flag_origin_vrc == 1)//ָ��i+1
				 {
					 if (PPSP.index_axis == (CTPOL.size() - 2)) {
						 getLineLocation(CTPOL[1].x, CTPOL[1].y, CTPOL[CTPOL.size() - 2].x, CTPOL[CTPOL.size() - 2].y, temp_path_y, axis_flag, paraO1);//��ȡ����
					 }
					 else {
						 getLineLocation(CTPOL[PPSP.index_axis].x, CTPOL[PPSP.index_axis].y, CTPOL[PPSP.index_axis + 1].x, CTPOL[PPSP.index_axis + 1].y, temp_path_y, axis_flag, paraO1);//��ȡ����
					 }
				 }

				 //if (PPSP.index_axis == (CTPOL.size() - 2))
				 //{

					// getLineLocation(CTPOL[0].x, CTPOL[0].y, CTPOL[1].x, CTPOL[1].y, temp_path_y, axis_flag, paraO1);//��ȡ����
				 //}
				 //else
				 //{
					// getLineLocation(CTPOL[PPSP.index_axis+1].x, CTPOL[PPSP.index_axis+1].y, CTPOL[PPSP.index_axis + 2].x, CTPOL[PPSP.index_axis +2].y, temp_path_y, axis_flag, paraO1);//��ȡ����
				 //}

				 
				 VPP.push_back(VecPathPlan(paraO1.x, paraO1.y));//���·�����

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

					 if (flag_origin_vrc == 0)//ָ��i
					 {
						 if (PPSP.index_axis == (CTPOL.size() - 2)) {
							 getLineLocation(CTPOL[0].x, CTPOL[0].y, CTPOL[1].x, CTPOL[1].y, temp_path_y, axis_flag, paraO1);//��ȡ����
						 }
						 else {
							 getLineLocation(CTPOL[PPSP.index_axis + 1].x, CTPOL[PPSP.index_axis + 1].y, CTPOL[PPSP.index_axis + 2].x, CTPOL[PPSP.index_axis + 2].y, temp_path_y, axis_flag, paraO1);//��ȡ����
						 }
					 }
					 else if (flag_origin_vrc == 1)//ָ��i+1
					 {
						 if (PPSP.index_axis == (CTPOL.size() - 2)) {
							 getLineLocation(CTPOL[1].x, CTPOL[1].y, CTPOL[CTPOL.size() - 2].x, CTPOL[CTPOL.size() - 2].y, temp_path_y, axis_flag, paraO1);//��ȡ����
						 }
						 else {
							 getLineLocation(CTPOL[PPSP.index_axis].x, CTPOL[PPSP.index_axis].y, CTPOL[PPSP.index_axis + 1].x, CTPOL[PPSP.index_axis + 1].y, temp_path_y, axis_flag, paraO1);//��ȡ����
						 }
					 }

					 //if (PPSP.index_axis == (CTPOL.size() - 2))
					 //{

						// getLineLocation(CTPOL[0].x, CTPOL[0].y, CTPOL[1].x, CTPOL[1].y, temp_path_y, axis_flag, paraO1);//��ȡ����
					 //}
					 //else
					 //{
						// getLineLocation(CTPOL[PPSP.index_axis + 1].x, CTPOL[PPSP.index_axis + 1].y, CTPOL[PPSP.index_axis + 2].x, CTPOL[PPSP.index_axis + 2].y, temp_path_y, axis_flag, paraO1);//��ȡ����
					 //}
					temp_path_x = paraO1.x;
					 temp_path_y = paraO1.y;
					 cout << "11temp_path=" << temp_path_x << "-" << temp_path_y << endl;
					 VPP.push_back(VecPathPlan(temp_path_x, temp_path_y));//������·����
					 cout << "11paraO1.x=" << paraO1.x << "-" << paraO1.y << endl;

				 }
				 else
				 {
					 PPSP.vector_flag = 5;
				 }
			 }
			 else if (PPSP.index_flag == 1)//i+1->i
			 {
				 cout << "����滮-i+1->i" << endl;

				 cout << "PPSP.index_flag=" << PPSP.index_flag << endl;

				 if (flag_origin_vrc == 0)//ָ��i
				 {
					 if (PPSP.index_axis == (CTPOL.size() - 2)) {
						 getLineLocation(CTPOL[PPSP.index_axis].x, CTPOL[PPSP.index_axis].y, CTPOL[PPSP.index_axis - 1].x, CTPOL[PPSP.index_axis - 1].y, temp_path_y, axis_flag, paraO1);//��ȡ����
					 }
					 else {
						 getLineLocation(CTPOL[PPSP.index_axis].x, CTPOL[PPSP.index_axis].y, CTPOL[PPSP.index_axis + 1].x, CTPOL[PPSP.index_axis + 1].y, temp_path_y, axis_flag, paraO1);//��ȡ����
					 }
				 }
				 else if (flag_origin_vrc == 1)//ָ��i+1
				 {
					 if (PPSP.index_axis == (CTPOL.size() - 2)) {
						 getLineLocation(CTPOL[1].x, CTPOL[1].y, CTPOL[CTPOL.size() - 2].x, CTPOL[CTPOL.size() - 2].y, temp_path_y, axis_flag, paraO1);//��ȡ����
					 }
					 else {
						 getLineLocation(CTPOL[PPSP.index_axis].x, CTPOL[PPSP.index_axis].y, CTPOL[PPSP.index_axis + 1].x, CTPOL[PPSP.index_axis + 1].y, temp_path_y, axis_flag, paraO1);//��ȡ����
					 }
				 }


				 //getLineLocation(CTPOL[PPSP.index_axis].x, CTPOL[PPSP.index_axis].y, CTPOL[CTPOL.size() - 2].x, CTPOL[CTPOL.size() - 2].y, temp_path_y, axis_flag, paraO1);//��ȡ����
				 cout << "paraO1.x=" << paraO1.x << "-" << paraO1.y << endl;
				 VPP.push_back(VecPathPlan(paraO1.x, paraO1.y));//���·�����
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

					 if (flag_origin_vrc == 0)//ָ��i
					 {
						 if (PPSP.index_axis == (CTPOL.size() - 2)) {
							 getLineLocation(CTPOL[PPSP.index_axis].x, CTPOL[PPSP.index_axis].y, CTPOL[PPSP.index_axis - 1].x, CTPOL[PPSP.index_axis - 1].y, temp_path_y, axis_flag, paraO1);//��ȡ����
						 }
						 else {
							 getLineLocation(CTPOL[PPSP.index_axis].x, CTPOL[PPSP.index_axis].y, CTPOL[PPSP.index_axis + 1].x, CTPOL[PPSP.index_axis + 1].y, temp_path_y, axis_flag, paraO1);//��ȡ����
						 }
					 }
					 else if (flag_origin_vrc == 1)//ָ��i+1
					 {
						 if (PPSP.index_axis == (CTPOL.size() - 2)) {
							 getLineLocation(CTPOL[1].x, CTPOL[1].y, CTPOL[CTPOL.size() - 2].x, CTPOL[CTPOL.size() - 2].y, temp_path_y, axis_flag, paraO1);//��ȡ����
						 }
						 else {
							 getLineLocation(CTPOL[PPSP.index_axis].x, CTPOL[PPSP.index_axis].y, CTPOL[PPSP.index_axis + 1].x, CTPOL[PPSP.index_axis + 1].y, temp_path_y, axis_flag, paraO1);//��ȡ����
						 }
					 }

					 //getLineLocation(CTPOL[PPSP.index_axis].x, CTPOL[PPSP.index_axis].y, CTPOL[CTPOL.size() - 2].x, CTPOL[CTPOL.size() - 2].y, temp_path_y, axis_flag, paraO1);//��ȡ����

					 temp_path_x = paraO1.x;
					 temp_path_y = paraO1.y;
					 VPP.push_back(VecPathPlan(paraO1.x, paraO1.y));//������·����
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
	 //����
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
			//	 VPP.insert(VPP.begin()+j+1,VecPathPlan(new_x, new_y));//�������·����
			// }

		 //}
		 
		 cout << "��" << j << "��VPP=" << VPP[j].x << "-" << VPP[j].y << endl;
	 }







 }



#endif // ! PATHPLAN_H_INCLUDE
