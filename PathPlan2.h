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
const double lon_nom2 = 102834.74258026089786013677476285;// ����
const double lat_nom2 = 111712.69150641055729984301412873;//γ��

using namespace std;

void SinglePathPlan2(const vector<Vec_tcp>& CTPOL1, double* arr_MPP, int axis_flag, double RunLens, PPStartPoint& PPSP);//����·����
/*
CTPOL--�з�������������
 arr_MPP--�з����������Χ
 axis_flag--�з�������
 Runlens--ɨ������(ֱ��)
 PPSP--·���滮��ʼ��
*/

//void ObsatcleAvoidance(double* PPL, double ox1, double oy1, double duo, vector<VecOA>& VOAPOL);//�ϰ�����
/*
PPL--�滮·����Ϣ
ox1,oy1--�ϰ�����Ϣ
duo--�ϰ��ﴹֱ�ڹ滮·���ľ���
VOAPOL--�ϰ�����·������Ϣ
*/

//double** M_Circle(double cx1, double cy1, double cr, double CNums);
/*
cx1,cy1--Բ��
cr--�뾶
CNums--�滮����Ϣ
MCP--�滮����Ϣָ��
*/

void SinglePathPlan2(const vector<Vec_tcp>& CTPOL1, double* arr_MPP, int axis_flag, double RunLens, PPStartPoint& PPSP)//����·����
{
	cout << "SinglePathPlan2_start" << endl;
	vector<VecPathPlan> VPP;//����һ��λ��vector
	vector<Vec_cp> SCPOL;//����һ�������з�λ��vector
	//VPP.push_back(VecPathPlan(PPSP.st_x, PPSP.st_y));//���·�����


	double Plan_length = 5;//�滮����
	double flag_HV = 0;//�����л�
	int flag_pathtime = 0;//�滮����
	double vector_flag_2;//����2   0-�� 1-�� 2-�� 3-��
	double axis_min_x, axis_max_x, axis_min_y, axis_max_y;//��ȡ���������޷�Χ��Ҳ������������
	double tsx1, tsy1, tsx2, tsy2;//��ʱ����
	double temp_cut_sx, temp_cut_sy;//��ʱ�зֵ�
	double temp_path_x, temp_path_y;//��ʱ·���滮��
	double flag_origin_vrc;
	int num_Z;//���˻����ι滮�۷�����
	double sc_start=0;//��ʼ�������зֵ�
	axis_min_x = arr_MPP[0];
	axis_max_x = arr_MPP[1];
	axis_min_y = arr_MPP[2];
	axis_max_y = arr_MPP[3];





	//�������۷�����
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
	

	//��ʼλ��Ϊ���
	temp_path_x = PPSP.st_x;
	temp_path_y = PPSP.st_y;

	//����ת������
	for (int i = 0; i < num_Z; i++){
		if (axis_flag == 0) {//�з�����x���ϣ�����y������Ϊ��
			if (CTPOL1[PPSP.index_axis].x == arr_MPP[0]){//����Сֵ�㣬��Ҫ�����ֵ����
				sc_start = arr_MPP[0];
				vector_flag_2 = 2;//y��������
			}
			else if (CTPOL1[PPSP.index_axis].x == arr_MPP[1]){//�����ֵ�㣬��Ҫ����Сֵ����
				sc_start = arr_MPP[0];
				vector_flag_2 = 3;//y��������
			}
		}
		else {
			if (CTPOL1[PPSP.index_axis].y == arr_MPP[2]){//����Сֵ�㣬��Ҫ�����ֵ����
				sc_start = arr_MPP[2];
				vector_flag_2 = 0;//x��������
			}
			else if (CTPOL1[PPSP.index_axis].y == arr_MPP[3]){//�����ֵ�㣬��Ҫ����Сֵ����
				sc_start = arr_MPP[2];
				vector_flag_2 = 1;//x��������
			}
		}
	}

	//PPSP.vector_flag  ��������滮���� 0 - �� 1 - �� 2 - �� 3 - ��
	//vector_flag_2  �з��᷽��0-�� 1-�� 2-�� 3-��


	//����·���滮�з� �з�����Ϊ��num_Z
	double* ptr_sca = new double[(int)num_Z];//����һ�������зֵ����������
	for (int i1 = 0; i1 < num_Z; i1++)//�����зֵ�����  num_c1--�з�����
	{
		ptr_sca[i1] = sc_start + (double)i1 * RunLens + (RunLens / 2);  //c_start--�зֵ����  num_sc1-- ��λ�зֳ���

		if (i1 == (num_Z - 1)) {
			if (axis_flag == 0) {
				if ((arr_MPP[1] - ptr_sca[i1 - 1]) < RunLens) {
					ptr_sca[i1] = arr_MPP[1] - (RunLens / 2);  //c_start--�зֵ����  num_sc1-- ��λ�зֳ���
				}
			}
			else {
				if ((arr_MPP[3] - ptr_sca[i1 - 1]) < RunLens) {
					ptr_sca[i1] = arr_MPP[3] - (RunLens / 2);  //c_start--�зֵ����  num_sc1-- ��λ�зֳ���
				}
			}
			
		}
		//cout << "ptr_sca=" << ptr_sca[i1] << endl;


		//if ((sc_start == arr_MPP[0]) || (sc_start == arr_MPP[2])) {
		//	ptr_sca[i1] = sc_start + (double)i1 * RunLens + (RunLens / 2);  //c_start--�зֵ����  num_sc1-- ��λ�зֳ���
		//}
		//else{
		//	ptr_sca[i1] = sc_start - (double)i1 * RunLens - (RunLens / 2);  //c_start--�зֵ����  num_sc1-- ��λ�зֳ���
		//}
	}


	//�������зֵ����ɵ������--���浽Vec_cp��
	for (int i1 = 0; i1 < num_Z; i1++) {//����ÿ���е� num_Z--·��ת������
		
		for (int i2 = 0; i2 < CTPOL1.size()-1; i2++) {//����ÿ������      
			
			if (axis_flag == 0) {//��Ϊx��    
				//cout << "CTPOL1[i2].x=" << CTPOL1[i2].x << endl;
				//cout << "CTPOL1[i2+1].x=" << CTPOL1[i2+1].x << endl;
				//cout << "ptr_sca[i1]=" << ptr_sca[i1];
				if (((CTPOL1[i2].x <= ptr_sca[i1]) && (CTPOL1[i2 + 1].x > ptr_sca[i1])) || ((CTPOL1[i2].x >= ptr_sca[i1]) && (CTPOL1[i2 + 1].x < ptr_sca[i1]))) {//���е��������x����֮��
					LinePara para1;
					tsx1 = CTPOL1[i2].x; tsy1 = CTPOL1[i2].y; tsx2 = CTPOL1[i2 + 1].x; tsy2 = CTPOL1[i2 + 1].y;
					//cout << "tsx1" <<tsx1;
					getLinePara(tsx1, tsy1, tsx2, tsy2, para1);//�����е�ֱ�ߵ�k��b
					temp_cut_sx = ptr_sca[i1]; //ȷ���е��x��
					temp_cut_sy = para1.k * ptr_sca[i1] + para1.b; //ȷ��y��
					SCPOL.push_back(Vec_cp(temp_cut_sx, temp_cut_sy)); //�����е�
					cout << "�е�" << temp_cut_sx << "-" << temp_cut_sy << endl;
				}
			}
			else {//y��          
				if (((CTPOL1[i2].y <= ptr_sca[i1]) && (CTPOL1[i2 + 1].y > ptr_sca[i1])) || ((CTPOL1[i2].y >= ptr_sca[i1]) && (CTPOL1[i2 + 1].y < ptr_sca[i1]))) {//��������֮��     
					LinePara para1;
					tsx1 = CTPOL1[i2].x; tsy1 = CTPOL1[i2].y; tsx2 = CTPOL1[i2 + 1].x; tsy2 = CTPOL1[i2 + 1].y;
					getLinePara(tsx1, tsy1, tsx2, tsy2, para1);//�����е�ֱ�ߵ�k��b
					if (CTPOL1[i2].x == CTPOL1[i2 + 1].x) {//���������㴹ֱ�����ʱ
						temp_cut_sy = ptr_sca[i1]; //�е�y��
						temp_cut_sx = CTPOL1[i2].x; //�е�x��Ϊ�������x������
					}
					else { //����ֱʱ�����������   
						temp_cut_sy = ptr_sca[i1];//ȷ���е��y��
						temp_cut_sx = (ptr_sca[i1] - para1.b) / para1.k;
					}
					SCPOL.push_back(Vec_cp(temp_cut_sx, temp_cut_sy)); // �����е�
					//cout << "�е�" << temp_cut_sx << "-" << temp_cut_sy << endl;                      
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


	
	//���ɵ���·��
	if (axis_flag == 0) {//x��
		cout << "����·������" << endl;
		cout << "vector_flag=" << PPSP.vector_flag << endl;
		cout << "vector_flag_2=" << vector_flag_2 << endl;
		if ((PPSP.vector_flag == 0) && (vector_flag_2 == 3)) {//�������
			for (int j = 0; j < num_Z; j++){
				if ((j % 2) == 0){//����
					cout << (num_Z - 1 - j) + 1 << endl;
					VPP.push_back(VecPathPlan(SCPOL[2 * (num_Z - 1 - j) + 1].x, SCPOL[2 * (num_Z - 1 - j) + 1].y));
					VPP.push_back(VecPathPlan(SCPOL[2 * (num_Z - 1 - j)].x, SCPOL[2 * (num_Z - 1 - j)].y));
				}
				else {//ż��
					VPP.push_back(VecPathPlan(SCPOL[2 * (num_Z - 1 - j)].x, SCPOL[2 * (num_Z - 1 - j)].y));
					VPP.push_back(VecPathPlan(SCPOL[2 * (num_Z - 1 - j) + 1].x, SCPOL[2 * (num_Z - 1 - j) + 1].y));
				}
			}
		}
		else if ((PPSP.vector_flag == 0) && (vector_flag_2 == 2)) {//�������
			for (int j = 0; j < num_Z; j++) {
				if ((j % 2) == 0) {//����
					VPP.push_back(VecPathPlan(SCPOL[2 * (j) + 1].x, SCPOL[2 * (j) + 1].y));
					VPP.push_back(VecPathPlan(SCPOL[2 * (j)].x, SCPOL[2 * (j)].y));
				}
				else {//ż��
					VPP.push_back(VecPathPlan(SCPOL[2 * j].x, SCPOL[2 * j].y));
					VPP.push_back(VecPathPlan(SCPOL[2 * j + 1].x, SCPOL[2 * j + 1].y));
				}
			}
		}
		else if ((PPSP.vector_flag == 1) && (vector_flag_2 == 3)) {//�������
			for (int j = 0; j < num_Z; j++) {
				if ((j % 2) == 0) {//����
					VPP.push_back(VecPathPlan(SCPOL[2 * (num_Z - 1 - j)].x, SCPOL[2 * (num_Z - 1 - j)].y));
					VPP.push_back(VecPathPlan(SCPOL[2 * (num_Z - 1 - j) + 1].x, SCPOL[2 * (num_Z - 1 - j) + 1].y));
				}
				else {//ż��
					VPP.push_back(VecPathPlan(SCPOL[2 * (num_Z - 1 - j) + 1].x, SCPOL[2 * (num_Z - 1 - j) + 1].y));
					VPP.push_back(VecPathPlan(SCPOL[2 * (num_Z - 1 - j)].x, SCPOL[2 * (num_Z - 1 - j)].y));
				}
			}
		}
		else if ((PPSP.vector_flag == 1) && (vector_flag_2 == 2)) {//�������
			cout << "numZ=" << num_Z << "SCPOL.size()=" << SCPOL.size() << endl;
		/*	cout << SCPOL[11].x << endl;*/
			for (int j = 0; j < num_Z; j++) {
				if ((j % 2) == 0) {//����
					VPP.push_back(VecPathPlan(SCPOL[2 * (j)].x, SCPOL[2 * (j)].y));
					VPP.push_back(VecPathPlan(SCPOL[2 * (j)+1].x, SCPOL[2 * (j)+1].y));
				}
				else {//ż��
					cout << "j=" <<j<< endl;
					cout<<j <<":"<< "SCPOL" << SCPOL[2 * j + 1].x << "-" << SCPOL[2 * j + 1].y << endl;

					VPP.push_back(VecPathPlan(SCPOL[2 * j + 1].x, SCPOL[2 * j + 1].y));
					VPP.push_back(VecPathPlan(SCPOL[2 * j].x, SCPOL[2 * j].y));
				}
			}
		}
	}
	else {//y��
		if ((PPSP.vector_flag == 3) && (vector_flag_2 == 0)) {//�������
			for (int j = 0; j < num_Z; j++) {
				if ((j % 2) == 0) {//����
					VPP.push_back(VecPathPlan(SCPOL[2*j + 1].x, SCPOL[2 * j + 1].y));
					VPP.push_back(VecPathPlan(SCPOL[2 * j].x, SCPOL[2 * j].y));
				}
				else {//ż��
					VPP.push_back(VecPathPlan(SCPOL[2 * j].x, SCPOL[2 * j].y));
					VPP.push_back(VecPathPlan(SCPOL[2 * j + 1].x, SCPOL[2 * j + 1].y));
				}
			}
		}
		else if ((PPSP.vector_flag == 2) && (vector_flag_2 == 0)) {//�������
			for (int j = 0; j < num_Z; j++) {
				if ((j % 2) == 0) {//����
					VPP.push_back(VecPathPlan(SCPOL[2 * j].x, SCPOL[2 * j].y));
					VPP.push_back(VecPathPlan(SCPOL[2 * j+1].x, SCPOL[2* j+1].y));
				}
				else {//ż��
					VPP.push_back(VecPathPlan(SCPOL[2 * j+1].x, SCPOL[2 * j+1].y));
					VPP.push_back(VecPathPlan(SCPOL[2 * j ].x, SCPOL[2 * j ].y));
				}
			}
		}
		else if ((PPSP.vector_flag == 3) && (vector_flag_2 == 1)) {//�������
			for (int j = 0; j < num_Z; j++) {
				if ((j % 2) == 0) {//����
					VPP.push_back(VecPathPlan(SCPOL[2 * (num_Z - 1 - j) + 1].x, SCPOL[2 * (num_Z - 1 - j) + 1].y));
					VPP.push_back(VecPathPlan(SCPOL[2 * (num_Z - 1 - j)].x, SCPOL[2 * (num_Z - 1 - j)].y));
				}
				else {//ż��
					VPP.push_back(VecPathPlan(SCPOL[2 * (num_Z - 1 - j) ].x, SCPOL[2 * (num_Z - 1 - j)].y));
					VPP.push_back(VecPathPlan(SCPOL[2 * (num_Z - 1 - j) + 1].x, SCPOL[2 * (num_Z - 1 - j) + 1].y));
				}
			}
		}
		else if ((PPSP.vector_flag == 2) && (vector_flag_2 == 1)) {//�������
			for (int j = 0; j < num_Z; j++) {
				if ((j % 2) == 0) {//����
					VPP.push_back(VecPathPlan(SCPOL[2 * (num_Z - 1 - j) ].x, SCPOL[2 * (num_Z - 1 - j)].y));
					VPP.push_back(VecPathPlan(SCPOL[2 * (num_Z - 1 - j) + 1].x, SCPOL[2 * (num_Z - 1 - j) + 1].y));
				}
				else {//ż��
					VPP.push_back(VecPathPlan(SCPOL[2 * (num_Z - 1 - j) + 1].x, SCPOL[2 * (num_Z - 1 - j) + 1].y));
					VPP.push_back(VecPathPlan(SCPOL[2 * (num_Z - 1 - j)].x, SCPOL[2 * (num_Z - 1 - j)].y));
				}
			}
		}
	}




	////������ϳ���
	//vector<VecOA> VOAPOL;
	//double* PPL[6];
	//double ux1, uy1, ux1_next, uy1_next, vec_path_flag, vec_axis_flag;

	for (int j = 0; j < VPP.size(); j++) {
		//PPL[0]=

		//	ux1 = PPL[0];//i��������Ϣ
		//double uy1 = PPL[1];
		//double ux1_next = PPL[2];//i+1��������Ϣ
		//double uy1_next = PPL[3];
		//double vec_path_flag = PPL[4]; // ��ǰi���ƶ����� 0 - �� 1 - �� 2 - �� 3 - ��
		//double vec_axis_flag = PPL[5]; //��������滮���� 0 - �� 1 - �� 2 - �� 3 - ��


		//ObsatcleAvoidance(PPL,  axis,  ox1,  oy1,  oraidus,  duo, vector<VecOA>& VOAPOL);

	}

	




	//cout << VPP.size() << endl;
	//����
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
		//   	 VPP.insert(VPP.begin()+j+1,VecPathPlan(new_x, new_y));//�������·����
		//    }

		//}

		//cout << "��" << j << "��VPP=" << VPP[j].x << "-" << VPP[j].y << endl;


	

		cout <<fixed << setprecision(8) << (VPP[j].x/ lat_nom2 )<<",";

	}
	cout << endl;

	for (int j = 0; j < VPP.size(); j++)
	{
		cout << fixed << setprecision(8) << (VPP[j].y/lon_nom2) << ",";

	}
	cout << endl;



	delete[] ptr_sca;//ɾ��new�Ķ���


}



//void ObsatcleAvoidance(double *PPL,double axis, double ox1, double oy1, double oraidus, double duo, vector<VecOA>& VOAPOL)
//{
//	double ux1 = PPL[0];//i��������Ϣ
//	double uy1 = PPL[1];
//	double ux1_next = PPL[2];//i+1��������Ϣ
//	double uy1_next = PPL[3];
//	double vec_path_flag = PPL[4]; // ��ǰi���ƶ����� 0 - �� 1 - �� 2 - �� 3 - ��
//	double vec_axis_flag=PPL[5]; //��������滮���� 0 - �� 1 - �� 2 - �� 3 - ��
//
//	double MinSafeRadius = oraidus + 1;//�����С��ȫ����
//	int CNums = 8;//�зֵ�
//
//	double** MCP = M_Circle(ox1, oy1, MinSafeRadius, CNums);//��ȡ����
//
//	//������Ϣ��ȡ
//	if (axis == 0) {//x��
//		if (vec_axis_flag == 3) {//x�ᡢ������ɨ��
//			for (int j = 0; j < CNums; j++) {
//				if (MCP[0][j] <= ux1) {//���ڸ�����ߵ�ȫ����ֵ
//					if (vec_path_flag == 0) {//�����ƶ�
//						VOAPOL.push_back(VecOA(MCP[0][CNums - 1 - j], MCP[1][CNums - 1 - j]));//���ڸ�����ߵĵ���µ���ȫ����ֵ
//					}
//					else {//�����ƶ�
//						VOAPOL.push_back(VecOA(MCP[0][j], MCP[1][j]));//���ڸ�����ߵĵ���ϵ���ȫ����ֵ
//					}
//				}
//			}
//		}
//		else if(vec_axis_flag == 2) {//x�ᡢ���ҷ���ɨ��
//			for (int j = 0; j < CNums; j++) {
//				if (MCP[0][j] >= ux1) {//���ڸ�����ߵ�ȫ����ֵ
//					if (vec_path_flag == 0) {//�����ƶ�
//						VOAPOL.push_back(VecOA(MCP[0][CNums - 1 - j], MCP[1][CNums - 1 - j]));//���ڸ�����ߵĵ���µ���ȫ����ֵ
//					}
//					else {//�����ƶ�
//						VOAPOL.push_back(VecOA(MCP[0][j], MCP[1][j]));//���ڸ�����ߵĵ���ϵ���ȫ����ֵ
//					}
//				}
//			}
//
//		}
//	}
//	//else {//y��
//
//	//}
//
//
//	//�ڴ����
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
