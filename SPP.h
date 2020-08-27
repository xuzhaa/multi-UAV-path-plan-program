#pragma once
#ifndef SPP_H_INCLUDE
#define SPP_H_INCLUDE

#include "VecT.h"
#include "MAM.h"
#include "PJ.h"
#include <vector>
#include "PathPlan.h"
#include "PathPlan2.h"

const double lon_nom3 = 102834.74258026089786013677476285;// ����
const double lat_nom3 = 111712.69150641055729984301412873;//γ��

using namespace std;

void PathPlanStartPoint(const vector<Vec_tcp>& CTPOL, int axis_flag, double* StartPosition, double RunLens);//����·���滮���
/*
CTPOL         -�������򶥵�
axis_flag     -���򻮷ֱ�־ ����x��ory��
StartPosition -���˻���ɵ�
RunLens       -ɨ�賤��
*/

void SPL(double arr_MM[], double dx1, double dy1, double axis_flag, double Rlens, Offset_SP& OSP);//�����������ƫ��

void PathPlanStartPoint(const vector<Vec_tcp>& CTPOL, int axis_flag, double* StartPosition, double RunLens)
{
	double Min_dis, Min_dis_next, Path_x1, Path_y1, Min_dis_index;//��С���룬�������꣬�����±�
	double CrDis, Min_CrDis_x, Min_CrDis_y, Min_CrDis_index;//��Ծ���
	double PathLineStart_x=0, PathLineStart_y=0;//��ɵ�
	double *ptr_CrDis=new double[CTPOL.size() - 1];//��������ڱ߳���С
	double* ptr_Min_dis = new double[CTPOL.size() - 1];//������򶥵�����ɵ����
	double arr_MPP[8];//���ص�xy��Сֵ
	double o_index;


	//��ʼ��-����һ������Ϊ��Сֵ��
	double temp_dis = DisCal(CTPOL[0].x, CTPOL[0].y, StartPosition[0], StartPosition[1]);//����һ������Ϊ��Сֵ��

	//��ʼ��-����һ������Ϊ���λ����Сֵ��  ���λ��Ϊ�õ������˻���ɵ����
	double temp_CrDis = 0;//����һ������Ϊ��Сֵ��
	double temp_MinCrDis = INT_MAX;//��С���λ�õ�

	getMaxAndMinPathPoint(CTPOL, arr_MPP);//��ȡPOL�е���С�����ֵ
	double ptr_MMP[8];//��ȡ�����Сֵ�Լ���ֵ��
	for (int i = 0; i < 8; i++)
	{
		ptr_MMP[i] = arr_MPP[i];
	}


	PPStartPoint PPSP;//�ṹ��
	
	//���Ҿ���StartPosition����ĵ㼰��������������֮����Ծ���
	for (int i = 0; i < CTPOL.size() - 1; i++){
		//����i���Լ�i+1�������ɵ�ľ���
		Min_dis = DisCal(CTPOL[i].x, CTPOL[i].y, StartPosition[0], StartPosition[1]);//������ɵ��뻮�����������ľ���
		Min_dis_next = DisCal(CTPOL[i + 1].x, CTPOL[i + 1].y, StartPosition[0], StartPosition[1]);//i+1�����������ɵ����

		//����i���Լ�i+1��֮��ľ���
		CrDis = DisCal(CTPOL[i].x, CTPOL[i].y, CTPOL[i + 1].x, CTPOL[i + 1].y);//���������ڸ�����֮�����Ծ���

		//x��Ϊ���
		if (axis_flag == 0){
			if ((CrDis >= temp_CrDis) && ((CTPOL[i].x - CTPOL[i + 1].x) == 0)&& ((Min_dis <= temp_MinCrDis) || (Min_dis_next <= temp_MinCrDis))){//��ֱ��x��������ɵ�
				temp_CrDis = CrDis;//ʵС����
				if (Min_dis > Min_dis_next){//i+1�����
					temp_MinCrDis = Min_dis_next;
					Offset_SP OSP;
					SPL(ptr_MMP, CTPOL[i + 1].x, CTPOL[i + 1].y, axis_flag, RunLens,OSP);//·�������֮Ϊ��׼
					Path_x1 = OSP.offset1;
					o_index = OSP.index_num;

					LineLocation paraL1;
					getLineLocation(CTPOL[i + 1].x, CTPOL[i + 1].y, CTPOL[i + 2].x, CTPOL[i + 2].y, Path_x1, axis_flag, paraL1);//��ȡ��ʼ��

					PPSP.st_x = paraL1.x;
					PPSP.st_y = paraL1.y;
					PPSP.index_axis = i + 1;
					PPSP.index_flag = 1;

					/*ȷ������i+1>i*/
					if ((CTPOL[i+1].y - CTPOL[i].y)>0){//i+1������i���� ���·�
						PPSP.vector_flag = 1;
					}
					else{
						PPSP.vector_flag = 0;
					}

					PathLineStart_x = PPSP.st_x;
					PathLineStart_y = PPSP.st_y;
				}
				else{//i�����
					temp_MinCrDis = Min_dis;
					Offset_SP OSP;
					SPL(ptr_MMP, CTPOL[i].x, CTPOL[i].y, axis_flag, RunLens, OSP);//·�������֮Ϊ��׼
					Path_x1=OSP.offset1;
					o_index = OSP.index_num;

					LineLocation paraL1;
					if (i == 0){//i=0ʱ�ɵ�һ��������һ���������ȷ��
						getLineLocation(CTPOL[i].x, CTPOL[i].y, CTPOL[CTPOL.size() - 2].x, CTPOL[CTPOL.size() - 2].y, Path_x1, 0, paraL1);//��ȡ��ʼ��
					}
					else{
						getLineLocation(CTPOL[i].x, CTPOL[i].y, CTPOL[i - 1].x, CTPOL[i - 1].y, Path_x1, axis_flag, paraL1);//��ȡ��ʼ��
					}
					PPSP.st_x = paraL1.x;
					PPSP.st_y = paraL1.y;
					//PPSP.index_axis = o_index;
					PPSP.index_axis = i;
					PPSP.index_flag = 0;

					//ȷ������i+1>i
					if ((CTPOL[i].y - CTPOL[i+1].y) > 0){//i������i+1���� ���·�
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
		else{ //axis_flag==1 y����
				
			if ((CrDis >= temp_CrDis) && ((CTPOL[i].y - CTPOL[i + 1].y) == 0)&&((Min_dis<= temp_MinCrDis)||(Min_dis_next <= temp_MinCrDis))){//��ֱ��y��������ɵ�
				temp_CrDis = CrDis;
			
				if (Min_dis > Min_dis_next){//i+1�����
					temp_MinCrDis = Min_dis_next;
					Offset_SP OSP;
					SPL(ptr_MMP, CTPOL[i + 1].x, CTPOL[i + 1].y, axis_flag, RunLens, OSP);//·�������֮Ϊ��׼
					Path_y1 = OSP.offset1;
					o_index = OSP.index_num;

					cout << "Path_y1" << Path_y1 << endl;
					LineLocation paraL1;
					getLineLocation(CTPOL[i + 1].x, CTPOL[i + 1].y, CTPOL[i + 2].x, CTPOL[i + 2].y, Path_y1, axis_flag, paraL1);//��ȡ��ʼ��
					PPSP.st_x = paraL1.x;
					PPSP.st_y = paraL1.y;
					PPSP.index_axis = i + 1;
					PPSP.index_flag = 1;

					/*ȷ������i+1>i*/
					if ((CTPOL[i + 1].x - CTPOL[i].x) > 0){//i+1����i���� �����
						PPSP.vector_flag = 3;//����滮
					}
					else{
						PPSP.vector_flag = 2;//���ҹ滮
					}

					PathLineStart_x = PPSP.st_x;
					PathLineStart_y = PPSP.st_y;
				}
				else{//i�����		
					temp_MinCrDis = Min_dis;
					Offset_SP OSP;
					SPL(ptr_MMP, CTPOL[i].x, CTPOL[i].y, axis_flag, RunLens, OSP);//·�������֮Ϊ��׼
					Path_y1 = OSP.offset1;
					o_index = OSP.index_num;

					LineLocation paraL1;
					if (i == 0){
						getLineLocation(CTPOL[i].x, CTPOL[i].y, CTPOL[CTPOL.size() - 2].x, CTPOL[CTPOL.size() - 2].y, Path_y1, axis_flag, paraL1);//��ȡ��ʼ��
					}
					else{
						getLineLocation(CTPOL[i].x, CTPOL[i].y, CTPOL[i - 1].x, CTPOL[i - 1].y, Path_y1, axis_flag, paraL1);//��ȡ��ʼ��
					}
					PPSP.st_x = paraL1.x;
					PPSP.st_y = paraL1.y;
					PPSP.index_axis = i;
					PPSP.index_flag = 0;

					/*ȷ������i->i+1*/
					if ((CTPOL[i].x - CTPOL[i+1].x) > 0){//i+1����i���� �����
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

	void SPL(double arr_MM[], double dx1, double dy1,double axis_flag, double Rlens, Offset_SP& OSP){ //�����������ƫ��
		double Offset_1;
		if (axis_flag == 0){//x��Ϊ��׼ʱ
			if (dx1 == arr_MM[0]){//��Сֵ��
				OSP.offset1= dx1 + (Rlens / 2);
				OSP.index_num = 0;
				//return Offset_1 = dx1 + (Rlens / 2);
			}
			else if (dx1 == arr_MM[1]){//���ֵ��
				OSP.offset1 = dx1 - (Rlens / 2);;
				OSP.index_num = 1;
				//return Offset_1 = dx1 - (Rlens / 2);
			}
		}
		else{
			if (dy1 == arr_MM[2]){//��Сֵ��
				OSP.offset1 = dy1 + (Rlens / 2);;
				OSP.index_num = 2;
				//return Offset_1 = dy1 + (Rlens / 2);
			}
			else if (dy1 == arr_MM[3]){//���ֵ��
				OSP.offset1= dy1 - (Rlens / 2);
				OSP.index_num = 3;
				//return Offset_1 = dy1 - (Rlens / 2);
			}
		}
	}

#endif // !SPP_H_INCLUDE
