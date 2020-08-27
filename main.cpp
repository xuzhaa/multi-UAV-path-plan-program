#include <iostream>
#include <cstdio>
#include <string>
#include <vector>
#include <algorithm>
#include <stdlib.h>
#include <iomanip>
#include "VecT.h"//һЩ��ͽṹ��
#include "MAM.h"//�Ƚϴ�Сֵ
#include "Ac.h"//�������
#include "PJ.h"//������㼰�������
#include "SPP.h"//������
#include "InfCal.h"

const double MATH_PI = 3.141592653589793238463;
const double lon_nom = 102834.74258026089786013677476285;// ����
const double lat_nom = 111712.69150641055729984301412873;//γ��

using namespace std;

double angle(double x_an2_1, double y_an2_1, double x_an2_2, double y_an2_2); //p1��p2֮��ĽǶ�

int main()
{
    //��ʼ������
    int num1;//���������
    int num2;//��������
    double sum_a = 0;//�����
    double sum_dc1 = 0;//�����з����
    double sum_tc1 = 0;
    double arr_MAM[8];//���ص�xy��Сֵ
    double num_mmx, num_mmy;//xy�᳤��
    double num_dc1;//���зֳ���
    double num_sc1=0.1;//��λ�зֳ���
    double num_c1;//�зֳ���
    double c_start;//�з����
    int axis_flag = 0;//�з�������x=0, y=1;
    double temp_cut_x,temp_cut_y;
    double t_vecx, t_vecy;
    double Destime;//����ʱ��
    vector<Vec2d> POL;//����һ��λ��vector
    vector<Vec_cp> CPOL;//����һ���з�λ��vector
    vector<Vec_tcp> CTPOL;//����һ����ʱλ��vector
    vector<Vec3_cp> V3CP;//��һ�������зֵ��vector
    int flag_nc = 0;//�з������־λ

    //����ɨ���Ⱥ���ʼ��
    double RunLens =3;
    double StartPosition[2];//���˻���ʼ��

    //StartPosition[0] = 40;
    //StartPosition[1] = 0;

    StartPosition[0] = 32.206878* lat_nom;
    StartPosition[1] = 118.719151 * lon_nom;

    //���������--����������������������������������㼰���˻�����
    cout << "�����������������";
    cin >> num1;//��������
    for (int i = 0; i < num1; i++)//������������
    {
        cout << "�������" << i + 1 << "��������:";
        cin >> t_vecx >> t_vecy;//��x��y����
        POL.push_back(Vec2d(t_vecx* lat_nom, t_vecy*lon_nom));//�洢����
    }
    cout<< "������ƻ��������˻�������";
    cin >> num2;//�������˻�����
    cout << "��������ʱ��";
    cin >> Destime;//����ʱ��


    //�ж���������ķ���(˳ʱ��or��ʱ��)
    int flag_clockwise = 0;//0-˳ʱ�� 1-��ʱ��
    float an1 = angle(POL[1].x, POL[1].y, POL[0].x, POL[0].y);
    float an2 = angle(POL[2].x, POL[2].y, POL[0].x, POL[0].y);
    float an3 = angle(POL[3].x, POL[3].y, POL[0].x, POL[0].y);
    cout << "an=" << an1 << "-" << an2 << "-" << an3<< endl;
    if (((an1 > an2)&&(an2 > an3))|| ((an1 < an2) && (an2 > an3)&&(an1<(MATH_PI/2)))){ //˳ʱ�����
        flag_clockwise = 0;
    }
    else{  
        flag_clockwise = 1;
    }
    cout << "flag_clockwise=" << flag_clockwise << endl;
    if (flag_clockwise == 1)//��ʱ��ʱ����Ҫ��˳��ĳ�˳ʱ��
    {
        int tn = (int)(POL.size() / 2);
        double te_x, te_y;//��ʱ����
        for (int i = 0; i < tn; i++){ //������β���ݵ���
            te_x = POL[i].x; te_y = POL[i].y;//��ʱ����
            POL[i].x = POL[num1 - 1 - i].x; POL[i].y = POL[num1 - 1 - i].y;//β����ֵ���ײ�
            POL[num1 - 1 - i].x = te_x; POL[num1 - 1 - i].y = te_y;
        }
    }



    //��ȡ���������������Сֵ��ȷ���з���
    getMaxAndMin(POL,arr_MAM);//��ȡPOL�е���С�����ֵ
    num_mmx = arr_MAM[1] - arr_MAM[0];//x�᳤��
    num_mmy = arr_MAM[3] - arr_MAM[2];//y�᳤��
    
    //ѡ��ϳ���һ��������з�
    num_dc1=max(num_mmx, num_mmy);//ѡ���з���(����) �Ƚ�xy���С-��ȡ�зֳ���

    //ѡ���з����е��зֵ�
    int p_start_index;//�������
    int p_middle_index;//���ֵ��
    if (num_dc1 == num_mmx){//�����x����
        c_start = arr_MAM[0];//��ȡx����С�㣬������Ϊ�з����
        p_start_index = (int)arr_MAM[4];//x��Сֵ������
        //p_middle_index = (int)arr_MAM[5];//���ֵ��
        axis_flag = 0;//��Ϊx��
    }
    else{
        c_start = arr_MAM[2]; // ��ȡy����С�㣬������Ϊ�з����
        p_start_index = (int)arr_MAM[6];//y��Сֵ������
        p_middle_index = (int)arr_MAM[7];//���ֵ��
        axis_flag = 1;//��Ϊy��
    }

    //��������������������Сֵ�� 
    if (p_start_index > 0)//�����붥�����㲻����Сֵ��ʱ
    {
        for (int i = 0; i < p_start_index; i++)//��ǰp_start_index�������
        {
            POL.push_back(Vec2d(POL[0].x, POL[0].y));//�洢���һ����
            POL.erase(POL.begin());//ɾ����һ����
        }
    }

    getMaxAndMin(POL, arr_MAM);//��ȡPOL�е���С�����ֵ
    p_start_index = (int)arr_MAM[4];//x��Сֵ������
    p_middle_index = (int)arr_MAM[5];//���ֵ��



    //����޸ĺ�ĵ�
    for (int i = 0; i < num1; i++)//������������
    {
        cout << "�޸ĳ�˳ʱ����С����ĵ�" << i + 1 << "��������:"<< POL[i].x<<"-"<<POL[i].y<<endl;
    }

    //������Ϣ���
    t_vecx = POL[0].x; t_vecy = POL[0].y;//��POL�������һ��λ��
    POL.push_back(Vec2d(t_vecx, t_vecy));//�洢���һ����


    //���г����з�
    num_c1 = num_dc1 / num_sc1;//�з�����
    cout <<  "�з�����num_c1=" << num_c1 << endl;
    double *ptr_ca = new double[(int)num_c1];//����һ�������зֵ����������
    for (int i1 = 0; i1 < (num_c1 -1); i1++)//�����зֵ�����  num_c1--�з�����
    {
        ptr_ca[i1] = c_start + (double)i1 * num_sc1+ num_sc1;  //c_start--�зֵ����  num_sc1-- ��λ�зֳ���
        //cout << i1 << "���зֵ�Ϊ" << ptr_ca[i1] << endl;
    }


    cout << "axis_flag=" << axis_flag << endl;
    //�������зֵ����ɵ������--���浽Vec_cp��
    for (int i1 = 0; i1 < num_c1 - 1; i1++){//����ÿ���е� num_c1--�����е�����  
        for (int i2 = 0; i2 < num1; i2++){//����ÿ������        
            if (axis_flag == 0){//��Ϊx��          
                if (((POL[i2].x <= ptr_ca[i1]) && (POL[i2 + 1].x > ptr_ca[i1]))|| ((POL[i2].x >= ptr_ca[i1]) && (POL[i2 + 1].x < ptr_ca[i1]))){//���е��������x����֮��
                    LinePara para1;
                    getLinePara(POL[i2].x, POL[i2].y, POL[i2 + 1].x, POL[i2 + 1].y, para1);//�����е�ֱ�ߵ�k��b
                    temp_cut_x = ptr_ca[i1]; //ȷ���е��x��
                    temp_cut_y = para1.k * ptr_ca[i1] + para1.b; //ȷ��y��
                    CPOL.push_back(Vec_cp(temp_cut_x, temp_cut_y)); //�����е�
                    //cout << "�е�" << temp_cut_x << "-" << temp_cut_y << endl;
                }
            }
            else{//y��          
                if (((POL[i2].y <= ptr_ca[i1]) && (POL[i2 + 1].y > ptr_ca[i1])) || ((POL[i2].y >= ptr_ca[i1]) && (POL[i2 + 1].y < ptr_ca[i1]))){//��������֮��     
                    LinePara para1;
                    getLinePara(POL[i2].x, POL[i2].y, POL[i2 + 1].x, POL[i2 + 1].y, para1);//�����е�ֱ�ߵ�k��b
                    if (POL[i2].x == POL[i2 + 1].x){//���������㴹ֱ�����ʱ
                        temp_cut_y = ptr_ca[i1]; //�е�y��
                        temp_cut_x = POL[i2].x; //�е�x��Ϊ�������x������
                    }
                    else{ //����ֱʱ�����������   
                        temp_cut_y = ptr_ca[i1];//ȷ���е��y��
                        temp_cut_x = (ptr_ca[i1] - para1.b) / para1.k;                    
                    }
                    CPOL.push_back(Vec_cp(temp_cut_x, temp_cut_y)); // �����е�
                    //cout << "�е�" << temp_cut_x << "-" << temp_cut_y << endl;                      
                }
            }
        }
    }

    //����������뻮���з����
    sum_a = vec_ac(POL);//���������
    sum_dc1 = sum_a / (double)num2;//���������з����
    
    //�����Ϣ
    cout << "�������" << sum_a << "\n" << "�����з������" << sum_dc1 << "\n" << "�зֶ�������" << num2 << endl;
    //double v_UAV = 2;
    //double dtime;//����ɨ��ʱ��
    //double duavnum;//�������˻�
    //dtime=DTimeCal(sum_a, v_UAV, RunLens);
    //duavnum = DUAVNum(sum_a, Destime, v_UAV, RunLens);
    //cout << "����ɨ��ʱ��" << dtime << endl;
    //cout << "�������˻�" << duavnum << endl;



     //�Ƚ������С
    double temp_cut_up_x , temp_cut_up_y ;//��һ���и���϶���
    double temp_cut_down_x , temp_cut_down_y ;//��һ���и���¶���  

    for (int j = 0; j < (num_c1 -1); j++){//���зֵĵ�һ��λ�ÿ�ʼ���㣬��������������¼  
        //x��Ϊ�зֵ�ʱ
        if (axis_flag == 0){ 
            //��1���з�����
            if (flag_nc == 0){
                //�����ϰ벿�ֶ��㣬��ѯ�����з������ڵĵ�
                for (int j1 = 0; j1 < p_middle_index; j1++){
                    if (POL[j1].x < ptr_ca[j]) {//�ϰ벿�ֵ㣬�����зֵ����
                        CTPOL.push_back(Vec_tcp(POL[j1].x, POL[j1].y));//�ϰ벿���зֵ�
                        cout << "�ϰ벿��=" << POL[j1].x << "-" << POL[j1].y << endl;
                    }
                }
                //����и��
                CTPOL.push_back(Vec_tcp(CPOL[2 * j].x, CPOL[2 * j].y));//����и��1
                CTPOL.push_back(Vec_tcp(CPOL[2 * j + 1].x, CPOL[2 * j + 1].y));//����и��2
                //cout <<  "����1=" << CPOL[2 * j].x << "-" << CPOL[2 * j].y << endl;
                //cout << "����2=" << CPOL[2 * j+1].x << "-" << CPOL[2 * j+1].y << endl;
                //�����°벿�ֶ��㣬��ѯ�����з������ڵĵ�
                for (int j2 = p_middle_index; j2 < (POL.size() - 1); j2++){
                    if (POL[j2].x < ptr_ca[j]){//�ϰ벿�ֵ㣬�����зֵ����
                        CTPOL.push_back(Vec_tcp(POL[j2].x, POL[j2].y));
                        //cout << "�°벿��=" << POL[j2].x << "-" << POL[j2].y << endl;

                    }
                }


                t_vecx = CTPOL[0].x; t_vecy = CTPOL[0].y;//���������һ��λ��
                CTPOL.push_back(Vec_tcp(t_vecx, t_vecy));

                sum_tc1 = vec_ac2(CTPOL);//�����з����������


                //cout << "sum_tc1=" << sum_tc1 << endl;//���ʵ���з����
                //for (int k = 0; k < CTPOL.size(); k++) {
                //    cout << k << "��CTPOL=" << CTPOL[k].x << "-" << CTPOL[k].y << endl;
                //}
               


                if (sum_tc1 >= sum_dc1){//�ж��Ƿ񵽴��������
                    cout << "��" << flag_nc + 1 << "������" << endl;//�ﵽ��������
                    cout << "���������=" << sum_tc1 << endl;//���ʵ���з����
                    //��ӡ�з����򶥵�
                    for (int k = 0; k < CTPOL.size(); k++){
                        cout << k << "�������򶥵�=" << CTPOL[k].x << "-" << CTPOL[k].y << endl;
                    }
                    //���и���ݴ�
                    temp_cut_up_x = CPOL[2 * j].x;
                    temp_cut_up_y = CPOL[2 * j].y;
                    temp_cut_down_x = CPOL[2 * j + 1].x;
                    temp_cut_down_y = CPOL[2 * j + 1].y;
                    flag_nc++;//ת����һ����
                    //����滮��������
                    PathPlanStartPoint(CTPOL, axis_flag, StartPosition, RunLens);
                    CTPOL.clear();//�������
                }
                else{
                    CTPOL.clear();//�������
                }
            }
            //��һ�������һ���зֵ�֮��
            else if ((flag_nc > 0) && (flag_nc < (num2 - 1)))
            {
                //����ʼ��������
                CTPOL.push_back(Vec_tcp(temp_cut_down_x, temp_cut_down_y));//���flag_nc+1��λ�õĵ�һ����
                CTPOL.push_back(Vec_tcp(temp_cut_up_x, temp_cut_up_y));//���flag_nc+1��λ�õĵڶ�����
                //�����ϰ벿�ֶ��㣬��ѯ�����з������ڵĵ�
                for (int j1 = 0; j1 <= p_middle_index; j1++){//�����ϰ벿�ֶ��㣬��ѯ�����з������ڵĵ�
                    if ((POL[j1].x >= temp_cut_up_x) && (POL[j1].x < ptr_ca[j])){//�ϰ벿�ֵ㣬�����зֵ����
                        CTPOL.push_back(Vec_tcp(POL[j1].x, POL[j1].y));//�ϰ벿���зֵ�
                        //cout << "�ϰ벿��=" << POL[j1].x << "-" << POL[j1].y << endl;
                    }
                }
                //����и��
                CTPOL.push_back(Vec_tcp(CPOL[2 * j].x, CPOL[2 * j].y));//����и��2*j
                CTPOL.push_back(Vec_tcp(CPOL[2 * j + 1].x, CPOL[2 * j + 1].y));//����и��2*j+1
                //cout <<  "����1=" << CPOL[2 * j].x << "-" << CPOL[2 * j].y << endl;
                //cout << "����2=" << CPOL[2 * j+1].x << "-" << CPOL[2 * j+1].y << endl;

                //�����°벿�ֶ��㣬��ѯ�����з������ڵĵ�
                for (int j2 = p_middle_index; j2 < (POL.size() - 1); j2++){
                    if ((POL[j2].x >= temp_cut_up_x) && (POL[j2].x < ptr_ca[j])){//�ϰ벿�ֵ㣬�����зֵ����
                        CTPOL.push_back(Vec_tcp(POL[j2].x, POL[j2].y));
                        //cout << "�°벿��=" << POL[j2].x << "-" << POL[j2].y << endl;
                    }
                }

                t_vecx = CTPOL[0].x; t_vecy = CTPOL[0].y;//�������һ��λ��
                CTPOL.push_back(Vec_tcp(t_vecx, t_vecy));//����λ��

                sum_tc1 = vec_ac2(CTPOL);//���������  
                if ((sum_tc1 >= (sum_dc1 - 0.0001))){//�ж��Ƿ񵽴��������
                    cout << "��" << flag_nc + 1 << "������" << endl;//�зֳɹ�����ӡ�з���Ϣ
                    cout << "���������=" << sum_tc1 << endl;//���ʵ���з����

                    //��ӡ
                    for (int k = 0; k < CTPOL.size(); k++){
                        cout << k << "�������򶥵�=" << CTPOL[k].x << "-" << CTPOL[k].y << endl;
                    }
                    //���и���ݴ�
                    temp_cut_up_x = CPOL[2 * j].x;
                    temp_cut_up_y = CPOL[2 * j].y;
                    temp_cut_down_x = CPOL[2 * j + 1].x;
                    temp_cut_down_y = CPOL[2 * j + 1].y;
                    flag_nc = flag_nc + 1;//ת����һ����
                    //����滮��������
                    PathPlanStartPoint(CTPOL, axis_flag, StartPosition, RunLens);
                    CTPOL.clear();//�������
                }
                else{
                    CTPOL.clear();//�������
                }
            }
            //���һ������
            else if (flag_nc == (num2 - 1)){
                //����ʼ�и��
                CTPOL.push_back(Vec_tcp(temp_cut_down_x, temp_cut_down_y));//���flag_nc+1��λ�õĵ�һ����
                CTPOL.push_back(Vec_tcp(temp_cut_up_x, temp_cut_up_y));//���flag_nc+1��λ�õĵڶ�����
                //�������ඥ�㣬��ѯ�����з������ڵĵ�
                for (int j1 = 0; j1 <= (POL.size() - 1); j1++){
                    if (POL[j1].x > temp_cut_up_x){//�ϰ벿�ֵ㣬�����зֵ����
                        CTPOL.push_back(Vec_tcp(POL[j1].x, POL[j1].y));//�ϰ벿���зֵ�
                    }
                }
                //�������һ��λ��
                t_vecx = CTPOL[0].x; t_vecy = CTPOL[0].y;
                CTPOL.push_back(Vec_tcp(t_vecx, t_vecy));

                sum_tc1 = vec_ac2(CTPOL);//���������           
                cout << "��" << flag_nc + 1 << "������" << endl;
                cout << "���������=" << sum_tc1 << endl;//���ʵ���з����
                // ��ӡ--������ʣ��ĵ�ȫ������Ϊ���һ���з�����
                for (int k = 0; k < CTPOL.size(); k++){
                    cout << k << "�������򶥵�=" << CTPOL[k].x << "-" << CTPOL[k].y << endl;
                }
             
                flag_nc++;//ֹͣ���
                PathPlanStartPoint(CTPOL, axis_flag, StartPosition, RunLens);//����滮��������
            }
        }
        //�зֵ�y��ʱ
        else if (axis_flag == 1){
            //��1���з�����
            if (flag_nc == 0){
                //�����ϰ벿�ֶ��㣬��ѯ�����з������ڵĵ�
                for (int j1 = 0; j1 <= p_middle_index; j1++){
                    if (POL[j1].y < ptr_ca[j]){//�ϰ벿�ֵ㣬�����зֵ����                    
                        CTPOL.push_back(Vec_tcp(POL[j1].x, POL[j1].y));//�ϰ벿���зֵ�
                        //cout << "�ϰ벿��=" << POL[j1].x << "-" << POL[j1].y << endl;

                    }
                }
                //����и��
                CTPOL.push_back(Vec_tcp(CPOL[2 * j].x, CPOL[2 * j].y));//����и��1
                CTPOL.push_back(Vec_tcp(CPOL[2 * j + 1].x, CPOL[2 * j + 1].y));//����и��1
                //cout <<  "����1=" << CPOL[2 * j].x << "-" << CPOL[2 * j].y << endl;
                //cout << "����2=" << CPOL[2 * j+1].x << "-" << CPOL[2 * j+1].y << endl;


                //�����°벿�ֶ��㣬��ѯ�����з������ڵĵ�
                for (int j2 = p_middle_index; j2 < (POL.size() - 1); j2++){   
                    if (POL[j2].y < ptr_ca[j]){//�ϰ벿�ֵ㣬�����зֵ����
                        CTPOL.push_back(Vec_tcp(POL[j2].x, POL[j2].y));
                        //cout << "�°벿��=" << POL[j2].x << "-" << POL[j2].y << endl;
                    }
                }

                t_vecx = CTPOL[0].x; t_vecy = CTPOL[0].y;//���������һ��λ��
                CTPOL.push_back(Vec_tcp(t_vecx, t_vecy));

                sum_tc1 = vec_ac2(CTPOL);//��������� 
                //cout << "sum_tc1=" << sum_tc1 << endl;//���ʵ���з����
                //for (int k = 0; k < CTPOL.size(); k++) {
                //    cout << k << "��CTPOL=" << CTPOL[k].x << "-" << CTPOL[k].y << endl;
                //}



                if (sum_tc1 >= sum_dc1){//�ж��Ƿ񵽴��������
                    cout << "��" << flag_nc + 1 << "������" << endl;//�ﵽ��������
                    cout << "���������=" << sum_tc1 << endl;//���ʵ���з����
                    //��ӡ�з����򶥵�
                    for (int k = 0; k < CTPOL.size(); k++){
                        cout << k << "�������򶥵�=" << CTPOL[k].x << "-" << CTPOL[k].y << endl;
                    }

                    //���и���ݴ�
                    temp_cut_up_x = CPOL[2 * j].x;
                    temp_cut_up_y = CPOL[2 * j].y;
                    temp_cut_down_x = CPOL[2 * j + 1].x;
                    temp_cut_down_y = CPOL[2 * j + 1].y;
                    flag_nc++;//ת����һ����
                    //����滮��������
                    PathPlanStartPoint(CTPOL, axis_flag, StartPosition, RunLens);

                    CTPOL.clear();//�������
                }
                else{
                    CTPOL.clear();//�������
                }
            }
            //��һ�������һ���зֵ�֮��
            else if ((flag_nc > 0) && (flag_nc < (num2 - 1))){
                //�����һ�и��
                CTPOL.push_back(Vec_tcp(temp_cut_down_x, temp_cut_down_y));//���flag_nc+1��λ�õĵ�һ����
                CTPOL.push_back(Vec_tcp(temp_cut_up_x, temp_cut_up_y));//���flag_nc+1��λ�õĵڶ�����
                //�����ϰ벿�ֶ��㣬��ѯ�����з������ڵĵ�
                for (int j1 = 0; j1 <= p_middle_index; j1++){
                    if ((POL[j1].y >= temp_cut_up_y) && (POL[j1].y < ptr_ca[j])){//�ϰ벿�ֵ㣬�����зֵ����
                        CTPOL.push_back(Vec_tcp(POL[j1].x, POL[j1].y));//�ϰ벿���зֵ�
                    }
                }
                //����и��
                CTPOL.push_back(Vec_tcp(CPOL[2 * j].x, CPOL[2 * j].y));//����и��1
                CTPOL.push_back(Vec_tcp(CPOL[2 * j + 1].x, CPOL[2 * j + 1].y));//����и��1
                //�����°벿�ֶ��㣬��ѯ�����з������ڵĵ�
                for (int j2 = p_middle_index; j2 < (POL.size() - 1); j2++){
                    if ((POL[j2].y >= temp_cut_up_y) && (POL[j2].y < ptr_ca[j])){//�ϰ벿�ֵ㣬�����зֵ����
                        CTPOL.push_back(Vec_tcp(POL[j2].x, POL[j2].y));
                    }
                }
                t_vecx = CTPOL[0].x; t_vecy = CTPOL[0].y;//�������һ��λ��
                CTPOL.push_back(Vec_tcp(t_vecx, t_vecy));//����λ��

                sum_tc1 = vec_ac2(CTPOL);//���������            
                if ((sum_tc1 >= (sum_dc1 - 0.0001))){//�ж��Ƿ񵽴��������
                    
                    cout << "��" << flag_nc + 1 << "������" << endl;//�зֳɹ�����ӡ�з���Ϣ
                    cout << "���������=" << sum_tc1 << endl;//���ʵ���з����
                    //��ӡ
                    for (int k = 0; k < CTPOL.size(); k++){
                        cout << k << "�������򶥵�=" << CTPOL[k].x << "-" << CTPOL[k].y << endl;
                    }
                    //���и���ݴ�
                    temp_cut_up_x = CPOL[2 * j].x;
                    temp_cut_up_y = CPOL[2 * j].y;
                    temp_cut_down_x = CPOL[2 * j + 1].x;
                    temp_cut_down_y = CPOL[2 * j + 1].y;
                    flag_nc = flag_nc + 1;//ת����һ����
                    //����滮��������
                    PathPlanStartPoint(CTPOL, axis_flag, StartPosition, RunLens);

                    CTPOL.clear();//�������
                }
                else{
                    CTPOL.clear();//�������
                }
            }
            else if (flag_nc == (num2 - 1)){//���һ������
                //�����һλ���и��
                CTPOL.push_back(Vec_tcp(temp_cut_down_x, temp_cut_down_y));//���flag_nc+1��λ�õĵ�һ����
                CTPOL.push_back(Vec_tcp(temp_cut_up_x, temp_cut_up_y));//���flag_nc+1��λ�õĵڶ�����
                //����ʣ�ඥ�㣬��ѯ�����з������ڵĵ�
                for (int j1 = 0; j1 <= (POL.size() - 1); j1++){
                    if (POL[j1].y > temp_cut_up_y){
                        CTPOL.push_back(Vec_tcp(POL[j1].x, POL[j1].y));//�ϰ벿���зֵ�
                    }
                }
                t_vecx = CTPOL[0].x; t_vecy = CTPOL[0].y;//���������һ��λ��
                CTPOL.push_back(Vec_tcp(t_vecx, t_vecy));
                
                sum_tc1 = vec_ac2(CTPOL);//���������
               
                cout << "��" << flag_nc + 1 << "������" << endl;
                cout << "sum_tc1=" << sum_tc1 << endl;//���ʵ���з����
                // ��ӡ--������ʣ��ĵ�ȫ������Ϊ���һ���з�����
                for (int k = 0; k < CTPOL.size(); k++){
                    cout << k << "��CTPOL=" << CTPOL[k].x << "-" << CTPOL[k].y << endl;
                }
                flag_nc++;//ֹͣ���
                //����滮��������
                PathPlanStartPoint(CTPOL, axis_flag, StartPosition, RunLens);
            }
        }
    }


    delete[] ptr_ca;//ɾ��new�Ķ���
	return 0;
}



double angle(double x_an2_1, double y_an2_1, double x_an2_2, double y_an2_2) //p1��p2֮��ĽǶ�
{   
    if ((x_an2_1 > x_an2_2) && (fabs(y_an2_1 - y_an2_2) < 0.0000001)) // 0��
        return 0;
    else if ((x_an2_1 > x_an2_2) && (y_an2_1 > y_an2_2)) //0-90��
        return atan((y_an2_1 - y_an2_2) / (x_an2_1 - x_an2_2));
    else if ((fabs(x_an2_1 - x_an2_2) < 0.0000001) && (y_an2_1 > y_an2_2)) //90��
        return MATH_PI / 2;
    else if ((x_an2_1 < x_an2_2) && (y_an2_1 > y_an2_2)) //90-180��
        return (atan((x_an2_2 - x_an2_1) / (y_an2_1 - y_an2_2)) + MATH_PI / 2);
    else if ((x_an2_1 < x_an2_2) && (fabs(y_an2_1 - y_an2_2) < 0.0000001)) //180��
        return MATH_PI;
    else if ((x_an2_1 < x_an2_2) && (y_an2_1 < y_an2_2)) //180-270��
        return (atan((y_an2_2 - y_an2_1) / (x_an2_2 - x_an2_1)) + MATH_PI);
    else if ((fabs(x_an2_1 - x_an2_2) < 0.0000001) && (y_an2_1 < y_an2_2)) //270��
        return 3 * MATH_PI / 2;
    else if ((x_an2_1 > x_an2_2) && (y_an2_1 < y_an2_2)) //270-360��
        return (atan((x_an2_1 - x_an2_2) / (y_an2_2 - y_an2_1)) + 3 * MATH_PI / 2);
    else
        return 0;
}


