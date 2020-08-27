#include <iostream>
#include <cstdio>
#include <string>
#include <vector>
#include <algorithm>
#include <stdlib.h>
#include <iomanip>
#include "VecT.h"//一些类和结构体
#include "MAM.h"//比较大小值
#include "Ac.h"//计算面积
#include "PJ.h"//距离计算及交点计算
#include "SPP.h"//起点计算
#include "InfCal.h"

const double MATH_PI = 3.141592653589793238463;
const double lon_nom = 102834.74258026089786013677476285;// 经度
const double lat_nom = 111712.69150641055729984301412873;//纬度

using namespace std;

double angle(double x_an2_1, double y_an2_1, double x_an2_2, double y_an2_2); //p1与p2之间的角度

int main()
{
    //初始化参数
    int num1;//特征点个数
    int num2;//划分数量
    double sum_a = 0;//总面积
    double sum_dc1 = 0;//期望切分面积
    double sum_tc1 = 0;
    double arr_MAM[8];//传回的xy大小值
    double num_mmx, num_mmy;//xy轴长度
    double num_dc1;//待切分长度
    double num_sc1=0.1;//单位切分长度
    double num_c1;//切分长度
    double c_start;//切分起点
    int axis_flag = 0;//切分轴类型x=0, y=1;
    double temp_cut_x,temp_cut_y;
    double t_vecx, t_vecy;
    double Destime;//期望时间
    vector<Vec2d> POL;//创建一个位置vector
    vector<Vec_cp> CPOL;//创建一个切分位置vector
    vector<Vec_tcp> CTPOL;//创建一个临时位置vector
    vector<Vec3_cp> V3CP;//建一个保存切分点的vector
    int flag_nc = 0;//切分区域标志位

    //定义扫描宽度和起始点
    double RunLens =3;
    double StartPosition[2];//无人机起始点

    //StartPosition[0] = 40;
    //StartPosition[1] = 0;

    StartPosition[0] = 32.206878* lat_nom;
    StartPosition[1] = 118.719151 * lon_nom;

    //输入各参数--包括搜索区域特征点参数数量、各特征点及无人机数量
    cout << "请输入特征点个数：";
    cin >> num1;//输入数量
    for (int i = 0; i < num1; i++)//输入特征顶点
    {
        cout << "请输入第" << i + 1 << "个特征点:";
        cin >> t_vecx >> t_vecy;//按x和y输入
        POL.push_back(Vec2d(t_vecx* lat_nom, t_vecy*lon_nom));//存储顶点
    }
    cout<< "请输入计划搜索无人机数量：";
    cin >> num2;//输入无人机数量
    cout << "期望搜索时间";
    cin >> Destime;//期望时间


    //判断输入坐标的方向(顺时针or逆时针)
    int flag_clockwise = 0;//0-顺时针 1-逆时针
    float an1 = angle(POL[1].x, POL[1].y, POL[0].x, POL[0].y);
    float an2 = angle(POL[2].x, POL[2].y, POL[0].x, POL[0].y);
    float an3 = angle(POL[3].x, POL[3].y, POL[0].x, POL[0].y);
    cout << "an=" << an1 << "-" << an2 << "-" << an3<< endl;
    if (((an1 > an2)&&(an2 > an3))|| ((an1 < an2) && (an2 > an3)&&(an1<(MATH_PI/2)))){ //顺时针情况
        flag_clockwise = 0;
    }
    else{  
        flag_clockwise = 1;
    }
    cout << "flag_clockwise=" << flag_clockwise << endl;
    if (flag_clockwise == 1)//逆时针时，需要将顺序改成顺时针
    {
        int tn = (int)(POL.size() / 2);
        double te_x, te_y;//临时变量
        for (int i = 0; i < tn; i++){ //进行首尾数据调换
            te_x = POL[i].x; te_y = POL[i].y;//临时变量
            POL[i].x = POL[num1 - 1 - i].x; POL[i].y = POL[num1 - 1 - i].y;//尾部赋值给首部
            POL[num1 - 1 - i].x = te_x; POL[num1 - 1 - i].y = te_y;
        }
    }



    //获取输入坐标点的最大最小值并确定切分轴
    getMaxAndMin(POL,arr_MAM);//获取POL中的最小和最大值
    num_mmx = arr_MAM[1] - arr_MAM[0];//x轴长度
    num_mmy = arr_MAM[3] - arr_MAM[2];//y轴长度
    
    //选择较长的一根轴进行切分
    num_dc1=max(num_mmx, num_mmy);//选择切分轴(长轴) 比较xy轴大小-提取切分长度

    //选择切分轴中的切分点
    int p_start_index;//调整起点
    int p_middle_index;//最大值点
    if (num_dc1 == num_mmx){//如果在x轴上
        c_start = arr_MAM[0];//提取x轴最小点，将其作为切分起点
        p_start_index = (int)arr_MAM[4];//x最小值点坐标
        //p_middle_index = (int)arr_MAM[5];//最大值点
        axis_flag = 0;//记为x轴
    }
    else{
        c_start = arr_MAM[2]; // 提取y轴最小点，将其作为切分起点
        p_start_index = (int)arr_MAM[6];//y最小值点坐标
        p_middle_index = (int)arr_MAM[7];//最大值点
        axis_flag = 1;//记为y轴
    }

    //将区域坐标起点调整至最小值点 
    if (p_start_index > 0)//当输入顶点的起点不在最小值点时
    {
        for (int i = 0; i < p_start_index; i++)//将前p_start_index个点后移
        {
            POL.push_back(Vec2d(POL[0].x, POL[0].y));//存储最后一个点
            POL.erase(POL.begin());//删除第一个点
        }
    }

    getMaxAndMin(POL, arr_MAM);//获取POL中的最小和最大值
    p_start_index = (int)arr_MAM[4];//x最小值点坐标
    p_middle_index = (int)arr_MAM[5];//最大值点



    //输出修改后的点
    for (int i = 0; i < num1; i++)//输入特征顶点
    {
        cout << "修改成顺时针最小起点后的第" << i + 1 << "个特征点:"<< POL[i].x<<"-"<<POL[i].y<<endl;
    }

    //进行信息填充
    t_vecx = POL[0].x; t_vecy = POL[0].y;//在POL最后填充第一个位置
    POL.push_back(Vec2d(t_vecx, t_vecy));//存储最后一个点


    //进行长轴切分
    num_c1 = num_dc1 / num_sc1;//切分数量
    cout <<  "切分数量num_c1=" << num_c1 << endl;
    double *ptr_ca = new double[(int)num_c1];//创建一个包含切分点坐标的数组
    for (int i1 = 0; i1 < (num_c1 -1); i1++)//计算切分点坐标  num_c1--切分数量
    {
        ptr_ca[i1] = c_start + (double)i1 * num_sc1+ num_sc1;  //c_start--切分点起点  num_sc1-- 单位切分长度
        //cout << i1 << "个切分点为" << ptr_ca[i1] << endl;
    }


    cout << "axis_flag=" << axis_flag << endl;
    //计算由切分点生成的坐标点--保存到Vec_cp中
    for (int i1 = 0; i1 < num_c1 - 1; i1++){//遍历每个切点 num_c1--长轴切点数量  
        for (int i2 = 0; i2 < num1; i2++){//遍历每个顶点        
            if (axis_flag == 0){//若为x轴          
                if (((POL[i2].x <= ptr_ca[i1]) && (POL[i2 + 1].x > ptr_ca[i1]))|| ((POL[i2].x >= ptr_ca[i1]) && (POL[i2 + 1].x < ptr_ca[i1]))){//在切点两个点的x坐标之间
                    LinePara para1;
                    getLinePara(POL[i2].x, POL[i2].y, POL[i2 + 1].x, POL[i2 + 1].y, para1);//计算切点直线的k和b
                    temp_cut_x = ptr_ca[i1]; //确定切点的x轴
                    temp_cut_y = para1.k * ptr_ca[i1] + para1.b; //确定y轴
                    CPOL.push_back(Vec_cp(temp_cut_x, temp_cut_y)); //保存切点
                    //cout << "切点" << temp_cut_x << "-" << temp_cut_y << endl;
                }
            }
            else{//y轴          
                if (((POL[i2].y <= ptr_ca[i1]) && (POL[i2 + 1].y > ptr_ca[i1])) || ((POL[i2].y >= ptr_ca[i1]) && (POL[i2 + 1].y < ptr_ca[i1]))){//在两个点之间     
                    LinePara para1;
                    getLinePara(POL[i2].x, POL[i2].y, POL[i2 + 1].x, POL[i2 + 1].y, para1);//计算切点直线的k和b
                    if (POL[i2].x == POL[i2 + 1].x){//出现两个点垂直的情况时
                        temp_cut_y = ptr_ca[i1]; //切点y轴
                        temp_cut_x = POL[i2].x; //切点x轴为这两点的x轴坐标
                    }
                    else{ //不垂直时，按常规计算   
                        temp_cut_y = ptr_ca[i1];//确定切点的y轴
                        temp_cut_x = (ptr_ca[i1] - para1.b) / para1.k;                    
                    }
                    CPOL.push_back(Vec_cp(temp_cut_x, temp_cut_y)); // 保存切点
                    //cout << "切点" << temp_cut_x << "-" << temp_cut_y << endl;                      
                }
            }
        }
    }

    //计算总面积与划区切分面积
    sum_a = vec_ac(POL);//计算总面积
    sum_dc1 = sum_a / (double)num2;//计算期望切分面积
    
    //输出信息
    cout << "总面积：" << sum_a << "\n" << "区域切分面积：" << sum_dc1 << "\n" << "切分段落数量" << num2 << endl;
    //double v_UAV = 2;
    //double dtime;//期望扫描时间
    //double duavnum;//期望无人机
    //dtime=DTimeCal(sum_a, v_UAV, RunLens);
    //duavnum = DUAVNum(sum_a, Destime, v_UAV, RunLens);
    //cout << "期望扫描时间" << dtime << endl;
    //cout << "期望无人机" << duavnum << endl;



     //比较面积大小
    double temp_cut_up_x , temp_cut_up_y ;//上一次切割的上顶点
    double temp_cut_down_x , temp_cut_down_y ;//上一次切割的下顶点  

    for (int j = 0; j < (num_c1 -1); j++){//从切分的第一个位置开始计算，到达计算面积即记录  
        //x轴为切分点时
        if (axis_flag == 0){ 
            //第1个切分区域
            if (flag_nc == 0){
                //遍历上半部分顶点，查询满足切分区域内的点
                for (int j1 = 0; j1 < p_middle_index; j1++){
                    if (POL[j1].x < ptr_ca[j]) {//上半部分点，且在切分点左侧
                        CTPOL.push_back(Vec_tcp(POL[j1].x, POL[j1].y));//上半部分切分点
                        cout << "上半部分=" << POL[j1].x << "-" << POL[j1].y << endl;
                    }
                }
                //填充切割点
                CTPOL.push_back(Vec_tcp(CPOL[2 * j].x, CPOL[2 * j].y));//填充切割点1
                CTPOL.push_back(Vec_tcp(CPOL[2 * j + 1].x, CPOL[2 * j + 1].y));//填充切割点2
                //cout <<  "填充点1=" << CPOL[2 * j].x << "-" << CPOL[2 * j].y << endl;
                //cout << "填充点2=" << CPOL[2 * j+1].x << "-" << CPOL[2 * j+1].y << endl;
                //遍历下半部分顶点，查询满足切分区域内的点
                for (int j2 = p_middle_index; j2 < (POL.size() - 1); j2++){
                    if (POL[j2].x < ptr_ca[j]){//上半部分点，且在切分点左侧
                        CTPOL.push_back(Vec_tcp(POL[j2].x, POL[j2].y));
                        //cout << "下半部分=" << POL[j2].x << "-" << POL[j2].y << endl;

                    }
                }


                t_vecx = CTPOL[0].x; t_vecy = CTPOL[0].y;//在最后填充第一个位置
                CTPOL.push_back(Vec_tcp(t_vecx, t_vecy));

                sum_tc1 = vec_ac2(CTPOL);//计算切分区域总面积


                //cout << "sum_tc1=" << sum_tc1 << endl;//输出实际切分面积
                //for (int k = 0; k < CTPOL.size(); k++) {
                //    cout << k << "个CTPOL=" << CTPOL[k].x << "-" << CTPOL[k].y << endl;
                //}
               


                if (sum_tc1 >= sum_dc1){//判断是否到达期望面积
                    cout << "第" << flag_nc + 1 << "个区域：" << endl;//达到面积即输出
                    cout << "子区域面积=" << sum_tc1 << endl;//输出实际切分面积
                    //打印切分区域顶点
                    for (int k = 0; k < CTPOL.size(); k++){
                        cout << k << "个子区域顶点=" << CTPOL[k].x << "-" << CTPOL[k].y << endl;
                    }
                    //将切割点暂存
                    temp_cut_up_x = CPOL[2 * j].x;
                    temp_cut_up_y = CPOL[2 * j].y;
                    temp_cut_down_x = CPOL[2 * j + 1].x;
                    temp_cut_down_y = CPOL[2 * j + 1].y;
                    flag_nc++;//转向下一个点
                    //计算规划区域的起点
                    PathPlanStartPoint(CTPOL, axis_flag, StartPosition, RunLens);
                    CTPOL.clear();//清空数组
                }
                else{
                    CTPOL.clear();//清空数组
                }
            }
            //第一个和最后一个切分点之间
            else if ((flag_nc > 0) && (flag_nc < (num2 - 1)))
            {
                //填充最开始的两个点
                CTPOL.push_back(Vec_tcp(temp_cut_down_x, temp_cut_down_y));//填充flag_nc+1个位置的第一个点
                CTPOL.push_back(Vec_tcp(temp_cut_up_x, temp_cut_up_y));//填充flag_nc+1个位置的第二个点
                //遍历上半部分顶点，查询满足切分区域内的点
                for (int j1 = 0; j1 <= p_middle_index; j1++){//遍历上半部分顶点，查询满足切分区域内的点
                    if ((POL[j1].x >= temp_cut_up_x) && (POL[j1].x < ptr_ca[j])){//上半部分点，且在切分点左侧
                        CTPOL.push_back(Vec_tcp(POL[j1].x, POL[j1].y));//上半部分切分点
                        //cout << "上半部分=" << POL[j1].x << "-" << POL[j1].y << endl;
                    }
                }
                //填充切割点
                CTPOL.push_back(Vec_tcp(CPOL[2 * j].x, CPOL[2 * j].y));//填充切割点2*j
                CTPOL.push_back(Vec_tcp(CPOL[2 * j + 1].x, CPOL[2 * j + 1].y));//填充切割点2*j+1
                //cout <<  "填充点1=" << CPOL[2 * j].x << "-" << CPOL[2 * j].y << endl;
                //cout << "填充点2=" << CPOL[2 * j+1].x << "-" << CPOL[2 * j+1].y << endl;

                //遍历下半部分顶点，查询满足切分区域内的点
                for (int j2 = p_middle_index; j2 < (POL.size() - 1); j2++){
                    if ((POL[j2].x >= temp_cut_up_x) && (POL[j2].x < ptr_ca[j])){//上半部分点，且在切分点左侧
                        CTPOL.push_back(Vec_tcp(POL[j2].x, POL[j2].y));
                        //cout << "下半部分=" << POL[j2].x << "-" << POL[j2].y << endl;
                    }
                }

                t_vecx = CTPOL[0].x; t_vecy = CTPOL[0].y;//最后填充第一个位置
                CTPOL.push_back(Vec_tcp(t_vecx, t_vecy));//保存位置

                sum_tc1 = vec_ac2(CTPOL);//计算总面积  
                if ((sum_tc1 >= (sum_dc1 - 0.0001))){//判断是否到达期望面积
                    cout << "第" << flag_nc + 1 << "个区域：" << endl;//切分成功即打印切分信息
                    cout << "子区域面积=" << sum_tc1 << endl;//输出实际切分面积

                    //打印
                    for (int k = 0; k < CTPOL.size(); k++){
                        cout << k << "个子区域顶点=" << CTPOL[k].x << "-" << CTPOL[k].y << endl;
                    }
                    //将切割点暂存
                    temp_cut_up_x = CPOL[2 * j].x;
                    temp_cut_up_y = CPOL[2 * j].y;
                    temp_cut_down_x = CPOL[2 * j + 1].x;
                    temp_cut_down_y = CPOL[2 * j + 1].y;
                    flag_nc = flag_nc + 1;//转向下一个点
                    //计算规划区域的起点
                    PathPlanStartPoint(CTPOL, axis_flag, StartPosition, RunLens);
                    CTPOL.clear();//清空数组
                }
                else{
                    CTPOL.clear();//清空数组
                }
            }
            //最后一个区域
            else if (flag_nc == (num2 - 1)){
                //填充初始切割点
                CTPOL.push_back(Vec_tcp(temp_cut_down_x, temp_cut_down_y));//填充flag_nc+1个位置的第一个点
                CTPOL.push_back(Vec_tcp(temp_cut_up_x, temp_cut_up_y));//填充flag_nc+1个位置的第二个点
                //遍历其余顶点，查询满足切分区域内的点
                for (int j1 = 0; j1 <= (POL.size() - 1); j1++){
                    if (POL[j1].x > temp_cut_up_x){//上半部分点，且在切分点左侧
                        CTPOL.push_back(Vec_tcp(POL[j1].x, POL[j1].y));//上半部分切分点
                    }
                }
                //最后填充第一个位置
                t_vecx = CTPOL[0].x; t_vecy = CTPOL[0].y;
                CTPOL.push_back(Vec_tcp(t_vecx, t_vecy));

                sum_tc1 = vec_ac2(CTPOL);//计算总面积           
                cout << "第" << flag_nc + 1 << "个区域：" << endl;
                cout << "子区域面积=" << sum_tc1 << endl;//输出实际切分面积
                // 打印--将所有剩余的点全部划分为最后一个切分区域
                for (int k = 0; k < CTPOL.size(); k++){
                    cout << k << "个子区域顶点=" << CTPOL[k].x << "-" << CTPOL[k].y << endl;
                }
             
                flag_nc++;//停止输出
                PathPlanStartPoint(CTPOL, axis_flag, StartPosition, RunLens);//计算规划区域的起点
            }
        }
        //切分点y轴时
        else if (axis_flag == 1){
            //第1个切分区域
            if (flag_nc == 0){
                //遍历上半部分顶点，查询满足切分区域内的点
                for (int j1 = 0; j1 <= p_middle_index; j1++){
                    if (POL[j1].y < ptr_ca[j]){//上半部分点，且在切分点左侧                    
                        CTPOL.push_back(Vec_tcp(POL[j1].x, POL[j1].y));//上半部分切分点
                        //cout << "上半部分=" << POL[j1].x << "-" << POL[j1].y << endl;

                    }
                }
                //填充切割点
                CTPOL.push_back(Vec_tcp(CPOL[2 * j].x, CPOL[2 * j].y));//填充切割点1
                CTPOL.push_back(Vec_tcp(CPOL[2 * j + 1].x, CPOL[2 * j + 1].y));//填充切割点1
                //cout <<  "填充点1=" << CPOL[2 * j].x << "-" << CPOL[2 * j].y << endl;
                //cout << "填充点2=" << CPOL[2 * j+1].x << "-" << CPOL[2 * j+1].y << endl;


                //遍历下半部分顶点，查询满足切分区域内的点
                for (int j2 = p_middle_index; j2 < (POL.size() - 1); j2++){   
                    if (POL[j2].y < ptr_ca[j]){//上半部分点，且在切分点左侧
                        CTPOL.push_back(Vec_tcp(POL[j2].x, POL[j2].y));
                        //cout << "下半部分=" << POL[j2].x << "-" << POL[j2].y << endl;
                    }
                }

                t_vecx = CTPOL[0].x; t_vecy = CTPOL[0].y;//在最后填充第一个位置
                CTPOL.push_back(Vec_tcp(t_vecx, t_vecy));

                sum_tc1 = vec_ac2(CTPOL);//计算总面积 
                //cout << "sum_tc1=" << sum_tc1 << endl;//输出实际切分面积
                //for (int k = 0; k < CTPOL.size(); k++) {
                //    cout << k << "个CTPOL=" << CTPOL[k].x << "-" << CTPOL[k].y << endl;
                //}



                if (sum_tc1 >= sum_dc1){//判断是否到达期望面积
                    cout << "第" << flag_nc + 1 << "个区域：" << endl;//达到面积即输出
                    cout << "子区域面积=" << sum_tc1 << endl;//输出实际切分面积
                    //打印切分区域顶点
                    for (int k = 0; k < CTPOL.size(); k++){
                        cout << k << "个子区域顶点=" << CTPOL[k].x << "-" << CTPOL[k].y << endl;
                    }

                    //将切割点暂存
                    temp_cut_up_x = CPOL[2 * j].x;
                    temp_cut_up_y = CPOL[2 * j].y;
                    temp_cut_down_x = CPOL[2 * j + 1].x;
                    temp_cut_down_y = CPOL[2 * j + 1].y;
                    flag_nc++;//转向下一个点
                    //计算规划区域的起点
                    PathPlanStartPoint(CTPOL, axis_flag, StartPosition, RunLens);

                    CTPOL.clear();//清空数组
                }
                else{
                    CTPOL.clear();//清空数组
                }
            }
            //第一个和最后一个切分点之间
            else if ((flag_nc > 0) && (flag_nc < (num2 - 1))){
                //填充上一切割点
                CTPOL.push_back(Vec_tcp(temp_cut_down_x, temp_cut_down_y));//填充flag_nc+1个位置的第一个点
                CTPOL.push_back(Vec_tcp(temp_cut_up_x, temp_cut_up_y));//填充flag_nc+1个位置的第二个点
                //遍历上半部分顶点，查询满足切分区域内的点
                for (int j1 = 0; j1 <= p_middle_index; j1++){
                    if ((POL[j1].y >= temp_cut_up_y) && (POL[j1].y < ptr_ca[j])){//上半部分点，且在切分点左侧
                        CTPOL.push_back(Vec_tcp(POL[j1].x, POL[j1].y));//上半部分切分点
                    }
                }
                //填充切割点
                CTPOL.push_back(Vec_tcp(CPOL[2 * j].x, CPOL[2 * j].y));//填充切割点1
                CTPOL.push_back(Vec_tcp(CPOL[2 * j + 1].x, CPOL[2 * j + 1].y));//填充切割点1
                //遍历下半部分顶点，查询满足切分区域内的点
                for (int j2 = p_middle_index; j2 < (POL.size() - 1); j2++){
                    if ((POL[j2].y >= temp_cut_up_y) && (POL[j2].y < ptr_ca[j])){//上半部分点，且在切分点左侧
                        CTPOL.push_back(Vec_tcp(POL[j2].x, POL[j2].y));
                    }
                }
                t_vecx = CTPOL[0].x; t_vecy = CTPOL[0].y;//最后填充第一个位置
                CTPOL.push_back(Vec_tcp(t_vecx, t_vecy));//保存位置

                sum_tc1 = vec_ac2(CTPOL);//计算总面积            
                if ((sum_tc1 >= (sum_dc1 - 0.0001))){//判断是否到达期望面积
                    
                    cout << "第" << flag_nc + 1 << "个区域：" << endl;//切分成功即打印切分信息
                    cout << "子区域面积=" << sum_tc1 << endl;//输出实际切分面积
                    //打印
                    for (int k = 0; k < CTPOL.size(); k++){
                        cout << k << "个子区域顶点=" << CTPOL[k].x << "-" << CTPOL[k].y << endl;
                    }
                    //将切割点暂存
                    temp_cut_up_x = CPOL[2 * j].x;
                    temp_cut_up_y = CPOL[2 * j].y;
                    temp_cut_down_x = CPOL[2 * j + 1].x;
                    temp_cut_down_y = CPOL[2 * j + 1].y;
                    flag_nc = flag_nc + 1;//转向下一个点
                    //计算规划区域的起点
                    PathPlanStartPoint(CTPOL, axis_flag, StartPosition, RunLens);

                    CTPOL.clear();//清空数组
                }
                else{
                    CTPOL.clear();//清空数组
                }
            }
            else if (flag_nc == (num2 - 1)){//最后一个区域
                //填充上一位置切割点
                CTPOL.push_back(Vec_tcp(temp_cut_down_x, temp_cut_down_y));//填充flag_nc+1个位置的第一个点
                CTPOL.push_back(Vec_tcp(temp_cut_up_x, temp_cut_up_y));//填充flag_nc+1个位置的第二个点
                //遍历剩余顶点，查询满足切分区域内的点
                for (int j1 = 0; j1 <= (POL.size() - 1); j1++){
                    if (POL[j1].y > temp_cut_up_y){
                        CTPOL.push_back(Vec_tcp(POL[j1].x, POL[j1].y));//上半部分切分点
                    }
                }
                t_vecx = CTPOL[0].x; t_vecy = CTPOL[0].y;//在最后填充第一个位置
                CTPOL.push_back(Vec_tcp(t_vecx, t_vecy));
                
                sum_tc1 = vec_ac2(CTPOL);//计算总面积
               
                cout << "第" << flag_nc + 1 << "个区域：" << endl;
                cout << "sum_tc1=" << sum_tc1 << endl;//输出实际切分面积
                // 打印--将所有剩余的点全部划分为最后一个切分区域
                for (int k = 0; k < CTPOL.size(); k++){
                    cout << k << "个CTPOL=" << CTPOL[k].x << "-" << CTPOL[k].y << endl;
                }
                flag_nc++;//停止输出
                //计算规划区域的起点
                PathPlanStartPoint(CTPOL, axis_flag, StartPosition, RunLens);
            }
        }
    }


    delete[] ptr_ca;//删除new的对象
	return 0;
}



double angle(double x_an2_1, double y_an2_1, double x_an2_2, double y_an2_2) //p1与p2之间的角度
{   
    if ((x_an2_1 > x_an2_2) && (fabs(y_an2_1 - y_an2_2) < 0.0000001)) // 0度
        return 0;
    else if ((x_an2_1 > x_an2_2) && (y_an2_1 > y_an2_2)) //0-90度
        return atan((y_an2_1 - y_an2_2) / (x_an2_1 - x_an2_2));
    else if ((fabs(x_an2_1 - x_an2_2) < 0.0000001) && (y_an2_1 > y_an2_2)) //90度
        return MATH_PI / 2;
    else if ((x_an2_1 < x_an2_2) && (y_an2_1 > y_an2_2)) //90-180度
        return (atan((x_an2_2 - x_an2_1) / (y_an2_1 - y_an2_2)) + MATH_PI / 2);
    else if ((x_an2_1 < x_an2_2) && (fabs(y_an2_1 - y_an2_2) < 0.0000001)) //180度
        return MATH_PI;
    else if ((x_an2_1 < x_an2_2) && (y_an2_1 < y_an2_2)) //180-270度
        return (atan((y_an2_2 - y_an2_1) / (x_an2_2 - x_an2_1)) + MATH_PI);
    else if ((fabs(x_an2_1 - x_an2_2) < 0.0000001) && (y_an2_1 < y_an2_2)) //270度
        return 3 * MATH_PI / 2;
    else if ((x_an2_1 > x_an2_2) && (y_an2_1 < y_an2_2)) //270-360度
        return (atan((x_an2_1 - x_an2_2) / (y_an2_2 - y_an2_1)) + 3 * MATH_PI / 2);
    else
        return 0;
}


