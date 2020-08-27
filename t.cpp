//#include <iostream>
//#include <cstdio>
//#include <string>
//#include <vector>
//#include "area_cal.h"
//
//using namespace std;

//int main()
//{
//	int num1;
//	double sum = 0;
//	cout << "请输入特征点个数：";
//	cin >> num1;
//	double(* ptr_ver)[2] = new double[num1+1][2];
//	for (int i = 0; i < num1; i++)
//	{
//		cout << "请输入第"<<i+1<<"个特征点:";
//		cin >> ptr_ver[i][0] >> ptr_ver[i][1];
//	}
//
//	ptr_ver[num1][0] = ptr_ver[0][0];
//	ptr_ver[num1][1] = ptr_ver[0][1];
//
//	sum=polygon_area_cal(ptr_ver, num1);
//
//	cout << "面积为："<<sum << endl;
//	return 0;
//}
