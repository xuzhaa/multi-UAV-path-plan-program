#pragma once
#ifndef FILES_H_INCLUDE
#define FILES_H_INCLUDE

#include <fstream>
#include <stdio.h>

using namespace std;

void CreatTxt(char* pathName, unsigned char* rBuffer, int length)//创建txt文件
{
	
	ofstream fout(pathName);
	if (fout) { // 如果创建成功
		for (int i = 0; i < length; i++)
		{
			fout << “”写入的内容“” << endl; // 使用与cout同样的方式进行写入
		}

		fout.close();  // 执行完操作后关闭文件句柄
	}
}

#endif // !FILES_H_INCLUDE
