#pragma once
#ifndef FILES_H_INCLUDE
#define FILES_H_INCLUDE

#include <fstream>
#include <stdio.h>

using namespace std;

void CreatTxt(char* pathName, unsigned char* rBuffer, int length)//����txt�ļ�
{
	
	ofstream fout(pathName);
	if (fout) { // ��������ɹ�
		for (int i = 0; i < length; i++)
		{
			fout << ����д������ݡ��� << endl; // ʹ����coutͬ���ķ�ʽ����д��
		}

		fout.close();  // ִ���������ر��ļ����
	}
}

#endif // !FILES_H_INCLUDE
