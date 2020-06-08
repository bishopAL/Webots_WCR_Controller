#include<iostream>
#include<math.h>
#include<ctime>
#include"Motion.hpp"

static void sleep_ms(unsigned int secs)

{

struct timeval tval;

tval.tv_sec=secs/1000;

tval.tv_usec=(secs*1000)%1000000;

select(0,NULL,NULL,NULL,&tval);
}

int main()
{
	Motion XY;
	float b[4][3]={0};
	XY.Set_Distance(0,30,30);
	//XY.TriangleGait();
	//XY.DiagonalGait();
 //     LARGE_INTEGER freq;  
 //     LARGE_INTEGER start_t, stop_t;  
 //     double exe_time; 
 //	 QueryPerformanceFrequency(&freq);

	for(int t=0;t<500;t++)
	{	
	//    QueryPerformanceCounter(&start_t); 
		XY.getNextStep();
	//	QueryPerformanceCounter(&stop_t);  
		
	//	exe_time = 1e3*(stop_t.QuadPart-start_t.QuadPart)/freq.QuadPart; 

	//	cout<<"计算时间:"<<exe_time<<endl;

	//	Sleep(10-exe_time);
		sleep_ms(10);

		for(int i=0;i<4;i++)
		{
			for(int j=0;j<3;j++)
			{
				cout<<XY.a[i][j]<<"      ";
			}
			cout<<endl;
		}
		cout<<endl;	
	}
		return 0;
}
