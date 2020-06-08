#include<iostream>
#include<math.h>
#include<unistd.h>//休眠头函数
#include<ctime>
#include <stdio.h>
#include <sys/select.h>
using namespace std;

# define RF 100
# define RR 200
# define LF 300
# define LR 400

const float L1=70;
const float L2=50;
const float L3=24;

class Motion
{
public:
	float origin_pos[4][3];//腿号，坐标
	float current_pos[4][3];//腿号，坐标
	float a[4][3];
	int step;
	float width;
	float length;
	float height;
	int t;
	float h;
	float ran;
	float hs;
	int state;
	Motion()
	{
		for(int i=0;i<4;i++)
		{
			origin_pos[i][0]=66;origin_pos[i][1]=40;origin_pos[i][2]=-37;
			current_pos[i][0]=66;current_pos[i][1]=40;current_pos[i][2]=-37;
			a[i][0]=0;	a[i][1]=0;	a[i][2]=0;
		}

		step=30;
		hs=1;//落地单腿距离
		state=1;
	//	h=0;
	//	ran=0;
	}

	void getNextStep()
	{

		if (state==1)
		{	
			cout<<"satae1"<<endl;
			Detach(RF);moving(RR);moving(LF);moving(LR);t++;
			if(t>step){state=2;setallcurrentposition();t=1;ran=Incident();PrintCoordinate();}
		}

		else if (state==2)
		{
			cout<<"satae2"<<endl;
			Adhesive(RF);t++;
			if(h>ran){state=3;setallcurrentposition();t=1;h=0;PrintCoordinate();}
		}

		else if (state==3)
		{
			cout<<"satae3"<<endl;
			Detach(RR);moving(RF);moving(LF);moving(LR);t++;
			if(t>step){state=4;setallcurrentposition();t=0;ran=Incident();PrintCoordinate();}
		}

		else if (state==4)
		{
			cout<<"satae4"<<endl;
			Adhesive(RR);t++;
			if(h>ran){state=5;setallcurrentposition();t=0;h=0;PrintCoordinate();}
		}

		else if (state==5)
		{	
			cout<<"satae5"<<endl;
			Detach(LF);moving(RR);moving(RF);moving(LR);t++;
			if(t>step){state=6;setallcurrentposition();t=0;ran=Incident();PrintCoordinate();}
		}

		else if (state==6)
		{
			cout<<"satae6"<<endl;
			Adhesive(LF);t++;
			if(h>ran){state=7;setallcurrentposition();t=0;h=0;PrintCoordinate();}
		}

		else if (state==7)
		{
			cout<<"satae7"<<endl;
			Detach(LR);moving(RR);moving(LF);moving(RF);t++;
			if(t>step){state=8;setallcurrentposition();t=0;ran=Incident();PrintCoordinate();}
		}

		else if (state==8)
		{
			cout<<"satae8"<<endl;
			Adhesive(LR);t++;
			if(h>ran){state=1;setallcurrentposition();t=0;h=0;PrintCoordinate();}
		}





		
	}


//	void TriangleGait()
//	{
//		for(t=0;t<step+1;t++)
//		{Detach(RF);moving(RR);	moving(LF);	moving(LR);Outputall_PWM();}  setallcurrentposition();  PrintCoordinate();
//		float ran=Incident();//获得随机数
//		cout<<ran<<endl;
//		for(t=0;h<ran;t++) {Adhesive(RF); }setcurrentposition(RF);PrintCoordinate();
/*	
		for(t=0;t<step+1;t++)
		{Detach(RR); moving(RF); moving(LF); moving(LR);} setallcurrentposition();PrintCoordinate();
		Adhesive(RR);setallcurrentposition();PrintCoordinate();
		
		for(t=0;t<step+1;t++)
		{Detach(LF); moving(RF); moving(RR); moving(LR);}setallcurrentposition();PrintCoordinate();
		Adhesive(LF);setallcurrentposition();PrintCoordinate();

		for(t=0;t<step+1;t++)
		{Detach(LR); moving(RF); moving(RR); moving(LF);}setallcurrentposition();PrintCoordinate();
		Adhesive(LR);setallcurrentposition();PrintCoordinate();
*/	
//	}
/*		void DiagonalGait()
	{
		for(t=0;t<step+1;t++)
		{Detach(RF);Detach(LR);	moving(RR);	moving(LF);}  setallcurrentposition();  PrintCoordinate();
		Adhesive(RF);Adhesive(RR);setallcurrentposition();PrintCoordinate();
		
		for(t=0;t<step+1;t++)
		{Detach(RR); Detach(LF); moving(RF); moving(LR);} setallcurrentposition();PrintCoordinate();
		Adhesive(RR);Adhesive(LR);setallcurrentposition();PrintCoordinate();
	}
*/
	void setallcurrentposition()
	{
		setcurrentposition(RF);
		setcurrentposition(RR);
		setcurrentposition(LF);
		setcurrentposition(LR);
	}
	void setcurrentposition(int ft)
	{
		if (ft==RF)
		{
			origin_pos[0][0]=current_pos[0][0];
			origin_pos[0][1]=current_pos[0][1];
			origin_pos[0][2]=current_pos[0][2];
		}

		else if (ft==RR)
		{
			origin_pos[1][0]=current_pos[1][0];
			origin_pos[1][1]=current_pos[1][1];
			origin_pos[1][2]=current_pos[1][2];
		}

		else if (ft==LF)
		{
			origin_pos[2][0]=current_pos[2][0];
			origin_pos[2][1]=current_pos[2][1];
			origin_pos[2][2]=current_pos[2][2];
		}

		else if (ft==LR)
		{
			origin_pos[3][0]=current_pos[3][0];
			origin_pos[3][1]=current_pos[3][1];
			origin_pos[3][2]=current_pos[3][2];
		}
	}

	void moving(int fn)
	{
		float widthL=-width/3;
		float lengthL=-length/3;
		float heightL=0;
		int k=10;
	
		     if(fn==RF){k=0;}
		else if(fn==RR){k=1;}
		else if(fn==LF){k=2;}
		else if(fn==LR){k=3;}
				current_pos[k][0]=origin_pos[k][0] + widthL / step * t;
				current_pos[k][1]=origin_pos[k][1] + lengthL / step * t;
				current_pos[k][2]=origin_pos[k][2] + heightL / step * t;

				a[k][0] = asin(-L3 / sqrt(current_pos[k][0] * current_pos[k][0] + current_pos[k][2] * current_pos[k][2]) ) - atan2(current_pos[k][2],current_pos[k][0]);

				a[k][1] = asin((current_pos[k][0] * current_pos[k][0] + current_pos[k][1] * current_pos[k][1] + current_pos[k][2] * current_pos[k][2] + L1 * L1 - L2 * L2 - L3 * L3) / ( 2 * L1 * sqrt (current_pos[k][0] * current_pos[k][0] +  current_pos[k][1] * current_pos[k][1] + current_pos[k][2] * current_pos[k][2] - L3 * L3)) )
						    	 - atan2(sqrt(current_pos[k][0] * current_pos[k][0] + current_pos[k][2] * current_pos[k][2] - L3 * L3) , current_pos[k][1]);

				a[k][2] = asin((L1 * L1 + L2 * L2 + L3 * L3 - current_pos[k][0] * current_pos[k][0] - current_pos[k][1] * current_pos[k][1] - current_pos[k][2] * current_pos[k][2]) / (2 * L1 * L2));

				for(int n=0;n<3;n++)
					{ a[k][n]=Decimal(a[k][n]);}
		//	Output_PWM(fn);
	}

	void Detach(int fn)
	//length x 左右
    //width y 前进
	{
		int k=10;

		if (fn==RF){k=0;}
		else if(fn==RR){k=1;}
		else if(fn==LF){k=2;}
		else if(fn==LR) {k=3;}

			current_pos[k][0]=origin_pos[k][0] + width / step * t;
			current_pos[k][1]=origin_pos[k][1] + length / step * t;
			current_pos[k][2]=origin_pos[k][2]- (height / (step * step) * t * t) + (2 * height / step * t);

			a[k][0] = asin(-L3 / sqrt(current_pos[k][0] * current_pos[k][0] + current_pos[k][2] * current_pos[k][2]) )- atan2(current_pos[k][2],current_pos[k][0]);
			a[k][1] = asin((current_pos[k][0] * current_pos[k][0] + current_pos[k][1] * current_pos[k][1] + current_pos[k][2]
				* current_pos[k][2] + L1 * L1 - L2 * L2 - L3 * L3) / ( 2 * L1 * sqrt (current_pos[k][0] * current_pos[k][0]
				+  current_pos[k][1] * current_pos[k][1] + current_pos[k][2] * current_pos[k][2] - L3 * L3)) )-atan2(sqrt(current_pos[k][0] * current_pos[k][0] + current_pos[k][2] * current_pos[k][2] - L3 * L3) , current_pos[k][1]);

			a[k][2] = asin((L1 * L1 + L2 * L2 + L3 * L3 - current_pos[k][0] * current_pos[k][0] - current_pos[k][1] * current_pos[k][1]
				- current_pos[k][2] * current_pos[k][2]) / (2 * L1 * L2));

			for(int n=0;n<3;n++)
				{
				a[k][n]=Decimal(a[k][n]);
				}
		// Output_PWM(fn);

	}

	void Adhesive(int fn)
	//length x 左右
  //width y 前进
	{	
		
		int k=0;
		if (fn==RF)
		{k=0;}
		else if(fn==RR)
		{k=1;}
		else if(fn==LF)
		{k=2;}
		else if(fn==LR)
		{k=3;}

			current_pos[k][0]=origin_pos[k][0] ;
			current_pos[k][1]=origin_pos[k][1] ;
		    current_pos[k][2]=origin_pos[k][2] - hs*t;

			h=origin_pos[k][2]-current_pos[k][2];

			a[k][0] = asin(-L3 / sqrt(current_pos[k][0] * current_pos[k][0] + current_pos[k][2] * current_pos[k][2]) ) - atan2(current_pos[k][2],current_pos[k][0]);
			a[k][1] = asin((current_pos[k][0] * current_pos[k][0] + current_pos[k][1] * current_pos[k][1] + current_pos[k][2] * current_pos[k][2] + L1 * L1 - L2 * L2 - L3 * L3) / ( 2 * L1 * sqrt (current_pos[k][0] * current_pos[k][0] +  current_pos[k][1] * current_pos[k][1] + current_pos[k][2] * current_pos[k][2] - L3 * L3)) )
							- atan2(sqrt(current_pos[k][0] * current_pos[k][0] + current_pos[k][2] * current_pos[k][2] - L3 * L3) , current_pos[k][1]);
			a[k][2] = asin((L1 * L1 + L2 * L2 + L3 * L3 - current_pos[k][0] * current_pos[k][0] - current_pos[k][1] * current_pos[k][1] - current_pos[k][2] * current_pos[k][2]) / (2 * L1 * L2));


			for(int n=0;n<3;n++)
			{
				a[k][n]=Decimal(a[k][n]);
			}
		//	Output_PWM(fn);
	}

	float Decimal(float a)
		{
			a=(int) (a/3.1415926*180*100);
			a= (float) (a/100);
			return a;
		}

	void PrintCoordinate()
		{	
			cout<<endl<<"RF坐标: "<<origin_pos[0][0]<<"  "<<origin_pos[0][1]<<"  "<<origin_pos[0][2]<<"  "<<endl;
			cout<<"RR坐标: "<<origin_pos[1][0]<<"  "<<origin_pos[1][1]<<"  "<<origin_pos[1][2]<<"  "<<endl;
			cout<<"LF坐标: "<<origin_pos[2][0]<<"  "<<origin_pos[2][1]<<"  "<<origin_pos[2][2]<<"  "<<endl;
			cout<<"LR坐标: "<<origin_pos[3][0]<<"  "<<origin_pos[3][1]<<"  "<<origin_pos[3][2]<<"  "<<endl<<endl;
		}

	void Set_Distance(float a,float b,float c)
		{
			width=a;
			length=b;
			height=c;
		}

	float Incident()
		{
		float ran=0;
		srand(int (time(0)));
		ran=rand()%10-5+height;
		return ran;
		}

	void Outputall_PWM()
		{
			cout<<endl<<a[0][0]<<"    "<<a[0][1]<<"    "<<a[0][2]<<endl;
			cout<<a[1][0]<<"    "<<a[1][1]<<"    "<<a[1][2]<<endl;
			cout<<a[2][0]<<"    "<<a[2][1]<<"    "<<a[2][2]<<endl;
			cout<<a[3][0]<<"    "<<a[3][1]<<"    "<<a[3][2]<<endl<<endl;
		}


	void Output_PWM(int fn)
		{
				 if(fn==RF)	{cout<<"RF:"<<a[0][0]<<"    "<<a[0][1]<<"    "<<a[0][2]<<endl;}
			else if(fn==RR)	{cout<<"RR:"<<a[1][0]<<"    "<<a[1][1]<<"    "<<a[1][2]<<endl;}	
			else if(fn==LF)	{cout<<"LF:"<<a[2][0]<<"    "<<a[2][1]<<"    "<<a[2][2]<<endl;}		
			else if(fn==LR)	{cout<<"LR:"<<a[3][0]<<"    "<<a[3][1]<<"    "<<a[3][2]<<endl;}
		}

};
Motion XY;

static void sleep_ms(unsigned int secs)

{

struct timeval tval;

tval.tv_sec=secs/1000;

tval.tv_usec=(secs*1000)%1000000;

select(0,NULL,NULL,NULL,&tval);
}




int main()
{
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
