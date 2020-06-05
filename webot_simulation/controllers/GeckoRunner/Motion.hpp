#include<iostream>
#include<math.h>
#include<ctime>
# define RF 100
# define RR 200
# define LF 300
# define LR 400

const float L1=66;
const float L2=40;
const float L3=37;
using namespace std;
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
		Motion()
		{
		for(int i=0;i<4;i++)
		{
			origin_pos[i][0]=66;origin_pos[i][1]=40;origin_pos[i][2]=-37;
			current_pos[i][0]=66;current_pos[i][1]=40;current_pos[i][2]=-37;
			a[i][0]=0;	a[i][1]=0;	a[i][2]=0;
		}
		step=20;
		}
		
	void TriangleGait();
	void setallcurrentposition();
	void setcurrentposition(int ft);
	void moving(int fn);	
	void Detach(int fn);
	void Adhesive(int fn);
	void Set_Distance(float a,float b,float c);
	float Incident();
	void Output_PWM(int fn);
	void PrintCoordinate();
	float Decimal(float a);
	float getNextStep();
	};
