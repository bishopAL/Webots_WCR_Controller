
# define RF 100
# define RR 200
# define LF 300
# define LR 400
#include"Motion.cpp" 
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
	void TriangleGait();
	void setallcurrentposition();
	void setcurrentposition(int ft);
	void moving(int fn);	
	void Detach(int fn);
	void Adhesive(int fn);
	void Set_Distance(float a,float b,float c);
	float Incident();
	void Output_PWM(int fn);
		
	};
