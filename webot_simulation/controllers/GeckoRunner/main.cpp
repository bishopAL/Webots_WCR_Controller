#include<iostream>
#include<math.h>
#include<ctime>
#include"Motion.hpp"
int main()
{
	Motion XY;
	XY.Set_Distance(0,30,30);
	XY.TriangleGait();
//	XY.DiagonalGait();
	return 0;
}


//Motion::Set_Distance(0,30,30);
//Motion::TriangleGait();
