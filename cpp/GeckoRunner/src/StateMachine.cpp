// Copyright (c) IBSS NUAA. All rights reserved.
#include "StateMachine.h"
#include <vector>
#include <random>
using namespace std;
class StateMachine
{
public:
	float ORIGIN_POS[4][3]; //腿号，坐标
	float CURRENT_POS[4][3]; //腿号，坐标
	int step;
	float WIDTH;
	float LENGTH;
	float HEIGHT;
	float CONTACT_FLAG[4];
	int GAIT_MODE;
};