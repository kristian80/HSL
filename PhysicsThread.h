#pragma once
#pragma once

#include "HSL.h"
#include "CargoObject.h"



class PhysicsThread
{
private:
	HSL_PlugIn& HSL;
	bool myRunFlag = true;
	bool HIGH_PERFORMANCE = false;
public:
	PhysicsThread(HSL_PlugIn& HSLNew);
	void RunPhysicsThread(int index);
};

