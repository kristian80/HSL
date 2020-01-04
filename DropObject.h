#pragma once

#include "HSL.h"
#include "LoadBaseObject.h"


class DropObject :
	public LoadBaseObject
{
public:


	vector<double> myVectorZeroVector = vector<double>(3);

	vector<double> myVectorVelocity = vector<double>(3);
	vector<double> myVectorForceGravity = vector<double>(3);
	vector<double> myVectorWindVelocity = vector<double>(3);

	vector<double> myVectorDisplayOffset = vector<double>(3);
	vector<double> myVectorDisplayAngle = vector<double>(3);

	vector<double> myVectorSize = vector<double>(3); //Length / Width / Height
	vector<double> myVectorCrossSection = vector<double>(3);
	vector<double> myVectorCW = vector<double>(3);

	bool myInstancedDrawing = false;

	double myMass = 5.0;
	double myVolume = 1.0;

	double myAirSpeed = 0;
	double myAirResistance = 0;

	vector<double> myVectorAirVelocity = vector<double>(3);
	vector<double> myVectorForceAir = vector<double>(3);
	vector<double> myVectorForceTotal = vector<double>(3);
	vector<double> myVectorAccTotal = vector<double>(3);
	vector<double> myVectorVelocityDelta = vector<double>(3);


	DropObject();
	DropObject(vector<double> pos, vector<double> vel);
	~DropObject();

	void CalculatePhysics(DropHSLData& HSL);
};

