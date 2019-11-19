#pragma once

#include "HSL.h"
#include "LoadBaseObject.h"


class DropObject :
	public LoadBaseObject
{
public:


	vector<float> myVectorZeroVector = vector<float>(3);

	vector<float> myVectorVelocity = vector<float>(3);
	vector<float> myVectorForceGravity = vector<float>(3);
	vector<float> myVectorWindVelocity = vector<float>(3);

	vector<float> myVectorDisplayOffset = vector<float>(3);
	vector<float> myVectorDisplayAngle = vector<float>(3);

	vector<float> myVectorSize = vector<float>(3); //Length / Width / Height
	vector<float> myVectorCrossSection = vector<float>(3);
	vector<float> myVectorCW = vector<float>(3);

	bool myInstancedDrawing = false;

	float myMass = 5.0f;
	float myVolume = 1.0f;

	float myAirSpeed = 0;
	float myAirResistance = 0;

	vector<float> myVectorAirVelocity = vector<float>(3);
	vector<float> myVectorForceAir = vector<float>(3);
	vector<float> myVectorForceTotal = vector<float>(3);
	vector<float> myVectorAccTotal = vector<float>(3);
	vector<float> myVectorVelocityDelta = vector<float>(3);


	DropObject();
	DropObject(vector<float> pos, vector<float> vel);
	~DropObject();

	void CalculatePhysics(DropHSLData& HSL);
};

