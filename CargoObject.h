#pragma once
#include "HSL.h"
#include "LoadBaseObject.h"
class CargoObject :
	public LoadBaseObject
{
public:
	HSL_PlugIn& HSL;

	vector<float> myVectorZeroVector = vector<float>(3);

	vector<float> myVectorVelocity = vector<float>(3);
	vector<float> myVectorForceGravity = vector<float>(3);
	vector<float> myVectorWindVelocity = vector<float>(3);
	vector<float> myVectorCargoOffset = vector<float>(3);
	vector<float> myVectorCargoRotation = vector<float>(3);

	vector<float> myVectorDisplayOffset = vector<float>(3);
	vector<float> myVectorDisplayAngle = vector<float>(3);

	vector<float> myVectorSize = vector<float>(3); //Length / Width / Height
	vector<float> myVectorCrossSection = vector<float>(3);
	vector<float> myVectorCW = vector<float>(3);

	bool myRopeConnected = false;
	bool myFollowOnly = false;
	bool myDrawingEnabled = false;

	float myWaterLevel = 0;
	float myVolume = 0;

	float myHeight = 0.1;
	float myMass = 5.0f;
	float myFrictionGlide = 0.35;
	float myFrictionStatic = 0.65;

	float myObjectTerrainLevel = 0;
	bool myTerrainHit = false;
	float myAirSpeed = 0;
	float myAirResistance = 0;
	float mySpeedStaticFriction = 0.3;
	bool myIsBambiBucket = false;

	bool myBambiBucketRelease = false;
	float myBambiBucketWaterLevel = 0;
	float myBambiBucketWaterWeight = 0;

	vector<float> myVectorForceRope = vector<float>(3);
	vector<float> myVectorAirVelocity = vector<float>(3);
	vector<float> myVectorWaterVelocity = vector<float>(3);
	vector<float> myVectorForceAir = vector<float>(3);
	vector<float> myVectorForceWater = vector<float>(3);
	vector<float> myVectorForceSwim = vector<float>(3);
	vector<float> myVectorForceTotal = vector<float>(3);
	vector<float> myVectorHorizontalVelocity = vector<float>(3);
	vector<float> myVectorForceFriction = vector<float>(3);
	vector<float> myVectorAccTotal = vector<float>(3);
	vector<float> myVectorVelocityDelta = vector<float>(3);
	vector<float> myVectorForceChopper = vector<float>(3);
	vector<float> myVectorMomentumChopper = vector<float>(3);

	CargoObject(HSL_PlugIn &HSLNew);
	~CargoObject();

	void CalculatePhysics();
	
};

