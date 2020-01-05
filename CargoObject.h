#pragma once
#include "HSL.h"
#include "LoadBaseObject.h"
class CargoObject :
	public LoadBaseObject
{
	bool HIGH_PERFORMANCE = false;
public:
	//HSL_PlugIn& HSL;
	CargoDataShared &myCargoDataShared;

	XPLMInstanceRef myDrawInstance = NULL;


	vector<double> myVectorZeroVector = vector<double>(3);

	vector<double> myVectorHelicopterPositionApprox = vector<double>(3);
	vector<double> myVectorHelicopterPositionApproxOld = vector<double>(3);
	vector<double> myVectorHelicopterVelocityApprox = vector<double>(3);
	vector<double> myVectorHelicopterPositionDeviation = vector<double>(3);

	vector<double> myVectorVelocity = vector<double>(3);
	vector<double> myVectorForceGravity = vector<double>(3);
	vector<double> myVectorWindVelocity = vector<double>(3);
	vector<double> myVectorCargoOffset = vector<double>(3);
	vector<double> myVectorCargoRotation = vector<double>(3);

	vector<double> myVectorDisplayOffset = vector<double>(3);
	vector<double> myVectorDisplayAngle = vector<double>(3);
	vector<double> myVectorDrawPosition = vector<double>(3);

	vector<double> myVectorSize = vector<double>(3); //Length / Width / Height
	vector<double> myVectorCrossSection = vector<double>(3);
	vector<double> myVectorCW = vector<double>(3);

	bool myRopeConnected = false;
	bool myFollowOnly = false;
	bool myDrawingEnabled = false;
	bool myInstancedDrawing = false;
	bool myOrientationFollowsDirection = false;
	bool myUpdateHelicopterPosition = true;

	std::chrono::time_point<std::chrono::steady_clock> myStartTime; 
	bool myResetTime = true;

	double myWaterLevel = 0;
	bool myTerrainIsWet = false;
	double myVolume = 0;

	double myHeight = 0.1;
	double myMass = 5.0;
	double myFrictionGlide = 0.35;
	double myFrictionStatic = 0.65;

	double myObjectTerrainLevel = 0;
	bool myTerrainHit = false;
	bool myStopMovement = false;
	double myAirSpeed = 0;
	double myAirResistance = 0;
	double mySpeedStaticFriction = 0.3;
	bool myIsBambiBucket = false;

	bool myBambiBucketRelease = false;
	double myBambiBucketWaterLevel = 0;
	double myBambiBucketWaterWeight = 0;

	vector<double> myVectorForceRope = vector<double>(3);
	vector<double> myVectorAirVelocity = vector<double>(3);
	vector<double> myVectorWaterVelocity = vector<double>(3);
	vector<double> myVectorForceAir = vector<double>(3);
	vector<double> myVectorForceWater = vector<double>(3);
	vector<double> myVectorForceSwim = vector<double>(3);
	vector<double> myVectorForceTotal = vector<double>(3);
	vector<double> myVectorHorizontalVelocity = vector<double>(3);
	vector<double> myVectorForceFriction = vector<double>(3);
	vector<double> myVectorAccTotal = vector<double>(3);
	vector<double> myVectorVelocityDelta = vector<double>(3);
	vector<double> myVectorForceChopper = vector<double>(3);
	vector<double> myVectorMomentumChopper = vector<double>(3);

	vector<double> myVectorForceAirNew = vector<double>(3);
	vector<double> myVectorForceAirCart = vector<double>(3);

	CargoObject(HSL_PlugIn &HSLNew);
	CargoObject(HSL_PlugIn& HSLNew, vector<double> pos, vector<double> vel);
	~CargoObject();

	
	void CalculatePhysics();
	
};

