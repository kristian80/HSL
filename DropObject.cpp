#include "DropObject.h"


DropObject::DropObject()
{
	for (unsigned i = 0; i < myVectorZeroVector.size(); ++i) myVectorZeroVector(i) = 0;

	myVectorVelocity = myVectorZeroVector;
	myVectorForceGravity = myVectorZeroVector;
	myVectorWindVelocity = myVectorZeroVector;

	myVectorSize = myVectorZeroVector;
	myVectorCrossSection = myVectorZeroVector;
	myVectorCW = myVectorZeroVector;

	myVectorDisplayOffset = myVectorZeroVector;
	myVectorDisplayAngle = myVectorZeroVector;

	myVectorAirVelocity = myVectorZeroVector;
	myVectorForceAir = myVectorZeroVector;
	myVectorForceTotal = myVectorZeroVector;
	myVectorAccTotal = myVectorZeroVector;
	myVectorVelocityDelta = myVectorZeroVector;
	myVectorPosition = myVectorZeroVector;

	myVectorSize(0) = 1;
	myVectorSize(1) = 1;
	myVectorSize(2) = 1;

	myVectorCW = myVectorSize * 0.9; //0.9 == Cube
}

DropObject::DropObject(vector<double> pos, vector<double> vel)
{
	for (unsigned i = 0; i < myVectorZeroVector.size(); ++i) myVectorZeroVector(i) = 0;

	myVectorVelocity = myVectorZeroVector;
	myVectorForceGravity = myVectorZeroVector;
	myVectorWindVelocity = myVectorZeroVector;

	myVectorSize = myVectorZeroVector;
	myVectorCrossSection = myVectorZeroVector;
	myVectorCW = myVectorZeroVector;

	myVectorDisplayOffset = myVectorZeroVector;
	myVectorDisplayAngle = myVectorZeroVector;

	myVectorAirVelocity = myVectorZeroVector;
	myVectorForceAir = myVectorZeroVector;
	myVectorForceTotal = myVectorZeroVector;
	myVectorAccTotal = myVectorZeroVector;
	myVectorVelocityDelta = myVectorZeroVector;

	myVectorSize(0) = 1;
	myVectorSize(1) = 1;
	myVectorSize(2) = 1;

	myVectorCW = myVectorSize * 0.9; //0.9 == Cube

	myMass = 997 * 0.000125;
	myVectorSize(0) = 0.05;
	myVectorSize(1) = 0.05;
	myVectorSize(2) = 0.05;
	myVectorCW(0) = 0.45;
	myVectorCW(1) = 0.45;
	myVectorCW(2) = 0.45;

	myVectorPosition = pos;

	myVectorVelocity = vel;
}

DropObject::~DropObject()
{
}

void DropObject::CalculatePhysics(DropHSLData& HSL)
{
	if (HSL.myFrameTime == 0) return;

	myVectorCrossSection(0) = myVectorSize(1) * myVectorSize(2); // Front
	myVectorCrossSection(2) = myVectorSize(0) * myVectorSize(2); // Side
	myVectorCrossSection(1) = myVectorSize(0) * myVectorSize(1); // Top

	myVolume = myVectorSize(0) * myVectorSize(1) * myVectorSize(2);

	//XPLMProbeInfo_t info;
	//info.structSize = sizeof(info);
	//XPLMProbeResult	result = XPLMProbeTerrainXYZ(HSL.myGroundProbe, myVectorPosition(0), myVectorPosition(1), myVectorPosition(2), &info);
	//myObjectTerrainLevel = info.locationY + myHeight;


	//get the air velocity vector: wind velocity is in opposite direction to our own velocity. If we move with the wind, this vector must be zero.
	myVectorAirVelocity = -1 * (myVectorVelocity - HSL.myVectorWindVelocity);
	myAirSpeed = norm_2(myVectorAirVelocity);



	// Get the force of the air
	//myAirResistance = HSL.myLfAirDensity * myVectorCW(0) * myVectorCrossSection(0) * myAirSpeed * myAirSpeed / 2.0; // If we are in the water, we would just need to correct for the density (+ split into air and water part).
	//myVectorForceAir = get_unit_vector(myVectorAirVelocity) * myAirResistance;


	// ToDo: Turn the air vector contrary to the object rotation. Otherwise CW and Crosssection are not correct
	vector<double> normalPosition(3);

	normalPosition(0) = 0;
	normalPosition(1) = 1;
	normalPosition(2) = 0;
	vector<double> normalPositionSphere = XPlaneCartToSphere(normalPosition);

	vector<double> negativeVelocity = -1 * myVectorVelocity;


	


	// To Cart again

	myVectorForceAir(0) = HSL.myLfAirDensity * myVectorCW(0) * myVectorCrossSection(0) * myVectorAirVelocity(0) * myVectorAirVelocity(0) / 2.0;
	myVectorForceAir(1) = HSL.myLfAirDensity * myVectorCW(1) * myVectorCrossSection(1) * myVectorAirVelocity(1) * myVectorAirVelocity(1) / 2.0;
	myVectorForceAir(2) = HSL.myLfAirDensity * myVectorCW(2) * myVectorCrossSection(2) * myVectorAirVelocity(2) * myVectorAirVelocity(2) / 2.0;

	// ToDo: Turn the force vector by the negative amount we turned the air first

	// To Sphere
	// Add ropeUnitSphere 1, 2
	// To Cart


	if (myVectorAirVelocity(0) < 0) myVectorForceAir(0) *= -1;
	if (myVectorAirVelocity(1) < 0) myVectorForceAir(1) *= -1;
	if (myVectorAirVelocity(2) < 0) myVectorForceAir(2) *= -1;

	myAirResistance = norm_2(myVectorForceAir);


	// Get the force of the gravity
	myVectorForceGravity(VERT_AXIS) = HSL.myLfGravitation * myMass;


	// Sum up the forces
	myVectorForceTotal = myVectorForceAir + myVectorForceGravity;


	// Get the acceleration
	if (myMass == 0.0f) return;
	myVectorAccTotal = myVectorForceTotal / myMass;

	// calcualte the new velocity
	myVectorVelocityDelta = myVectorAccTotal * HSL.myFrameTime;
	myVectorVelocity += myVectorVelocityDelta;

	check_nan(myVectorVelocity);
	limit_max(myVectorVelocity, MAX_OBJ_SPEED);


	myVectorPosition += myVectorVelocity * HSL.myFrameTime;
	check_nan(myVectorPosition);

	

	negativeVelocity = -1 * myVectorVelocity;
	vector<double> ropeUnitSphere = XPlaneCartToSphere(get_unit_vector(negativeVelocity));

	normalPositionSphere(1) = ropeUnitSphere(1) - normalPositionSphere(1);
	normalPositionSphere(2) = ropeUnitSphere(2) - normalPositionSphere(2);

	// Using Pitch/Roll
	myVectorDisplayAngle(0) = ropeUnitSphere(1) * 180.0f / M_PI; // Pitch
	myVectorDisplayAngle(1) =ropeUnitSphere(2) * 180.0f / M_PI; // Roll
	myVectorDisplayAngle(2) = 0; //heading
		
	myVectorDisplayOffset = myVectorZeroVector;
}

