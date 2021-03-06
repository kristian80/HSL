/*
 * This file is part of the HSL distribution (https://github.com/kristian80/HSL).
 * Copyright (c) 2019 Kristian80.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 3.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#include "CargoObject.h"
#include "HSL_PlugIn.h"

CargoObject::CargoObject(HSL_PlugIn& HSLNew) :
	//HSL(HSLNew),
	myCargoDataShared(HSLNew.myCargoDataShared)
{
	CARGO_SHM_SECTION_START
	HIGH_PERFORMANCE = myCargoDataShared.myHighPerformace;
	for (unsigned i = 0; i < myVectorZeroVector.size(); ++i) myVectorZeroVector(i) = 0;

	myVectorHelicopterPositionApprox = myVectorZeroVector;
	myVectorHelicopterPositionApproxOld = myVectorZeroVector;
	myVectorHelicopterVelocityApprox = myVectorZeroVector;
	myVectorHelicopterPositionDeviation = myVectorZeroVector;
	myVectorVelocity = myVectorZeroVector;
	myVectorForceGravity = myVectorZeroVector;
	myVectorWindVelocity = myVectorZeroVector;
	myVectorCargoOffset = myVectorZeroVector;
	myVectorCargoRotation = myVectorZeroVector;
	myVectorWaterVelocity = myVectorZeroVector;


	myVectorSize = myVectorZeroVector;
	myVectorCrossSection = myVectorZeroVector;
	myVectorCW = myVectorZeroVector;

	myVectorDisplayOffset = myVectorZeroVector;
	myVectorDisplayAngle = myVectorZeroVector;

	myVectorForceRope = myVectorZeroVector;
	myVectorAirVelocity = myVectorZeroVector;
	myVectorForceAir = myVectorZeroVector;
	myVectorForceWater = myVectorZeroVector;
	myVectorForceTotal = myVectorZeroVector;
	myVectorHorizontalVelocity = myVectorZeroVector;
	myVectorForceFriction = myVectorZeroVector;
	myVectorAccTotal = myVectorZeroVector;
	myVectorVelocityDelta = myVectorZeroVector;
	myVectorForceChopper = myVectorZeroVector;
	myVectorMomentumChopper = myVectorZeroVector;
	myVectorForceSwim = myVectorZeroVector;

	myVectorForceAirNew = myVectorZeroVector;
	myVectorForceAirCart = myVectorZeroVector;
	myVectorDrawPosition = myVectorZeroVector;

	myVectorExternalForce = myVectorZeroVector;
	myVectorForceOperator = myVectorZeroVector;

	myVectorSize(0) = 1;
	myVectorSize(1) = 1;
	myVectorSize(2) = 1;

	myVectorCW = myVectorSize * 0.9; //0.9 == Cube
}

CargoObject::CargoObject(HSL_PlugIn& HSLNew, vector<double> pos, vector<double> vel) : 
	//HSL(HSLNew),
	myCargoDataShared(HSLNew.myCargoDataShared)
{
	CARGO_SHM_SECTION_START

	for (unsigned i = 0; i < myVectorZeroVector.size(); ++i) myVectorZeroVector(i) = 0;

	myVectorHelicopterPositionApprox = myVectorZeroVector;
	myVectorHelicopterVelocityApprox = myVectorZeroVector;
	myVectorHelicopterPositionApproxOld = myVectorZeroVector;
	myVectorHelicopterPositionDeviation = myVectorZeroVector;
	myVectorVelocity = myVectorZeroVector;
	myVectorForceGravity = myVectorZeroVector;
	myVectorWindVelocity = myVectorZeroVector;
	myVectorCargoOffset = myVectorZeroVector;
	myVectorCargoRotation = myVectorZeroVector;
	myVectorWaterVelocity = myVectorZeroVector;


	myVectorSize = myVectorZeroVector;
	myVectorCrossSection = myVectorZeroVector;
	myVectorCW = myVectorZeroVector;

	myVectorDisplayOffset = myVectorZeroVector;
	myVectorDisplayAngle = myVectorZeroVector;

	myVectorForceRope = myVectorZeroVector;
	myVectorAirVelocity = myVectorZeroVector;
	myVectorForceAir = myVectorZeroVector;
	myVectorForceWater = myVectorZeroVector;
	myVectorForceTotal = myVectorZeroVector;
	myVectorHorizontalVelocity = myVectorZeroVector;
	myVectorForceFriction = myVectorZeroVector;
	myVectorAccTotal = myVectorZeroVector;
	myVectorVelocityDelta = myVectorZeroVector;
	myVectorForceChopper = myVectorZeroVector;
	myVectorMomentumChopper = myVectorZeroVector;
	myVectorForceSwim = myVectorZeroVector;
	myVectorDrawPosition = myVectorZeroVector;
	myVectorForceOperator = myVectorZeroVector;

	myVectorSize(0) = 1;
	myVectorSize(1) = 1;
	myVectorSize(2) = 1;

	myVectorCW = myVectorSize * 0.9; //0.9 == Cube

	myRopeConnected = false;
	myDrawingEnabled = true;
	myInstancedDrawing = true;
	myOrientationFollowsDirection = true;
	myHeight = 0.05;



	myHeight = 0.05;
	myMass = 0.000125;
	myVectorSize(0) = 0.05;
	myVectorSize(1) = 0.05;
	myVectorSize(2) = 0.05;
	myVectorCW(0) = 0.45;
	myVectorCW(1) = 0.45;
	myVectorCW(2) = 0.45;
	myHeight = 0.05;
	myHeight = 0.05;

	myVectorPosition = pos;

	myVectorVelocity = vel;
}

CargoObject::~CargoObject()
{
}

void CargoObject::CalculatePhysics()
{
	CARGO_SHM_SECTION_START

	auto timeNow = std::chrono::steady_clock::now();

	if (myCargoDataShared.mySlingLineEnabled == false)
	{
		myResetTime = true;
		return;
	}

	if ((myCargoDataShared.myLiPause > 0) || (myCargoDataShared.myIsInReplay > 0))
	{
		myResetTime = true;
		return;
	}
	
	
	
	if (myResetTime == true)
	{
		myVectorHelicopterVelocityApprox = myCargoDataShared.myVectorHelicopterVelocity;

		myStartTime = timeNow;
		myResetTime = false;
	}


	// Not more than 100 kHz, otherwise values might get too small
	double frameTimeNano = (double) std::chrono::duration_cast<std::chrono::nanoseconds>(timeNow - myStartTime).count();// *myCargoDataShared.myLfTimeActual;

	double minFrameTimeNano = 1000000000.0 /(sqrt(myCargoDataShared.myRopeK / myMass) / (2.0 * M_PI) * 20);

	if (minFrameTimeNano > (1000000000.0 / 250.0))
		minFrameTimeNano = (1000000000.0 / 250.0);

	if (frameTimeNano < 100000) 
	//if (frameTimeNano < minFrameTimeNano)
		return;
	
	double frameTime = frameTimeNano / (1000000000.0);
	myStartTime = timeNow;

	myCargoDataShared.myFrameTimeMax = max(myCargoDataShared.myFrameTimeMax, frameTimeNano);
	
	//double frameTime = myCargoDataShared.myFrameTime;

	if (frameTime == 0) return;



	// Cargo Values

	myVectorCrossSection(0) = myVectorSize(1) * myVectorSize(2); // Front
	myVectorCrossSection(2) = myVectorSize(0) * myVectorSize(2); // Side
	myVectorCrossSection(1) = myVectorSize(0) * myVectorSize(1); // Top

	myVolume = myVectorSize(0) * myVectorSize(1) * myVectorSize(2);

	myVectorWindVelocity = myCargoDataShared.myVectorWindVelocity;


	// Update Position and Velocity
	if (myStopMovement == false)
	{
		myVectorHelicopterPositionApprox += myCargoDataShared.myVectorHelicopterVelocity * frameTime;
		myVectorHelicopterVelocityApprox += myCargoDataShared.myVectorHelicopterAcceleration * frameTime;
	}

	if ((myFollowOnly == true) && (myRopeConnected == true))
	{
		myVectorVelocity = (myCargoDataShared.myVectorHookPosition - myVectorPosition) / frameTime;
		myVectorPosition = myCargoDataShared.myVectorHookPosition;
		myVectorHelicopterPositionApprox = myCargoDataShared.myVectorHelicopterPosition;
		myVectorHelicopterVelocityApprox = myCargoDataShared.myVectorHelicopterVelocity;
		return;
	}

	// Check for unconsistent Movement
	vector<double> heliMovement = myVectorHelicopterPositionApprox - myVectorHelicopterPositionApproxOld;

	double heliDistance = norm_2(heliMovement);
	double heliSpeedDistance = norm_2(myVectorHelicopterVelocityApprox) * frameTime;


	myVectorHelicopterPositionApproxOld = myVectorHelicopterPositionApprox;



	// If we are below the ground, correct vertical position and velocity
	if ((myVectorPosition(VERT_AXIS) <= (myObjectTerrainLevel+0.0001)) && (myTerrainIsWet == false))
	{
		myTerrainHit = true;

		myVectorPosition(VERT_AXIS) = myObjectTerrainLevel;
		if (myVectorVelocity(VERT_AXIS) < 0) myVectorVelocity(VERT_AXIS) = 0; //no velocity targeting below ground

	}
	else if ((myVectorPosition(VERT_AXIS) <= myObjectTerrainLevel) && (myTerrainIsWet == true))
	{
		if (myVectorSize(2) == 0) return;

		double sink = myObjectTerrainLevel - myVectorPosition(VERT_AXIS);
		myWaterLevel = sink / myVectorSize(2);
		if (myWaterLevel > 1) myWaterLevel = 1;
		
	}

	if (myIsBambiBucket == true)
	{
		if (myBambiBucketWaterLevel < myWaterLevel) myBambiBucketWaterLevel = myWaterLevel;
		myBambiBucketWaterWeight = myBambiBucketWaterLevel * myVolume * HSL_Data::density_water;
	}

	if (myIsBambiBucket == false)
	{
		myBambiBucketWaterLevel = 0;
		myBambiBucketWaterWeight = 0;
	}

	if ((myBambiBucketRelease == true) && (myBambiBucketWaterWeight > 0))
	{
		double releasedWater = myCargoDataShared.myBambiBucketWaterFlow * frameTime;
		myBambiBucketWaterWeight -= releasedWater;
		if (myBambiBucketWaterWeight < 0) myBambiBucketWaterWeight = 0;
		
		if (myVolume == 0) myBambiBucketWaterLevel = 0;
		else myBambiBucketWaterLevel = myBambiBucketWaterWeight / (myVolume * HSL_Data::density_water);
	}


	// Get the rope vector and length. Store length in 2nd variable, as we need old+new values to compute the rope stretch speed for the damping factor
	
	myCargoDataShared.myVectorRope = myVectorHelicopterPositionApprox - myCargoDataShared.myVectorHookPosition;
	myCargoDataShared.myNewRopeLength = norm_2(myCargoDataShared.myVectorRope);

	// Check for Div/0
	if (myCargoDataShared.myRopeLengthNormal == 0.0) return;

	// Compute the length stretch
	myCargoDataShared.myRopeStretchRelative = (myCargoDataShared.myNewRopeLength / myCargoDataShared.myRopeLengthNormal) - 1;
	myCargoDataShared.myRopeForceScalar = 0.0;


	// if rope is streched, calcuate the rope force
	if ((myCargoDataShared.myRopeStretchRelative > 0.0) && (myCargoDataShared.myRopeRuptured == false) && (myRopeConnected == true))
	{
		myCargoDataShared.myRopeLengthDelta = myCargoDataShared.myNewRopeLength - myCargoDataShared.myCurrentRopeLength;

		// Check for Div/0
		if (frameTime == 0.0) return;

		myCargoDataShared.myRopeStretchSpeed = myCargoDataShared.myRopeLengthDelta / frameTime;

		myCargoDataShared.myRopeCorrectedD = 0;

		//myCargoDataShared.myDebugValue3 = 0;

		if (myCargoDataShared.myRopeStretchSpeed < 0.0)
		{
			//myCargoDataShared.myDebugValue3 = 200;
			myCargoDataShared.myRopeCorrectedD = myCargoDataShared.myRopeDamping * 2 * sqrt((myMass + myBambiBucketWaterWeight) * myCargoDataShared.myRopeK);
		}

		check_nan(myCargoDataShared.myRopeCorrectedD);

		double ropeForceDamping = myCargoDataShared.myRopeCorrectedD * myCargoDataShared.myRopeStretchSpeed;
		double ropeForceStrech = myCargoDataShared.myRopeK * myCargoDataShared.myRopeStretchRelative;

		myCargoDataShared.myRopeForceScalar = ropeForceStrech + ropeForceDamping;
		
		if (myCargoDataShared.myRopeForceScalar < 0.0)
		{
			myCargoDataShared.myRopeForceScalar = 0;  // Rope can never apply negative forces, our damping could ;-)
		}

		myCargoDataShared.myCurrentRopeLength = myCargoDataShared.myNewRopeLength;

	}
	else
	{
		myCargoDataShared.myCurrentRopeLength = myCargoDataShared.myRopeLengthNormal;
	}

	// get the vector for the rope force
	myVectorForceRope = get_unit_vector(myCargoDataShared.myVectorRope) * myCargoDataShared.myRopeForceScalar;

	//get the air velocity vector: wind velocity is in opposite direction to our own velocity. If we move with the wind, this vector must be zero.
	myVectorAirVelocity = -1 * (myVectorVelocity - myVectorWindVelocity);
	myAirSpeed = norm_2(myVectorAirVelocity);

	

	// Get the force of the air
	//myAirResistance = HSL.myLfAirDensity * myVectorCW(0) * myVectorCrossSection(0) * myAirSpeed * myAirSpeed / 2.0; // If we are in the water, we would just need to correct for the density (+ split into air and water part).
	//myVectorForceAir = get_unit_vector(myVectorAirVelocity) * myAirResistance;

	vector<double> vectorRopeStart = myVectorHelicopterPositionApprox - myCargoDataShared.myVectorHookPosition;
	vector<double> vectorRopeUnitStart = get_unit_vector(vectorRopeStart);

	vector<double> ropeUnitSphere = XPlaneCartToSphere(vectorRopeUnitStart);

	//vector<double> negativeVelocity = -1 * myVectorVelocity;


	vector<double> vectorAirVelocityTurnedSphere = XPlaneCartToSphere(myVectorAirVelocity);

	vectorAirVelocityTurnedSphere(1) += ropeUnitSphere(1);
	vectorAirVelocityTurnedSphere(2) += ropeUnitSphere(2);

	double angle1 = ropeUnitSphere(1);
	double angle2 = ropeUnitSphere(2);

	double n1 = norm_2(myVectorAirVelocity);
	double n2 = vectorAirVelocityTurnedSphere(0);


	vector<double> vectorAirVelocityTurnedCart = XPlaneSphereToCart(vectorAirVelocityTurnedSphere);

	double n3 = norm_2(vectorAirVelocityTurnedCart);

	myVectorForceAirCart = myVectorZeroVector;




	myVectorForceAirCart(0) = (1.0 - myWaterLevel) * myCargoDataShared.myLfAirDensity * myVectorCW(0) * myVectorCrossSection(0) * vectorAirVelocityTurnedCart(0) * vectorAirVelocityTurnedCart(0) / 2.0;
	myVectorForceAirCart(1) = (1.0 - myWaterLevel) * myCargoDataShared.myLfAirDensity * myVectorCW(1) * myVectorCrossSection(1) * vectorAirVelocityTurnedCart(1) * vectorAirVelocityTurnedCart(1) / 2.0;
	myVectorForceAirCart(2) = (1.0 - myWaterLevel) * myCargoDataShared.myLfAirDensity * myVectorCW(2) * myVectorCrossSection(2) * vectorAirVelocityTurnedCart(2) * vectorAirVelocityTurnedCart(2) / 2.0;

	if (vectorAirVelocityTurnedCart(0) < 0) myVectorForceAirCart(0) *= -1;
	if (vectorAirVelocityTurnedCart(1) < 0) myVectorForceAirCart(1) *= -1;
	if (vectorAirVelocityTurnedCart(2) < 0) myVectorForceAirCart(2) *= -1;

	double v1 = norm_2(myVectorForceAirCart);

	vector<double> vectorForceAirSphere = XPlaneCartToSphere(myVectorForceAirCart);

	vectorForceAirSphere(1) -= ropeUnitSphere(1);
	vectorForceAirSphere(2) -= ropeUnitSphere(2);

	double v2 = vectorForceAirSphere(0);

	myVectorForceAirNew = XPlaneSphereToCart(vectorForceAirSphere);

	double v3 = norm_2(myVectorForceAirNew);


	//myCargoDataShared.myVectorDebug = myVectorForceAirNew;
	myVectorForceAir(0) = (1.0 - myWaterLevel) * myCargoDataShared.myLfAirDensity * myVectorCW(0) * myVectorCrossSection(0) * myVectorAirVelocity(0) * myVectorAirVelocity(0) / 2.0;
	myVectorForceAir(1) = (1.0 - myWaterLevel) * myCargoDataShared.myLfAirDensity * myVectorCW(1) * myVectorCrossSection(1) * myVectorAirVelocity(1) * myVectorAirVelocity(1) / 2.0;
	myVectorForceAir(2) = (1.0 - myWaterLevel) * myCargoDataShared.myLfAirDensity * myVectorCW(2) * myVectorCrossSection(2) * myVectorAirVelocity(2) * myVectorAirVelocity(2) / 2.0;

	if (myVectorAirVelocity(0) < 0) myVectorForceAir(0) *= -1;
	if (myVectorAirVelocity(1) < 0) myVectorForceAir(1) *= -1;
	if (myVectorAirVelocity(2) < 0) myVectorForceAir(2) *= -1;

	double v4 = norm_2(myVectorForceAir);

	myAirResistance = norm_2(myVectorForceAir);

	myVectorWaterVelocity = -1 * myVectorVelocity;

	myVectorForceWater(0) = myWaterLevel * HSL_Data::density_water * myVectorCW(0) * myVectorCrossSection(0) * myVectorWaterVelocity(0) * myVectorWaterVelocity(0) / 2.0;
	myVectorForceWater(1) = myWaterLevel * HSL_Data::density_water * myVectorCW(1) * myVectorCrossSection(1) * myVectorWaterVelocity(1) * myVectorWaterVelocity(1) / 2.0;
	myVectorForceWater(2) = myWaterLevel * HSL_Data::density_water * myVectorCW(2) * myVectorCrossSection(2) * myVectorWaterVelocity(2) * myVectorWaterVelocity(2) / 2.0;

	if (myVectorWaterVelocity(0) < 0) myVectorForceWater(0) *= -1;
	if (myVectorWaterVelocity(1) < 0)
	{
		myVectorForceWater(1) *= -1;
		if (myWaterLevel <= 1)       myVectorForceWater(1) = 0;
	}
	//else if (myWaterLevel <= 1)       myVectorForceWater(1) = 0; //When pulling out, top has only water resistance when the object is fully under the water
	if (myVectorWaterVelocity(2) < 0) myVectorForceWater(2) *= -1;

	double waterResistance = norm_2(myVectorForceWater);

	myVectorForceSwim(VERT_AXIS) = 0;
	if (myIsBambiBucket == false) myVectorForceSwim(VERT_AXIS) = -1 * HSL_Data::density_water * myVolume * myWaterLevel * myCargoDataShared.myLfGravitation;


	// Get the force of the gravity
	myVectorForceGravity(VERT_AXIS) = myCargoDataShared.myLfGravitation * (myMass + (myBambiBucketWaterWeight * (1.0 - myWaterLevel)));

	// New: 
	double forceGravity = myCargoDataShared.myLfGravitation * (myMass + (myBambiBucketWaterWeight * (1.0 - myWaterLevel)));
	myVectorForceGravity = myCargoDataShared.myVectorDownDirection * forceGravity;


	// Get the force of the winch operator trying to stop the load oscillation

	myVectorForceOperator = myVectorZeroVector;

	
	{
		myVectorForceOperator = -1 * (myVectorVelocity - myVectorHelicopterVelocityApprox) * myMass / frameTime; // F= v * m / t
		//myVectorForceOperator(VERT_AXIS) = 0; // Only horizontally

		double dampingForceMax = myCargoDataShared.myRopeArtificialDampingForce;// * max(1, myCargoDataShared.myRopeLengthNormal); //5N per meter cable

		// Limit force
		if (myCargoDataShared.myRopeLengthNormal <= myCargoDataShared.myRopeOperatorDampingLength)
			dampingForceMax = myCargoDataShared.myRopeOperatorDampingForce;


		if ((norm_2(myVectorForceOperator) > dampingForceMax))
		{
			//if ((myCargoDataShared.myRopeLengthNormal > 0.2))
			myVectorForceOperator = get_unit_vector(myVectorForceOperator) * dampingForceMax;
		}

		myCargoDataShared.myDebugValue3 = norm_2(myVectorForceOperator);
	}

	// Sum up the forces
	myVectorForceTotal = myVectorForceRope + myVectorForceAirNew + myVectorForceGravity + myVectorForceWater + myVectorForceSwim + myVectorExternalForce + myVectorForceOperator;
	myStopMovement = false;
	// If we are on the ground and not pulled up, we need to compute the friction
	if ((myTerrainHit == true) && (myTerrainIsWet == false))
	{
		if (myVectorForceTotal(VERT_AXIS) < 0)
		{
			// we did that before, right?
			myVectorHorizontalVelocity = myVectorVelocity;
			myVectorHorizontalVelocity(VERT_AXIS) = 0;

			// compute glide friction
			myVectorForceFriction = get_unit_vector(myVectorHorizontalVelocity) * myVectorForceTotal(VERT_AXIS) * myFrictionGlide;

			// Compute the speed, where the static friction would stop us within a frame. Multiply with 3 to be on the safe side for fps drop
			if ((myMass + myBambiBucketWaterWeight) == 0.0) return;
			mySpeedStaticFriction = std::abs(3 * frameTime * myVectorForceTotal(VERT_AXIS) * myFrictionStatic / (myMass + myBambiBucketWaterWeight)); // adapt stop speed to frame rate

			// if we are below static friction speed
			if (norm_2(myVectorHorizontalVelocity) < mySpeedStaticFriction)
			{
				// compute the static friction (again...)
				myVectorForceFriction = get_unit_vector(myVectorHorizontalVelocity) * myVectorForceTotal(VERT_AXIS) * myFrictionStatic;
				myVectorForceTotal(VERT_AXIS) = 0; //Stop forces pulling below ground

				// If all forces are less than the static friction, we will not move
				if (norm_2(myVectorForceFriction) > norm_2(myVectorForceTotal))
				{
					// Stop movement
					myStopMovement = true;
					myVectorVelocity = myVectorZeroVector;
					myVectorForceTotal = myVectorZeroVector;
					myVectorForceFriction = myVectorZeroVector;
				}
				else
				{
					int i;
					i = 0;
				}
			}

			// Add friction to total foces
			myVectorForceTotal += myVectorForceFriction;
			myVectorForceTotal(VERT_AXIS) = 0; //Stop forces pulling below ground
		}
	}


	// Get the acceleration
	if ((myMass + myBambiBucketWaterWeight) == 0.0) return;
	myVectorAccTotal = myVectorForceTotal / (myMass + myBambiBucketWaterWeight);

	// calcualte the new velocity
	myVectorVelocityDelta = myVectorAccTotal * frameTime;
	myVectorVelocity += myVectorVelocityDelta;

	check_nan(myVectorVelocity);
	limit_max(myVectorVelocity, MAX_OBJ_SPEED);

	if (myCargoDataShared.myPhysicsEnabled == true)
	{
		// When the rope is below the threshold, we stabilize it directly below the winch
		if ((myCargoDataShared.myRopeLengthNormal < HSL_Data::rope_lenght_static) && (myRopeConnected == true) && (myCargoDataShared.myRopeRuptured == false))
		{
			myVectorPosition = myVectorHelicopterPositionApprox;
			myVectorPosition(1) -= myCargoDataShared.myRopeLengthNormal;

			myVectorAccTotal = myVectorZeroVector;
			myVectorVelocityDelta = myVectorZeroVector;
			myVectorVelocity = myVectorHelicopterVelocityApprox;

			myVectorForceTotal = myVectorZeroVector;
			myVectorForceRope = myVectorZeroVector;
		}
		else
		{
			myVectorPosition += myVectorVelocity * frameTime;
 		}

		

		check_nan(myVectorPosition);

		// Compute the force in aircraft coordinates. This just turns the vector in the right direction.
		myVectorForceChopper = HSL_PlugIn::TurnWorldToAircraft(-1 * myVectorForceRope, myCargoDataShared);

		// Momentum in Center of Gravity = rxF
		myVectorMomentumChopper = cross_product(myCargoDataShared.myVectorWinchPosition, myVectorForceChopper);
	}

	// Only if we are really intended to apply forces to the chopper
	if ((myRopeConnected == true) && (myFollowOnly == false) && (myCargoDataShared.myRopeRuptured == false))
	{

		check_nan(myVectorForceChopper);
		check_nan(myVectorMomentumChopper);

		ForceData forceChopper;
		forceChopper.myVectorForce = myVectorForceChopper;
		forceChopper.myVectorMomentum = myVectorMomentumChopper;
		forceChopper.myTimeApplied = frameTime;
		myCargoDataShared.myHelicopterForceQueue.push(forceChopper);
	}
	else
	{
		ForceData forceChopper;
		forceChopper.myVectorForce = myVectorZeroVector;
		forceChopper.myVectorMomentum = myVectorZeroVector;
		forceChopper.myTimeApplied = frameTime;
		myCargoDataShared.myHelicopterForceQueue.push(forceChopper);
	}


	if ((myCargoDataShared.myRopeRuptured == false) && (myRopeConnected == true) && (myFollowOnly == false))
	{
		myCargoDataShared.myVectorHookPosition = myVectorPosition;
	}

	// Calculate Object Offset
	double scalarObjectOffset = norm_2(myVectorCargoOffset);
	vector<double>  myVectorFinalRope = myVectorHelicopterPositionApprox - myCargoDataShared.myVectorHookPosition;
	vector<double> vectorRopeUnit = get_unit_vector(myVectorFinalRope);


	// If ruptured, keep offset and angle

	if (((myCargoDataShared.myRopeRuptured == false) && (myRopeConnected == true)) || (myOrientationFollowsDirection == true))
	{
		vector<double> normalPosition(3);

		normalPosition(0) = 0;
		normalPosition(1) = 1;
		normalPosition(2) = 0;

		vector<double> normalPositionSphere = XPlaneCartToSphere(normalPosition);
		vector<double> ropeUnitSphere = XPlaneCartToSphere(vectorRopeUnit);

		vector<double> negativeVelocity = -1 * myVectorVelocity;

		if (myOrientationFollowsDirection == true) ropeUnitSphere = XPlaneCartToSphere(get_unit_vector(negativeVelocity));

		normalPositionSphere(1) = ropeUnitSphere(1) - normalPositionSphere(1);
		normalPositionSphere(2) = ropeUnitSphere(2) - normalPositionSphere(2);


		if (myTerrainHit == false)
		{
			// Using Pitch/Roll
			myVectorDisplayAngle(0) = myVectorCargoRotation(0) + ropeUnitSphere(1) * 180.0 / M_PI; // Pitch
			myVectorDisplayAngle(1) = myVectorCargoRotation(1) + ropeUnitSphere(2) * 180.0 / M_PI; // Roll
			myVectorDisplayAngle(2) = myVectorCargoRotation(2); //heading
		}
		else
		{
			myVectorDisplayAngle = myVectorCargoRotation;
		}



		// Object Offset
		if (scalarObjectOffset > 0)
		{

			vector<double> myVectorCargoOffsetRotated = myVectorCargoOffset;

			double length = sqrt((myVectorCargoOffset(0) * myVectorCargoOffset(0)) + (myVectorCargoOffset(2) * myVectorCargoOffset(2)));
			double angle = atan2(myVectorCargoOffset(2), myVectorCargoOffset(0));

			angle += myVectorDisplayAngle(2) * M_PI / 180.0;

			//myVectorCargoOffsetRotated(0) = length * cos(angle);
			//myVectorCargoOffsetRotated(2) = length * sin(angle);

			vector<double> vectorObjectOffsetSphere = XPlaneCartToSphere(myVectorCargoOffsetRotated);
			if (myTerrainHit == false)
			{
				vectorObjectOffsetSphere(1) += ropeUnitSphere(1);
				vectorObjectOffsetSphere(2) += ropeUnitSphere(2);
			}

			myVectorDisplayOffset = XPlaneSphereToCart(vectorObjectOffsetSphere);
		}
		else
		{
			myVectorDisplayOffset = myVectorZeroVector;
		}

	}
	else
	{
		if (myTerrainHit == true)
		{
			myVectorDisplayOffset = myVectorCargoOffset;
			myVectorDisplayAngle = myVectorCargoRotation;
		}
	}

	if (myUpdateHelicopterPosition == true)
	{
		double dev_old = norm_2(myVectorHelicopterPositionDeviation);
		myVectorHelicopterPositionDeviation = myCargoDataShared.myVectorHelicopterPosition - myVectorHelicopterPositionApprox;
		
		myVectorHelicopterVelocityApprox = myCargoDataShared.myVectorHelicopterVelocity;

		if ((myTerrainHit == true) || (myRopeConnected == false))
		{
			if (dev_old > 0) 
				myVectorPosition += myVectorHelicopterPositionDeviation;
			myVectorHelicopterPositionApprox = myCargoDataShared.myVectorHelicopterPosition;
			myVectorHelicopterPositionDeviation = myVectorZeroVector;
		}
		else
		{
			//myVectorHelicopterVelocityApprox += myVectorHelicopterPositionDeviation * 10.0;
			//myVectorHelicopterPositionDeviation = myCargoDataShared.myVectorWinchPosition - myVectorHelicopterPositionApprox;
		}

		// Move the Object by the deviation of the helicopter position approximation
		
		//myVectorPosition += myVectorHelicopterPositionDeviation;
		//myVectorHelicopterPositionApprox += myVectorHelicopterPositionDeviation;

		// Assign New Values
		
		
		myUpdateHelicopterPosition = false;
	}

}
