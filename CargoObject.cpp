#include "CargoObject.h"
#include "HSL_PlugIn.h"

CargoObject::CargoObject(HSL_PlugIn& HSLNew) :
	HSL(HSLNew)
{
	for (unsigned i = 0; i < myVectorZeroVector.size(); ++i) myVectorZeroVector(i) = 0;

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

	myVectorSize(0) = 1;
	myVectorSize(1) = 1;
	myVectorSize(2) = 1;

	myVectorCW = myVectorSize * 0.9; //0.9 == Cube
}

CargoObject::CargoObject(HSL_PlugIn& HSLNew, vector<float> pos, vector<float> vel) : 
	HSL(HSLNew)
{
	for (unsigned i = 0; i < myVectorZeroVector.size(); ++i) myVectorZeroVector(i) = 0;

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
	if (HSL.myFrameTime == 0) return;

	myVectorCrossSection(0) = myVectorSize(1) * myVectorSize(2); // Front
	myVectorCrossSection(2) = myVectorSize(0) * myVectorSize(2); // Side
	myVectorCrossSection(1) = myVectorSize(0) * myVectorSize(1); // Top

	myVolume = myVectorSize(0) * myVectorSize(1) * myVectorSize(2);

	myVectorWindVelocity = HSL.myVectorWindVelocity;

	if ((myFollowOnly == true) && (myRopeConnected == true))
	{
		myVectorVelocity = (HSL.myVectorHookPosition - myVectorPosition) / HSL.myFrameTime;
		myVectorPosition = HSL.myVectorHookPosition;
		return;
	}


	// Check if load is on the ground
	myTerrainHit = false;
	myWaterLevel = 0;
	XPLMProbeInfo_t info;
	info.structSize = sizeof(info);
	XPLMProbeResult	result = XPLMProbeTerrainXYZ(HSL.myGroundProbe, myVectorPosition(0), myVectorPosition(1), myVectorPosition(2), &info);
	myObjectTerrainLevel = info.locationY + myHeight;

	// If we are below the ground, correct vertical position and velocity
	if ((myVectorPosition(VERT_AXIS) <= myObjectTerrainLevel) && (info.is_wet == false))
	{
		myTerrainHit = true;

		myVectorPosition(VERT_AXIS) = myObjectTerrainLevel;
		if (myVectorVelocity(VERT_AXIS) < 0) myVectorVelocity(VERT_AXIS) = 0; //no velocity targeting below ground

	}
	else if ((myVectorPosition(VERT_AXIS) <= myObjectTerrainLevel) && (info.is_wet > 0))
	{
		if (myVectorSize(2) == 0) return;

		float sink = myObjectTerrainLevel - myVectorPosition(VERT_AXIS);
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
		float releasedWater = HSL.myBambiBucketWaterFlow * HSL.myFrameTime;
		myBambiBucketWaterWeight -= releasedWater;
		if (myBambiBucketWaterWeight < 0) myBambiBucketWaterWeight = 0;
		
		if (myVolume == 0) myBambiBucketWaterLevel = 0;
		else myBambiBucketWaterLevel = myBambiBucketWaterWeight / (myVolume * HSL_Data::density_water);
	}


	// Get the rope vector and length. Store length in 2nd variable, as we need old+new values to compute the rope stretch speed for the damping factor
	
	HSL.myVectorRope = HSL.myVectorHelicopterPosition - HSL.myVectorHookPosition;
	HSL.myNewRopeLength = norm_2(HSL.myVectorRope);

	// Check for Div/0
	if (HSL.myRopeLengthNormal == 0.0f) return;

	// Compute the length stretch
	HSL.myRopeStretchRelative = (HSL.myNewRopeLength / HSL.myRopeLengthNormal) - 1;
	HSL.myRopeForceScalar = 0.0f;


	// if rope is streched, calcuate the rope force
	if ((HSL.myRopeStretchRelative > 0.0f) && (HSL.myRopeRuptured == false) && (myRopeConnected == true))
	{
		HSL.myRopeLengthDelta = HSL.myNewRopeLength - HSL.myCurrentRopeLength;

		// Check for Div/0
		if (HSL.myFrameTime == 0.0f) return;

		HSL.myRopeStretchSpeed = HSL.myRopeLengthDelta / HSL.myFrameTime;

		if (HSL.myRopeStretchSpeed < 0.0f) HSL.myDebugValue3 = 100;
		HSL.myRopeCorrectedD = HSL.myRopeDamping * 2 * sqrt((myMass + myBambiBucketWaterWeight) * HSL.myRopeK);

		check_nan(HSL.myRopeCorrectedD);

		float ropeForceDamping = HSL.myRopeCorrectedD * HSL.myRopeStretchSpeed;
		float ropeForceStrech = HSL.myRopeK * HSL.myRopeStretchRelative;

		// Apply damping only when rope is retracting. 
		// Otherwise it is difficult to ensure that it's not the damping which is shooting the load back at the helicopter (which damping could never do).
		// That's once of the problems that it's a difference and not a differential equations.
		/*if (ropeForceDamping > 0.0f)
		{
			// Damping can never be stronger than spring force
			if ((-1 * ropeForceDamping) > ropeForceStrech) ropeForceDamping = -1 * ropeForceStrech;

		}
		else
		{
			myRopeForceScalar = ropeForceStrech;
		}*/

		HSL.myRopeForceScalar = ropeForceStrech + ropeForceDamping;
		if (HSL.myRopeForceScalar < 0.0f)
		{
			HSL.myRopeForceScalar = 0;  // Rope can never apply negative forces, our damping could ;-)
			HSL.myDebugValue2 = 100.f;
		}

		// If we are still in the stretching phase, we want to make sure we do not shoot down the helicopter after an fps lag
		// Hence, we make sure that we will have at least another computation in the negative stretching phase

		// Debug: We just put critical damping on the rope. FPS Stuttering is just too much
		//if (myRopeStretchSpeed > 0.0f)
		{
			// force to stop the object at the current point in time
			float ropeStopForce = HSL.myRopeStretchSpeed * (myMass + myBambiBucketWaterWeight);

			// calcuate the required speed to reach the unstreched rope length in a single frame (= excactly a bit too much):
			float ropeTotalStretchMeters = HSL.myNewRopeLength - HSL.myRopeLengthNormal;
			float ropeEscapeSpeed = ropeTotalStretchMeters / HSL.myFrameTime;


			if (HSL.myMaxAccRopeFactor != 0.0f)
			{
				// divide this speed by the myMaxAccRopeFactor to be within limits
				ropeEscapeSpeed /= HSL.myMaxAccRopeFactor;

				//cacluate the max force to stop and accelerate in the other direction and still be within the streched rope
				float ropeMaxEscapeForce = ropeStopForce + (ropeEscapeSpeed * (myMass + myBambiBucketWaterWeight));

				HSL.myDebugValue1 = ropeMaxEscapeForce;

				//limit the force to this factor
				if ((HSL.myRopeForceScalar > ropeMaxEscapeForce))
				{
					HSL.myDebugStatement = false;
					HSL.myRopeForceScalar = ropeMaxEscapeForce;
				}
			}

		}

		// Did we rupture the rope?
		if (HSL.myRopeRuptureForce <= HSL.myRopeForceScalar)
		{
			HSL.myRopeRuptured = true;
			HSL.myRopeForceScalar = 0.0f;
			HSL.myCurrentRopeLength = HSL.myRopeLengthNormal;
		}
		else
		{
			// update the last-cycle value
			HSL.myCurrentRopeLength = HSL.myNewRopeLength;
		}
	}
	else
	{
		HSL.myCurrentRopeLength = HSL.myRopeLengthNormal;
	}

	// get the vector for the rope force
	myVectorForceRope = get_unit_vector(HSL.myVectorRope) * HSL.myRopeForceScalar;

	//get the air velocity vector: wind velocity is in opposite direction to our own velocity. If we move with the wind, this vector must be zero.
	myVectorAirVelocity = -1 * (myVectorVelocity - myVectorWindVelocity);
	myAirSpeed = norm_2(myVectorAirVelocity);

	

	// Get the force of the air
	//myAirResistance = HSL.myLfAirDensity * myVectorCW(0) * myVectorCrossSection(0) * myAirSpeed * myAirSpeed / 2.0; // If we are in the water, we would just need to correct for the density (+ split into air and water part).
	//myVectorForceAir = get_unit_vector(myVectorAirVelocity) * myAirResistance;

	/*vector<float> vectorRopeStart = HSL.myVectorHelicopterPosition - HSL.myVectorHookPosition;
	vector<float> vectorRopeUnitStart = get_unit_vector(vectorRopeStart);

	vector<float> ropeUnitSphere = XPlaneCartToSphere(vectorRopeUnitStart);

	//vector<float> negativeVelocity = -1 * myVectorVelocity;


	vector<float> vectorAirVelocityTurnedSphere = XPlaneCartToSphere(myVectorAirVelocity);

	vectorAirVelocityTurnedSphere(1) -= ropeUnitSphere(1);
	vectorAirVelocityTurnedSphere(2) -= ropeUnitSphere(2);


	vector<float> vectorAirVelocityTurnedCart = XPlaneSphereToCart(vectorAirVelocityTurnedSphere);

	vector<float> vectorForceAirCart = myVectorZeroVector;


	vectorForceAirCart(0) = HSL.myLfAirDensity * myVectorCW(0) * myVectorCrossSection(0) * vectorAirVelocityTurnedCart(0) * vectorAirVelocityTurnedCart(0) / 2.0f;
	vectorForceAirCart(1) = HSL.myLfAirDensity * myVectorCW(1) * myVectorCrossSection(1) * vectorAirVelocityTurnedCart(1) * vectorAirVelocityTurnedCart(1) / 2.0f;
	vectorForceAirCart(2) = HSL.myLfAirDensity * myVectorCW(2) * myVectorCrossSection(2) * vectorAirVelocityTurnedCart(2) * vectorAirVelocityTurnedCart(2) / 2.0f;

	vector<float> vectorForceAirSphere = XPlaneCartToSphere(vectorForceAirCart);

	vectorForceAirSphere(1) += ropeUnitSphere(1);
	vectorForceAirSphere(2) += ropeUnitSphere(2);

	myVectorForceAir = XPlaneSphereToCart(vectorForceAirSphere);*/


	
	myVectorForceAir(0) = (1.0f - myWaterLevel) * HSL.myLfAirDensity * myVectorCW(0) * myVectorCrossSection(0) * myVectorAirVelocity(0) * myVectorAirVelocity(0) / 2.0;
	myVectorForceAir(1) = (1.0f - myWaterLevel) * HSL.myLfAirDensity * myVectorCW(1) * myVectorCrossSection(1) * myVectorAirVelocity(1) * myVectorAirVelocity(1) / 2.0;
	myVectorForceAir(2) = (1.0f - myWaterLevel) * HSL.myLfAirDensity * myVectorCW(2) * myVectorCrossSection(2) * myVectorAirVelocity(2) * myVectorAirVelocity(2) / 2.0;

	if (myVectorAirVelocity(0) < 0) myVectorForceAir(0) *= -1;
	if (myVectorAirVelocity(1) < 0) myVectorForceAir(1) *= -1;
	if (myVectorAirVelocity(2) < 0) myVectorForceAir(2) *= -1;

	myAirResistance = norm_2(myVectorForceAir);

	myVectorWaterVelocity = -1 * myVectorVelocity;

	myVectorForceWater(0) = myWaterLevel * HSL_Data::density_water * myVectorCW(0) * myVectorCrossSection(0) * myVectorWaterVelocity(0) * myVectorWaterVelocity(0) / 2.0;
	myVectorForceWater(1) = myWaterLevel * HSL_Data::density_water * myVectorCW(1) * myVectorCrossSection(1) * myVectorWaterVelocity(1) * myVectorWaterVelocity(1) / 2.0;
	myVectorForceWater(2) = myWaterLevel * HSL_Data::density_water * myVectorCW(2) * myVectorCrossSection(2) * myVectorWaterVelocity(2) * myVectorWaterVelocity(2) / 2.0;

	if (myVectorWaterVelocity(0) < 0) myVectorForceWater(0) *= -1;
	if (myVectorWaterVelocity(1) < 0) myVectorForceWater(1) *= -1;
	if (myVectorWaterVelocity(2) < 0) myVectorForceWater(2) *= -1;

	float waterResistance = norm_2(myVectorForceWater);

	myVectorForceSwim(VERT_AXIS) = 0;
	if (myIsBambiBucket == false) myVectorForceSwim(VERT_AXIS) = -1 * HSL_Data::density_water * myVolume * myWaterLevel * HSL.myLfGravitation;


	// Get the force of the gravity
	myVectorForceGravity(VERT_AXIS) = HSL.myLfGravitation * (myMass + (myBambiBucketWaterWeight * (1.0f - myWaterLevel)));


	// Sum up the forces
	myVectorForceTotal = myVectorForceRope + myVectorForceAir + myVectorForceGravity + myVectorForceWater + myVectorForceSwim;

	// If we are on the ground and not pulled up, we need to compute the friction
	if (myTerrainHit == true)
	{
		if (myVectorForceTotal(VERT_AXIS) < 0)
		{
			// we did that before, right?
			myVectorHorizontalVelocity = myVectorVelocity;
			myVectorHorizontalVelocity(VERT_AXIS) = 0;

			// compute glide friction
			myVectorForceFriction = get_unit_vector(myVectorHorizontalVelocity) * myVectorForceTotal(VERT_AXIS) * myFrictionGlide;

			// Compute the speed, where the static friction would stop us within a frame. Multiply with 3 to be on the safe side for fps drop
			if ((myMass + myBambiBucketWaterWeight) == 0.0f) return;
			mySpeedStaticFriction = std::abs(3 * HSL.myFrameTime * myVectorForceTotal(VERT_AXIS) * myFrictionStatic / (myMass + myBambiBucketWaterWeight)); // adapt stop speed to frame rate

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
					myVectorVelocity = myVectorZeroVector;
					myVectorForceTotal = myVectorZeroVector;
					myVectorForceFriction = myVectorZeroVector;
				}
			}

			// Add friction to total foces
			myVectorForceTotal += myVectorForceFriction;
			myVectorForceTotal(VERT_AXIS) = 0; //Stop forces pulling below ground
		}
	}


	// Get the acceleration
	if ((myMass + myBambiBucketWaterWeight) == 0.0f) return;
	myVectorAccTotal = myVectorForceTotal / (myMass + myBambiBucketWaterWeight);

	// calcualte the new velocity
	myVectorVelocityDelta = myVectorAccTotal * HSL.myFrameTime;
	myVectorVelocity += myVectorVelocityDelta;

	check_nan(myVectorVelocity);
	limit_max(myVectorVelocity, MAX_OBJ_SPEED);

	if (HSL.myPhysicsEnabled == true)
	{

		myVectorPosition += myVectorVelocity * HSL.myFrameTime;

		check_nan(myVectorPosition);

		// Compute the force in aircraft coordinates. This just turns the vector in the right direction.
		myVectorForceChopper = HSL.TurnWorldToAircraft(-1 * myVectorForceRope);

		// Momentum in Center of Gravity = rxF
		myVectorMomentumChopper = cross_product(HSL.myVectorWinchPosition, myVectorForceChopper);
	}

	// Only if we are really intended to apply forces to the chopper
	if ((myRopeConnected == true) && (myFollowOnly == false) && (HSL.myRopeRuptured == false))
	{

		check_nan(myVectorForceChopper);
		check_nan(myVectorMomentumChopper);

		// Apply Forces
		XPLMSetDataf(HSL.myDrForceX, HSL.myLfForceX + myVectorForceChopper(0));
		XPLMSetDataf(HSL.myDrForceY, HSL.myLfForceY + myVectorForceChopper(1));
		XPLMSetDataf(HSL.myDrForceZ, HSL.myLfForceZ + myVectorForceChopper(2));

		// The momentum axis as not as in my mechanics book, checked by experiment
		XPLMSetDataf(HSL.myDrMomentumX, HSL.myLfMomentumX - myVectorMomentumChopper(2));
		XPLMSetDataf(HSL.myDrMomentumY, HSL.myLfMomentumY + myVectorMomentumChopper(0));
		XPLMSetDataf(HSL.myDrMomentumZ, HSL.myLfMomentumZ - myVectorMomentumChopper(1));
	}

	if ((HSL.myRopeRuptured == false) && (myRopeConnected == true) && (myFollowOnly == false))
	{
		HSL.myVectorHookPosition = myVectorPosition;
	}

	// Calculate Object Offset
	float scalarObjectOffset = norm_2(myVectorCargoOffset);
	vector<float>  myVectorFinalRope = HSL.myVectorHelicopterPosition - HSL.myVectorHookPosition;
	vector<float> vectorRopeUnit = get_unit_vector(myVectorFinalRope);


	// If ruptured, keep offset and angle

	if (((HSL.myRopeRuptured == false) && (myRopeConnected == true)) || (myOrientationFollowsDirection == true))
	{
		vector<float> normalPosition(3);

		normalPosition(0) = 0;
		normalPosition(1) = 1;
		normalPosition(2) = 0;

		vector<float> normalPositionSphere = XPlaneCartToSphere(normalPosition);
		vector<float> ropeUnitSphere = XPlaneCartToSphere(vectorRopeUnit);

		vector<float> negativeVelocity = -1 * myVectorVelocity;

		if (myOrientationFollowsDirection == true) ropeUnitSphere = XPlaneCartToSphere(get_unit_vector(negativeVelocity));

		normalPositionSphere(1) = ropeUnitSphere(1) - normalPositionSphere(1);
		normalPositionSphere(2) = ropeUnitSphere(2) - normalPositionSphere(2);


		if (myTerrainHit == false)
		{
			// Using Pitch/Roll
			myVectorDisplayAngle(0) = myVectorCargoRotation(0) + ropeUnitSphere(1) * 180.0f / M_PI; // Pitch
			myVectorDisplayAngle(1) = myVectorCargoRotation(1) + ropeUnitSphere(2) * 180.0f / M_PI; // Roll
			myVectorDisplayAngle(2) = myVectorCargoRotation(2); //heading
		}
		else
		{
			myVectorDisplayAngle = myVectorCargoRotation;
		}



		// Object Offset
		if (scalarObjectOffset > 0)
		{

			vector<float> myVectorCargoOffsetRotated = myVectorCargoOffset;

			float length = sqrt((myVectorCargoOffset(0) * myVectorCargoOffset(0)) + (myVectorCargoOffset(2) * myVectorCargoOffset(2)));
			float angle = atan2(myVectorCargoOffset(2), myVectorCargoOffset(0));

			angle += myVectorDisplayAngle(2) * M_PI / 180.0f;

			//myVectorCargoOffsetRotated(0) = length * cos(angle);
			//myVectorCargoOffsetRotated(2) = length * sin(angle);

			vector<float> vectorObjectOffsetSphere = XPlaneCartToSphere(myVectorCargoOffsetRotated);
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
}
