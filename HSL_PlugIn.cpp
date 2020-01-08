#include "HSL_PlugIn.h"
#include <thread>

std::recursive_mutex cargoDataSharedMutex;

bool HSL_PlugIn::HIGH_PERFORMANCE = false;

HSL_PlugIn::HSL_PlugIn() :
	myCargo(*this),
	myHook(*this),
	myPhysicsThreadObject(*this)
{
	CARGO_SHM_SECTION_START

	myRaindrops = new RainDropDrawData[HSL_RAINDROPS_DRAW_MAX];

	for (int rain_index = 0; rain_index < HSL_RAINDROPS_DRAW_MAX; rain_index++)
	{
		myRaindrops->dataValid = false;
		myRaindrops->myDrawInstance = NULL;
	}

	myCargo.myHeight = 0.9;
	myCargo.myMass = 75.0;
	myCargo.myFrictionGlide = 0.35;
	myCargo.myFrictionStatic = 0.65;

	myHook.myHeight = 0.1;
	myHook.myMass = 5.0;
	myHook.myFrictionGlide = 0.35;
	myHook.myFrictionStatic = 0.65;

	for (unsigned i = 0; i < myVectorZeroVector.size(); ++i) myVectorZeroVector(i) = 0;

	myCargoDataShared.myVectorHelicopterPosition = myVectorZeroVector;
	myCargoDataShared.myVectorHookPosition = myVectorZeroVector;
	myCargoDataShared.myVectorWinchPosition = myVectorZeroVector;


	//myCargoDataShared.myVectorHelicopterPositionFlightLoop = myVectorZeroVector;
	myCargoDataShared.myVectorHelicopterVelocity = myVectorZeroVector;
	myCargoDataShared.myVectorHelicopterAcceleration = myVectorZeroVector;
	myCargoDataShared.myVectorDebug = myVectorZeroVector;
	
	myVectorDefaultWinchPosition = myVectorZeroVector;
	myCargoDataShared.myVectorWindVelocity = myVectorZeroVector;

	myCargoDataShared.myVectorRope = myVectorZeroVector;
	myVectorBambiBucketReleasePosition = myVectorZeroVector;

	myVectorDefaultWinchPosition(0) = 0.0;
	myVectorDefaultWinchPosition(1) = -0.75;
	myVectorDefaultWinchPosition(2) = 0.0;

	memset(myRopeInstances, 0, sizeof(XPLMInstanceRef) * HSL_ROPE_POINTS_MAX);

	for (unsigned i = 0; i < HSL_ROPE_POINTS_MAX; ++i) myRopePoints[i] = myVectorZeroVector;
	for (unsigned i = 0; i < HSL_ROPE_POINTS_MAX; ++i) myDebugInstances[i] = NULL;
	
	
}

HSL_PlugIn::~HSL_PlugIn()
{
}

void HSL_PlugIn::PluginStart()
{
	CARGO_SHM_SECTION_START

	myDS = XPLMGetDirectorySeparator();

	char buffer[2048];
	XPLMGetSystemPath(buffer);
	mySystemPath = buffer;

	myConfigPath = mySystemPath + "Resources" + myDS + "plugins" + myDS + "HSL" + myDS;
	myFireAircraftPath = myConfigPath + myFireAircraftPath;
	myNoFireAircraftPath = myConfigPath + myNoFireAircraftPath;

	
	XPLMCreateFlightLoop_t flightLoopParams;
	flightLoopParams.structSize = sizeof(flightLoopParams);
	flightLoopParams.callbackFunc = WrapFlightLoopCallback;
	flightLoopParams.phase = xplm_FlightLoop_Phase_BeforeFlightModel;
	flightLoopParams.refcon = 0;

	//XPLMRegisterFlightLoopCallback(WrapFlightLoopCallback, 1, 0);

	myFlightLoopID = XPLMCreateFlightLoop(&flightLoopParams);
	XPLMScheduleFlightLoop(myFlightLoopID, -1, true);



	// Menu;
	myPluginMenu = XPLMAppendMenuItem(XPLMFindPluginsMenu(), "Sling Line", 0, 1);
	myPluginMenuID = XPLMCreateMenu("Sling Line", XPLMFindPluginsMenu(), myPluginMenu, WrapMenuHandler, 0);
	myEnableSlingMenu = XPLMAppendMenuItem(myPluginMenuID, "Enable Sling Line", "ItemEnable", 1);
	XPLMAppendMenuItem(myPluginMenuID, "Toggle Control Window", "ItemWindow", 2);


	// Commands


	myCmdRefWinchUp = XPLMCreateCommand("HSL/Winch_Up", "Move the winch up");
	myCmdRefWinchDown = XPLMCreateCommand("HSL/Winch_Down", "Move the winch down");
	myCmdRefWinchStop = XPLMCreateCommand("HSL/Winch_Stop", "Stop the winch");

	myCmdRefEnable = XPLMCreateCommand("HSL/Sling_Enable", "Enable slingline");
	myCmdRefDisable = XPLMCreateCommand("HSL/Sling_Disable", "Disable slingline");
	myCmdRefReset = XPLMCreateCommand("HSL/Sling_Reset", "Reset slingline");
	myCmdRefConnectLoad = XPLMCreateCommand("HSL/Load_Connect", "Connect the load");
	myCmdRefReleaseLoad = XPLMCreateCommand("HSL/Load_Release", "Release the load");
	myCmdRefPlaceCargoGround = XPLMCreateCommand("HSL/Load_On_Ground", "Place the load in front of the aircraft");
	myCmdRefPlaceCoordinates = XPLMCreateCommand("HSL/Load_On_Coordinates", "Place the load at given coordinates");
	myCmdRefToggleControlWindow = XPLMCreateCommand("HSL/ToogleControlWindow", "Toggle Control Window");
	myCmdRefUpdateObjects = XPLMCreateCommand("HSL/UpdateObjects", "Update Objects");

	myCmdRefFirePlaceGround = XPLMCreateCommand("HSL/Fire_On_Ground", "Place fire in front of the aircraft");
	myCmdRefFirePlaceCoordinates = XPLMCreateCommand("HSL/Fire_On_Coordinates", "Place fire at given coordinates");
	myCmdRefBambiBucketRelease = XPLMCreateCommand("HSL/Bucket_Release", "Release Water in Bambi Bucket");

	XPLMRegisterCommandHandler(myCmdRefWinchUp, WrapWinchUpCallback, 0, 0);
	XPLMRegisterCommandHandler(myCmdRefWinchDown, WrapWinchDownCallback, 0, 0);
	XPLMRegisterCommandHandler(myCmdRefWinchStop, WrapWinchStopCallback, 0, 0);
	XPLMRegisterCommandHandler(myCmdRefEnable, WrapEnableCallback, 0, 0);
	XPLMRegisterCommandHandler(myCmdRefDisable, WrapDisableCallback, 0, 0);
	XPLMRegisterCommandHandler(myCmdRefReset, WrapResetCallback, 0, 0);
	XPLMRegisterCommandHandler(myCmdRefConnectLoad, WrapConnectLoadCallback, 0, 0);
	XPLMRegisterCommandHandler(myCmdRefReleaseLoad, WrapReleaseLoadCallback, 0, 0);

	XPLMRegisterCommandHandler(myCmdRefPlaceCargoGround, WrapLoadGroundCallback, 0, 0);
	XPLMRegisterCommandHandler(myCmdRefPlaceCoordinates, WrapLoadCoordinatesCallback, 0, 0);

	XPLMRegisterCommandHandler(myCmdRefToggleControlWindow, WrapToggleControlWindowCallback, 0, 0);
	XPLMRegisterCommandHandler(myCmdRefUpdateObjects, WrapUpdateObjectCallback, 0, 0);


	XPLMRegisterCommandHandler(myCmdRefFirePlaceGround, WrapFireGroundCallback, 0, 0);
	XPLMRegisterCommandHandler(myCmdRefFirePlaceCoordinates, WrapFireCoordinatesCallback, 0, 0);
	XPLMRegisterCommandHandler(myCmdRefBambiBucketRelease, WrapBambiBucketRelease, 0, 0);




	//myDrShRopeLengthStart = XPLMRegisterDataAccessor("HSL/RopeLengthStart", xplmType_double, 1, NULL, NULL, WrapReaddoubleCallback, WrapWritedoubleCallback, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, &myRopeLengthStart, &myRopeLengthStart);

	
	
	
	
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//                      DATAREF REGISTER
	
	
	
	RegisterDoubleDataref(myCargoDataShared.myCargoSetLatitutde,				"HSL/Cargo/SetLatitude");
	RegisterDoubleDataref(myCargoDataShared.myCargoSetLongitude,				"HSL/Cargo/SetLongitude");
	
	RegisterDoubleDataref(myCargoDataShared.myWinchSpeed,						"HSL/WinchSpeed");
	RegisterVectorDataref(myCargoDataShared.myVectorWinchPosition,			"HSL/VectorWinchPosition");
	RegisterDoubleDataref(myCargoDataShared.myRopeLengthStart,					"HSL/RopeLengthStart");
	RegisterDoubleDataref(myCargoDataShared.myRopeLengthNormal,				"HSL/RopeLengthNormal");
	RegisterDoubleDataref(myCargoDataShared.myRopeDamping,						"HSL/RopeDamping");
	RegisterDoubleDataref(myCargoDataShared.myRopeK,							"HSL/RopeK");
	RegisterDoubleDataref(myCargoDataShared.myRopeRuptureForce,				"HSL/Rope/RuptureForce");
	RegisterDoubleDataref(myCargoDataShared.myMaxAccRopeFactor,				"HSL/Rope/MaxAccRopeFactor");

	// Hook 
	RegisterIntDataref(myHook.myRopeConnected,				"HSL/Hook/Connected");
	RegisterIntDataref(myHook.myFollowOnly,					"HSL/Hook/FollowOnly");
	RegisterIntDataref(myHook.myDrawingEnabled,				"HSL/Hook/DrawingEnabled");
	RegisterIntDataref(myHook.myInstancedDrawing,			"HSL/Hook/InstancedDrawing");

	RegisterDoubleDataref(myHook.myHeight,					"HSL/Hook/Height");
	RegisterDoubleDataref(myHook.myMass,						"HSL/Hook/Mass");
	RegisterVectorDataref(myHook.myVectorSize,				"HSL/Hook/Size");
	RegisterVectorDataref(myHook.myVectorCW,				"HSL/Hook/CW");
	RegisterDoubleDataref(myHook.myFrictionGlide,			"HSL/Hook/FrictionGlide");
	RegisterDoubleDataref(myHook.myFrictionStatic,			"HSL/Hook/FrictionStatic");

	RegisterVectorDataref(myHook.myVectorExternalForce,		"HSL/Hook/ExternalForce");
	RegisterVectorDataref(myHook.myVectorCargoRotation,		"HSL/Hook/Rotation");

	// Cargo
	RegisterIntDataref(myCargo.myRopeConnected,				"HSL/Cargo/Connected");
	RegisterIntDataref(myCargo.myFollowOnly,				"HSL/Cargo/FollowOnly");
	RegisterIntDataref(myCargo.myDrawingEnabled,			"HSL/Cargo/DrawingEnabled");
	RegisterIntDataref(myCargo.myInstancedDrawing,			"HSL/Cargo/InstancedDrawing");

	RegisterVectorDataref(myCargo.myVectorPosition,			"HSL/Cargo/Position");
	RegisterDoubleDataref(myCargo.myHeight,					"HSL/Cargo/Height");
	RegisterDoubleDataref(myCargo.myMass,					"HSL/Cargo/Mass");
	RegisterVectorDataref(myCargo.myVectorSize,				"HSL/Cargo/Size");
	RegisterVectorDataref(myCargo.myVectorCW,				"HSL/Cargo/CWFront");
	RegisterDoubleDataref(myCargo.myFrictionGlide,			"HSL/Cargo/FrictionGlide");
	RegisterDoubleDataref(myCargo.myFrictionStatic,			"HSL/Cargo/FrictionStatic");
	RegisterIntDataref(myCargo.myIsBambiBucket,				"HSL/Cargo/IsBambiBucket");
	RegisterIntDataref(myCargo.myBambiBucketRelease,		"HSL/Cargo/BambiBucketReleaseWater");
	RegisterDoubleDataref(myCargo.myBambiBucketWaterWeight,	"HSL/Cargo/BambiBucketWaterWeigth");
	RegisterDoubleDataref(myCargo.myBambiBucketWaterLevel,	"HSL/Cargo/BambiBucketWaterLevel");
	RegisterVectorDataref(myCargo.myVectorCargoOffset,		"HSL/Cargo/RopeOffset");
	RegisterVectorDataref(myCargo.myVectorExternalForce,	"HSL/Cargo/ExternalForce");
	RegisterVectorDataref(myCargo.myVectorCargoRotation,	"HSL/Cargo/Rotation");



	RegisterVectorDataref(myCargoDataShared.myVectorHelicopterPosition,			"HSL/Calculated/VectorHelicopterPosition");
	RegisterVectorDataref(myCargoDataShared.myVectorHookPosition,				"HSL/Calculated/VectorHookPosition");


	RegisterDoubleDataref(myCargoDataShared.myFrameTime,						"HSL/FrameTime");
	RegisterDoubleDataref(myCargoDataShared.myNewRopeLength,					"HSL/NewRopeLength");
	RegisterDoubleDataref(myCargoDataShared.myRopeStretchRelative,				"HSL/RopeStretchRelative");
	RegisterDoubleDataref(myCargoDataShared.myRopeForceScalar,					"HSL/RopeForceScalar");
	RegisterDoubleDataref(myCargoDataShared.myRopeLengthDelta,					"HSL/RopeLengthDelta");
	RegisterDoubleDataref(myCargoDataShared.myRopeStretchSpeed,					"HSL/RopeStretchSpeed");
	RegisterDoubleDataref(myCargoDataShared.myRopeCorrectedD,					"HSL/RopeCorrectedD");
	RegisterIntDataref(myCargoDataShared.myRopeRuptured,						"HSL/RopeRuptured");
	



	RegisterStringDataref(myWinchPath,						"HSL/WinchObjectPath");
	RegisterStringDataref(myRopePath,						"HSL/RopeObjectPath");
	RegisterStringDataref(myHookPath,						"HSL/HookObjectPath");
	RegisterStringDataref(myCargoPath,						"HSL/CargoObjectPath");

	// Fire:

	RegisterStringDataref(myFireAircraftPath,				"HSL/Fire/FireAircraftPath");
	RegisterDoubleDataref(myFireWaterRadius,					"HSL/Fire/WaterRadius");
	RegisterDoubleDataref(myFireLiftNom,						"HSL/Fire/LiftNom");
	RegisterDoubleDataref(myFireLiftDenom,					"HSL/Fire/LiftDenom");
	RegisterDoubleDataref(myFireDistDemon,					"HSL/Fire/DistDenom");

	RegisterDoubleDataref(myFireStrengthStart,				"HSL/Fire/StrengthStart");
	RegisterDoubleDataref(myFireStrengthMax,					"HSL/Fire/StrengthMax");
	RegisterDoubleDataref(myFireStrengthIncrease,			"HSL/Fire/StrengthIncrease");


	RegisterDoubleDataref(myFireSetLatitutde,				"HSL/Fire/SetLatitude");
	RegisterDoubleDataref(myFireSetLongitude,				"HSL/Fire/SetLongitude");
	RegisterDoubleDataref(myFireSetElevation,				"HSL/Fire/SetElevation");

	RegisterDoubleDataref(myFireCount,						"HSL/Fire/Count");
	RegisterDoubleArrayDataref(myFireStrength,				"HSL/Fire/FireStrength", MAX_FIRES);

	RegisterIntDataref(myFireCreateFailed,					"HSL/Fire/CreateFailed");
	RegisterIntDataref(myUpdateFirePositions,				"HSL/Fire/UpdatePositions");
	RegisterIntDataref(myRemoveFires,						"HSL/Fire/RemoveFires");





	// Drawing
	XPLMRegisterDrawCallback(WrapDrawCallback, xplm_Phase_Objects, 0, NULL); //xplm_Phase_Airplanes
	//XPLMRegisterDrawCallback(WrapDrawCallback, xplm_Phase_Airplanes, 0, NULL); //

	int left, top, right, bot;
	XPLMGetScreenBoundsGlobal(&left, &top, &right, &bot);


	int width = 1000;
	int height = 700;
	int left_pad = 175;
	int top_pad = 75;
	int x = left + left_pad;
	int y = top - top_pad;

	int decorate = 1;
	imguiPtr = std::make_shared<HSLImguiWidget>(this,
		300,
		550,
		300 + 1200,
		550 - 400,
		decorate);

	HSLDebugString("Lookup Datarefs");


	myDrLocalX = XPLMFindDataRef("sim/flightmodel/position/local_x");
	myDrLocalY = XPLMFindDataRef("sim/flightmodel/position/local_y");
	myDrLocalZ = XPLMFindDataRef("sim/flightmodel/position/local_z");

	myDrLocalXv = XPLMFindDataRef("sim/flightmodel/position/local_vx");
	myDrLocalYv = XPLMFindDataRef("sim/flightmodel/position/local_vy");
	myDrLocalZv = XPLMFindDataRef("sim/flightmodel/position/local_vz");

	myDrLocalXa = XPLMFindDataRef("sim/flightmodel/position/local_ax");
	myDrLocalYa = XPLMFindDataRef("sim/flightmodel/position/local_ay");
	myDrLocalZa = XPLMFindDataRef("sim/flightmodel/position/local_az");

	myDrLocalPhi = XPLMFindDataRef("sim/flightmodel/position/phi");
	myDrLocalPsi = XPLMFindDataRef("sim/flightmodel/position/psi");
	myDrLocalTheta = XPLMFindDataRef("sim/flightmodel/position/theta");

	myDrPause = XPLMFindDataRef("sim/time/paused");
	myDrTimeActual = XPLMFindDataRef("sim/time/sim_speed");
	myDrWindDirection = XPLMFindDataRef("sim/weather/wind_direction_degt");
	myDrWindSpeed = XPLMFindDataRef("sim/weather/wind_speed_kt");

	myDrWindX = XPLMFindDataRef("sim/weather/wind_now_x_msc");
	myDrWindY = XPLMFindDataRef("sim/weather/wind_now_y_msc");
	myDrWindZ = XPLMFindDataRef("sim/weather/wind_now_z_msc");

	myDrForceX = XPLMFindDataRef("sim/flightmodel/forces/fside_plug_acf");
	myDrForceY = XPLMFindDataRef("sim/flightmodel/forces/fnrml_plug_acf");
	myDrForceZ = XPLMFindDataRef("sim/flightmodel/forces/faxil_plug_acf");

	myDrMomentumX = XPLMFindDataRef("sim/flightmodel/forces/L_plug_acf");
	myDrMomentumY = XPLMFindDataRef("sim/flightmodel/forces/M_plug_acf");
	myDrMomentumZ = XPLMFindDataRef("sim/flightmodel/forces/N_plug_acf");

	myDrAirDensity = XPLMFindDataRef("sim/weather/rho");
	myDrGravitation = XPLMFindDataRef("sim/weather/gravity_mss");

	HSLDebugString("Config Read");
	ConfigRead();

	myInitialized = true;

	HSLDebugString("Starting Drop Thread");
	myDropThreadPtr = new std::thread(&DropThread::RunDropThread, &myDropThreadObject, 1);
	HSLDebugString("Drop Thread running");

	HSLDebugString("Starting Physics Thread");
	myPhysicsThreadPtr = new std::thread(&PhysicsThread::RunPhysicsThread, &myPhysicsThreadObject, 1);
	HSLDebugString("Physics Thread running");


}

void HSL_PlugIn::PluginStop()
{
	if (myInitialized == false) return;
	ConfigSave();


	XPLMDestroyMenu(myPluginMenuID);
	for (auto dataRef: myRegisteredDatarefs) XPLMUnregisterDataAccessor(dataRef);

	if (myDropThreadPtr != NULL)
	{
		myDropThreadObject.myHSLCommandsIn.push(DropThreadData::ThreadStop);
		myDropThreadPtr->join();
		delete myDropThreadPtr;
		myDropThreadPtr = NULL;
	}

	if (myPhysicsThreadPtr != NULL)
	{
		CARGO_SHM_SECTION_START
		myCargoDataShared.myThreadRunFlag = false;
		CARGO_SHM_SECTION_END

		myPhysicsThreadPtr->join();
		delete myPhysicsThreadPtr;
		myPhysicsThreadPtr = NULL;
	}

	XPLMReleasePlanes();

}

void HSL_PlugIn::PluginEnable()
{
	myPluginEnabled = 1;
}

void HSL_PlugIn::PluginDisable()
{
	myPluginEnabled = 0;
}

void HSL_PlugIn::PluginReceiveMessage(XPLMPluginID inFromWho, int inMessage, void* inParam)
{
	HSLDebugString("Message Received");

	if (myInitialized == false) return;

	if (inFromWho == XPLM_PLUGIN_XPLANE)
	{
		//if ((inMessage == XPLM_MSG_PLANE_LOADED) && (*((int *) (inParam)) == 0)) // 0 = User Aircraft
		if (inMessage == XPLM_MSG_PLANE_LOADED)
		{
			HSLDebugString("Plane loaded");

			myAircraftLoaded = 1;
			//SlingDisable();

		}
		if (inMessage == XPLM_MSG_AIRPORT_LOADED)
		{
			HSLDebugString("Airport loaded");
			SlingDisable();
		}


		if (inMessage == XPLM_MSG_SCENERY_LOADED)
		{
			HSLDebugString("Scenery loaded");
			SlingDisable();

		}
	}
}

void HSL_PlugIn::PluginMenuHandler(void* in_menu_ref, void* inItemRef)
{
	HSLDebugString("Menu Handler");
	if (myInitialized == false) return;

	if (strcmp((char*)inItemRef, "ItemWindow") == NULL)
	{
		if (!imguiPtr->GetVisible())
		{
			imguiPtr->Visible(true);
		}
		else
		{
			imguiPtr->Visible(false);
		}
	}
	else if (strcmp((char*)inItemRef, "ItemEnable") == NULL)
	{
		if (mySlingLineEnabled == true)
			SlingDisable();
		else
			SlingEnable();
	}
}

void HSL_PlugIn::PluginKeyCallback(XPLMWindowID inWindowID, char inKey, XPLMKeyFlags inFlags, char inVirtualKey, void* inRefcon, int losingFocus)
{
}

int HSL_PlugIn::PluginMouseClickCallback(XPLMWindowID inWindowID, int x, int y, XPLMMouseStatus inMouse, void* inRefcon)
{
	return 0;
}

int HSL_PlugIn::DrawCallback(XPLMDrawingPhase inPhase, int inIsBefore, void* inRefcon)
{
	

	static XPLMDrawInfo_t ropePositions[HSL_ROPE_POINTS_MAX];

	auto time_start = std::chrono::steady_clock::now();

	if (mySlingLineEnabled == true)
	{
		CARGO_SHM_SECTION_START

		ReadDataRefs();

		for (auto fire : myFires)
		{
			//XPLMDrawAircraft(fire.planeIndex, fire.posX, fire.posY, fire.posZ, 0, 0, 0, 1, &(fire.drawState));
		}

		vector<double> vectorWinchWorld = AdjustFrameMovement(myCargoDataShared.myVectorHelicopterPosition);
		//vector<double> vectorWinchWorld = myVectorHelicopterPosition;
		DrawInstanceCreate(myWinchInstanceRef, myWinchObjectRef);
		DrawInstanceSetPosition(myWinchInstanceRef, myWinchObjectRef, vectorWinchWorld, false);

		/*int index;

		for (index = 0; index < HSL_ROPE_POINTS_MAX; index++)
		{
			if (index < myRopePoints.size())
			{
				vector<double> vectorRopePointOpenGL = AdjustFrameMovement(myRopePoints[index]);
				DrawInstanceCreate(myRopeInstances[index], myRopeObjectRef);
				DrawInstanceSetPosition(myRopeInstances[index], myRopeObjectRef, vectorRopePointOpenGL, true);
			}
			else
			{
				DrawInstanceDestroy(myRopeInstances[index]);
			}
		}*/

		/*else
		{
			static XPLMDrawInfo_t		drawInfo[HSL_ROPE_POINTS_MAX];
			static bool init = false;

			if (init == false)
			{
				for (index = 0; index < HSL_ROPE_POINTS_MAX; index++)
				{
					drawInfo[index].structSize = sizeof(drawInfo);
					drawInfo[index].x = 0;
					drawInfo[index].y = 0;
					drawInfo[index].z = 0;
					drawInfo[index].pitch = 0;
					drawInfo[index].heading = 0;
					drawInfo[index].roll = 0;
				}
				init = true;
			}


			int elements = myRopePoints.size();
			if (elements > HSL_ROPE_POINTS_MAX) elements = HSL_ROPE_POINTS_MAX;

			for (index = 0; index < myRopePoints.size(); index++)
			{
				vector<double> vectorRopePointOpenGL = AdjustFrameMovement(myRopePoints[index]);

				drawInfo[index].x = vectorRopePointOpenGL(0);
				drawInfo[index].y = vectorRopePointOpenGL(1);
				drawInfo[index].z = vectorRopePointOpenGL(2);
			}

			XPLMDrawObjects(myRopeObjectRef, myRopePoints.size(), drawInfo, 0, 0);
		}*/

		vector<double>  vectorFinalRope = myVectorZeroVector;
		vector<double>  vectorFinalRopeStart = myVectorZeroVector;
		

		
		if (myCargo.myDrawingEnabled == true)
		{
			

			 
			vector<double> vectorCargoPointOpenGL = myCargo.myVectorPosition; //AdjustFrameMovement(myCargo.myVectorPosition);
			vectorCargoPointOpenGL += myCargo.myVectorHelicopterPositionDeviation;

			vectorFinalRope = vectorWinchWorld - vectorCargoPointOpenGL;
			vectorFinalRopeStart = vectorCargoPointOpenGL;

			vectorCargoPointOpenGL -= myCargo.myVectorDisplayOffset;

			//vectorFinalRopeStart = vectorCargoPointOpenGL;

			DrawInstanceCreate(myCargoInstanceRef, myCargoObjectRef);
			DrawInstanceSetPosition(myCargoInstanceRef, myCargoObjectRef, vectorCargoPointOpenGL, myCargo.myVectorDisplayAngle, false);
			
			myVectorBambiBucketReleasePosition = vectorCargoPointOpenGL;
			vector<double> vectorReleaseHeight = get_unit_vector(vectorFinalRope) * myCargo.myHeight;
			myVectorBambiBucketReleasePosition -= vectorReleaseHeight;
			myCargo.myVectorDrawPosition = vectorCargoPointOpenGL;
		}
		else
		{
			DrawInstanceDestroy(myCargoInstanceRef);
		}

		
		if (myHook.myDrawingEnabled == true)
		{
			

			vector<double> vectorHookPointOpenGL = myHook.myVectorPosition; // AdjustFrameMovement(myHook.myVectorPosition);
			vectorHookPointOpenGL += myHook.myVectorHelicopterPositionDeviation;

			vectorFinalRope = vectorWinchWorld - vectorHookPointOpenGL;
			vectorFinalRopeStart = vectorHookPointOpenGL;
			DrawInstanceCreate(myHookInstanceRef, myHookObjectRef);
			DrawInstanceSetPosition(myHookInstanceRef, myHookObjectRef, vectorHookPointOpenGL, myHook.myVectorDisplayAngle, false);
			myHook.myVectorDrawPosition = vectorHookPointOpenGL;
		}
		else
		{
			DrawInstanceDestroy(myHookInstanceRef);
		}

		bool ropeRuptured = myCargoDataShared.myRopeRuptured;
		int ropeObjectCount = (int) (myCargoDataShared.myRopeLengthNormal / 0.01f);
		
		if (ropeObjectCount > HSL_ROPE_POINTS_MAX) ropeObjectCount = HSL_ROPE_POINTS_MAX;

		auto time_end = std::chrono::steady_clock::now();
		myProcessingTimeDrawRoutine = (int) std::chrono::duration_cast<std::chrono::microseconds>(time_end - time_start).count();
		vector<double> vectorDebug = myCargoDataShared.myVectorDebug;

		CARGO_SHM_SECTION_END // Now comes the time consuming part

		if ((ropeRuptured == false) && (norm_2(vectorFinalRope) > 0))
		{
			vector<double> vectorRopeUnit = get_unit_vector(vectorFinalRope);

			

			double ropeDistance = norm_2(vectorFinalRope) / ((double) ropeObjectCount);
			int index = 0;
			for (index = 0; index < ropeObjectCount; index++)
			{
				double distance = ropeDistance * index;
				vector<double> position = vectorFinalRopeStart + (distance * vectorRopeUnit);

				DrawInstanceCreate(myRopeInstances[index], myRopeObjectRef);
				DrawInstanceSetPosition(myRopeInstances[index], myRopeObjectRef, position, true);

			}
			for (; index < HSL_ROPE_POINTS_MAX; index++)
			{
				DrawInstanceDestroy(myRopeInstances[index]);
			}
		}
		else
		{
			for (int index = 0; index < HSL_ROPE_POINTS_MAX; index++)
			{
				DrawInstanceDestroy(myRopeInstances[index]);
			}
		}

		if (norm_2(vectorDebug) != 0)
		{
			vector<double> vectorDebugUnit = get_unit_vector(vectorDebug);
			for (int index = 0; index < 100; index++)
			{
				double distance = ((double) index)/100.0;
				vector<double> position = vectorFinalRopeStart + (distance * vectorDebugUnit);
				DrawInstanceCreate(myDebugInstances[index], myRopeObjectRef);
				DrawInstanceSetPosition(myDebugInstances[index], myRopeObjectRef, position, true);
			}

		}

		/*int index;

		for (index = 0; index < HSL_ROPE_POINTS_MAX; index++)
		{
			if (index < myRopePoints.size())
			{
				vector<double> vectorRopePointOpenGL = AdjustFrameMovement(myRopePoints[index]);
				DrawInstanceCreate(myRopeInstances[index], myRopeObjectRef);
				DrawInstanceSetPosition(myRopeInstances[index], myRopeObjectRef, vectorRopePointOpenGL, true);
			}
			else
			{
				DrawInstanceDestroy(myRopeInstances[index]);
			}
		}*/

		/*
				myRopePoints.clear();
		vector<double>  myVectorFinalRope = myCargoDataShared.myVectorHelicopterPosition - myCargoDataShared.myVectorHookPosition;
		vector<double> vectorRopeUnit = get_unit_vector(myVectorFinalRope);

		// If the rope ruptured, the empty vector will disable 
		if (myCargoDataShared.myRopeRuptured == false)
		{
			

			double ropeStepSize = norm_2(myVectorFinalRope) / ((double)HSL_ROPE_POINTS_MAX);

			if (ropeStepSize < 0.01f) ropeStepSize = 0.01;

			for (double distance = 0; distance < (myCargoDataShared.myNewRopeLength); distance += ropeStepSize)
			{
				vector<double> position = myCargoDataShared.myVectorHookPosition + (distance * vectorRopeUnit);
				myRopePoints.push_back(position);
			}
		}
		*/

		for (int rain_index = 0; rain_index < HSL_RAINDROPS_DRAW_MAX; rain_index++)
		{
			
			if (myRaindrops[rain_index].dataValid == true)
			{
				vector<double> vectorCargoPointOpenGL = AdjustFrameMovement(myRaindrops[rain_index].myVectorPosition);
				DrawInstanceCreate(myRaindrops[rain_index].myDrawInstance, myRaindropObjectRef);
				DrawInstanceSetPosition(myRaindrops[rain_index].myDrawInstance, myRaindropObjectRef, vectorCargoPointOpenGL, myRaindrops[rain_index].myVectorDisplayAngle, true);
			}
			else
			{
				DrawInstanceCreate(myRaindrops[rain_index].myDrawInstance, myRaindropObjectRef);
				DrawInstanceSetPosition(myRaindrops[rain_index].myDrawInstance, myRaindropObjectRef, myVectorZeroVector, myVectorZeroVector, true);
			}
		}
	}

	

	return 1;
}

void HSL_PlugIn::ConfigSave()
{
	CARGO_SHM_SECTION_START
	try
	{



			boost::property_tree::ptree pt;
			pt.put("System.simple_mode", mySimpleMode);
			pt.put("System.high_performance", myCargoDataShared.myHighPerformace);
			pt.put("Rope.library_object", myRopePath);
			pt.put("Rope.length", myCargoDataShared.myRopeLengthStart);
			pt.put("Rope.damping", myCargoDataShared.myRopeDamping);
			pt.put("Rope.k_factor", myCargoDataShared.myRopeK);
			pt.put("Rope.rupture_force", myCargoDataShared.myRopeRuptureForce);
			pt.put("Rope.max_acc_factor", myCargoDataShared.myMaxAccRopeFactor);
			

			pt.put("Winch.library_object", myWinchPath);
			pt.put("Winch.speed", myCargoDataShared.myWinchSpeed);
			pt.put("Winch.operator_force", myCargoDataShared.myRopeOperatorDampingForce);
			pt.put("Winch.operator_length", myCargoDataShared.myRopeOperatorDampingLength);
			
			pt.put("Bambi.water_per_second", myCargoDataShared.myBambiBucketWaterFlow);
			pt.put("Bambi.water_speed", myRainSpeed);
			pt.put("Bambi.water_directions", myRainDirections);
			pt.put("Bambi.water_variance", myRainVariance);
			pt.put("Bambi.water_period", myRainReleasePeriod);

			pt.put("Hook.library_object", myHookPath);
			pt.put("Hook.height", myHook.myHeight);
			pt.put("Hook.mass", myHook.myMass);

			ConfigWriteVector(pt, myHook.myVectorSize, "Hook.size");
			ConfigWriteVector(pt, myHook.myVectorCW, "Hook.CW");

			pt.put("Hook.friction_glide", myHook.myFrictionGlide);
			pt.put("Hook.friction_static", myHook.myFrictionStatic);

			pt.put("Cargo.library_object", myCargoPath);
			pt.put("Cargo.height", myCargo.myHeight);
			pt.put("Cargo.mass", myCargo.myMass);
			pt.put("Cargo.is_bambi_bucket", myCargo.myIsBambiBucket);

			ConfigWriteVector(pt, myCargo.myVectorSize, "Cargo.size");
			ConfigWriteVector(pt, myCargo.myVectorCW, "Cargo.CW");

			pt.put("Cargo.friction_glide", myCargo.myFrictionGlide);
			pt.put("Cargo.friction_static", myCargo.myFrictionStatic);
			ConfigWriteVector(pt, myCargo.myVectorCargoOffset, "Cargo.offset");
			
			ConfigWriteVector(pt, myCargoDataShared.myVectorWinchPosition, "HSL_Aircraft.winch_position");
			
			boost::property_tree::ini_parser::write_ini(myConfigPath + myConfigIniFile, pt);

	}
	catch (...)
	{
		HSLDebugString("Writing aircraft ini file crashed");
	}
}

void HSL_PlugIn::ConfigRead()
{
	CARGO_SHM_SECTION_START
	boost::property_tree::ptree pt;
	try
	{
		boost::property_tree::ini_parser::read_ini(myConfigPath + myConfigIniFile, pt);

		ConfigReadBool(pt, "System.simple_mode", mySimpleMode);
		ConfigReadBool(pt, "System.high_performance", myCargoDataShared.myHighPerformace);


		ConfigReadString(pt, "Rope.library_object", myRopePath);
		ConfigReaddouble(pt, "Rope.length", myCargoDataShared.myRopeLengthStart);
		ConfigReaddouble(pt, "Rope.damping", myCargoDataShared.myRopeDamping);
		ConfigReaddouble(pt, "Rope.k_factor", myCargoDataShared.myRopeK);
		ConfigReaddouble(pt, "Rope.rupture_force", myCargoDataShared.myRopeRuptureForce);
		ConfigReaddouble(pt, "Rope.max_acc_factor", myCargoDataShared.myMaxAccRopeFactor);

		ConfigReadString(pt, "Winch.library_object", myWinchPath);
		ConfigReaddouble(pt, "Winch.speed", myCargoDataShared.myWinchSpeed);

		ConfigReaddouble(pt, "Winch.operator_force", myCargoDataShared.myRopeOperatorDampingForce);
		ConfigReaddouble(pt, "Winch.operator_length", myCargoDataShared.myRopeOperatorDampingLength);

		ConfigReaddouble(pt, "Bambi.water_per_second", myCargoDataShared.myBambiBucketWaterFlow);


		ConfigReaddouble(pt, "Bambi.water_speed", myRainSpeed);
		ConfigReadInt(pt, "Bambi.water_directions", myRainDirections);
		ConfigReadInt(pt, "Bambi.water_variance", myRainVariance);
		ConfigReaddouble(pt, "Bambi.water_period", myRainReleasePeriod);
		

		ConfigReadString(pt, "Hook.library_object", myHookPath);
		ConfigReaddouble(pt, "Hook.height", myHook.myHeight);
		ConfigReaddouble(pt, "Hook.mass", myHook.myMass);

		ConfigReadVector(pt, myHook.myVectorSize, "Hook.size");
		ConfigReadVector(pt, myHook.myVectorCW, "Hook.CW");

		ConfigReaddouble(pt, "Hook.friction_glide", myHook.myFrictionGlide);
		ConfigReaddouble(pt, "Hook.friction_static", myHook.myFrictionStatic);

		ConfigReadVector(pt, myCargo.myVectorSize, "Cargo.size");
		ConfigReadVector(pt, myCargo.myVectorCW, "Cargo.CW");
		ConfigReadBool(pt, "Cargo.is_bambi_bucket", myCargo.myIsBambiBucket);

		ConfigReadString(pt, "Cargo.library_object", myCargoPath);
		ConfigReaddouble(pt, "Cargo.height", myCargo.myHeight);
		ConfigReaddouble(pt, "Cargo.mass", myCargo.myMass);
		ConfigReaddouble(pt, "Cargo.friction_glide", myCargo.myFrictionGlide);
		ConfigReaddouble(pt, "Cargo.friction_static", myCargo.myFrictionStatic);
		ConfigReadVector(pt, myCargo.myVectorCargoOffset, "Cargo.offset");
	}
	catch (...)
	{
		HSLDebugString("Could not read config file");
		return;
	}

	//try { m_global_path_index = pt.get<int>("HRM.global_path_index"); }
	//catch (...) { HRMDebugString("Ini File: Entry not found."); }
}

void HSL_PlugIn::AircraftConfigSave()
{
	CARGO_SHM_SECTION_START
	try
	{
		char buffer[2048];
		char buffer2[2048];
		XPLMGetNthAircraftModel(0, buffer, buffer2);

		std::string file_name = buffer2;

		std::size_t found = file_name.find_last_of(myDS);

		if (found != std::string::npos)
		{
			file_name = file_name.substr(0, found + 1);
			file_name += myAircraftIniFile;

			boost::property_tree::ptree pt;
			ConfigWriteVector(pt, myCargoDataShared.myVectorWinchPosition, "HSL_Aircraft.winch_position");
			boost::property_tree::ini_parser::write_ini(file_name, pt);
		}
	}
	catch (...)
	{
		HSLDebugString("Writing aircraft ini file crashed");
	}
}

void HSL_PlugIn::AircraftConfigRead()
{
	CARGO_SHM_SECTION_START
	try
	{
		char buffer[2048];
		char buffer2[2048];
		XPLMGetNthAircraftModel(0, buffer, buffer2);

		std::string file_name = buffer2;

		std::size_t found = file_name.find_last_of(myDS);

		if (found != std::string::npos)
		{
			file_name = file_name.substr(0, found + 1);
			file_name += myAircraftIniFile;
			boost::property_tree::ptree pt;
			boost::property_tree::ini_parser::read_ini(file_name, pt);

			ConfigReadVector(pt, myCargoDataShared.myVectorWinchPosition, "HSL_Aircraft.winch_position");
		}
	}
	catch (...)
	{
		HSLDebugString("Writing aircraft ini file crashed");
	}
}

void HSL_PlugIn::ConfigWriteVector(boost::property_tree::ptree& pt, vector<double>& vectorIn, std::string nameIn)
{
	pt.put(nameIn + "_1", vectorIn(0));
	pt.put(nameIn + "_2", vectorIn(1));
	pt.put(nameIn + "_3", vectorIn(2));
}

void HSL_PlugIn::ConfigReadVector(boost::property_tree::ptree& pt, vector<double>& vectorOut, std::string nameIn)
{
	ConfigReaddouble(pt, nameIn + "_1", vectorOut(0));
	ConfigReaddouble(pt, nameIn + "_2", vectorOut(1));
	ConfigReaddouble(pt, nameIn + "_3", vectorOut(2));
}

void HSL_PlugIn::ConfigReaddouble(boost::property_tree::ptree& pt, std::string nameIn, double& doubleOut)
{
	try { doubleOut = pt.get<double>(nameIn); }
	catch (...) { HSLDebugString("Ini File: Entry not found:" + nameIn); }
}

void HSL_PlugIn::ConfigReadInt(boost::property_tree::ptree& pt, std::string nameIn, int& intOut)
{
	try { intOut = pt.get<int>(nameIn); }
	catch (...) { HSLDebugString("Ini File: Entry not found:" + nameIn); }
}

void HSL_PlugIn::ConfigReadBool(boost::property_tree::ptree& pt, std::string nameIn, bool& boolOut)
{
	try { boolOut = pt.get<bool>(nameIn); }
	catch (...) { HSLDebugString("Ini File: Entry not found:" + nameIn); }
}

void HSL_PlugIn::ConfigReadString(boost::property_tree::ptree& pt, std::string nameIn, std::string& stringOut)
{
	try { stringOut = pt.get<std::string>(nameIn); }
	catch (...) { HSLDebugString("Ini File: Entry not found:" + nameIn); }
}



void HSL_PlugIn::SlingEnable()
{
	XPLMCheckMenuItem(myPluginMenuID, myEnableSlingMenu, xplm_Menu_Checked);
	mySlingLineEnabled = myCargoDataShared.mySlingLineEnabled = true;
	SlingReset();
	XPLMSetActiveAircraftCount(MAX_FIRES);
}

void HSL_PlugIn::SlingDisable()
{
	CARGO_SHM_SECTION_START
	myCargoDataShared.myComputationRunFlag = false;
	XPLMCheckMenuItem(myPluginMenuID, myEnableSlingMenu, xplm_Menu_Unchecked);
	mySlingLineEnabled = myCargoDataShared.mySlingLineEnabled = false;
}

void HSL_PlugIn::SlingReset()
{
	CARGO_SHM_SECTION_START
	myCargoDataShared.myComputationRunFlag = false;

	for (unsigned i = 0; i < myCargoDataShared.myVectorHelicopterPosition.size(); ++i) myCargoDataShared.myVectorHelicopterPosition(i) = 0;
	for (unsigned i = 0; i < myCargoDataShared.myVectorHookPosition.size(); ++i) myCargoDataShared.myVectorHookPosition(i) = 0;
	
	myCargoDataShared.myVectorWinchPosition = myVectorDefaultWinchPosition;
	myCargo.myVectorDisplayAngle = myVectorZeroVector;
	myHook.myVectorDisplayAngle = myVectorZeroVector;
	
	myCargo.myVectorDisplayOffset = myVectorZeroVector;
	myHook.myVectorDisplayOffset = myVectorZeroVector;

	myCargo.myResetTime = true;
	myHook.myResetTime = true;
	myCargoDataShared.myFrameTimeMax = 0;

	for (unsigned i = 0; i < myVectorZeroVector.size(); ++i) myVectorZeroVector(i) = 0;

	// reset rope length

	myCargoDataShared.myRopeLengthNormal = myCargoDataShared.myRopeLengthStart;


	

	// load hook
	myCargo.myRopeConnected = true;
	myCargo.myFollowOnly = true;
	myCargo.myDrawingEnabled = false;

	myHook.myRopeConnected = true;
	myHook.myFollowOnly = false;
	myHook.myDrawingEnabled = true;

	myCargoDataShared.myRopeRuptured = false;

	ReadDataRefs();
	myCargoDataShared.myNewFrame = true;

	myLastLocalX = myCargoDataShared.myLdLocalX;
	myLastLocalY = myCargoDataShared.myLdLocalY;
	myLastLocalZ = myCargoDataShared.myLdLocalZ;

	myCargoDataShared.myCurrentRopeLength = myCargoDataShared.myRopeLengthNormal;
	AircraftConfigRead();
	myCargoDataShared.myVectorHookPosition = AircraftToWorld(myCargoDataShared.myVectorWinchPosition);
	myCargoDataShared.myVectorHookPosition(VERT_AXIS) -= myCargoDataShared.myRopeLengthNormal * 0.5;


	myWinchDirection = HSL::Stop;

	myCargo.myVectorPosition = myCargoDataShared.myVectorHookPosition;
	myCargo.myVectorVelocity = myVectorZeroVector;
	myCargo.myVectorHelicopterPositionDeviation = myVectorZeroVector;
	myCargo.myVectorDrawPosition = myCargoDataShared.myVectorHookPosition;
	myHook.myVectorPosition = myCargoDataShared.myVectorHookPosition;
	myHook.myVectorVelocity = myVectorZeroVector;
	myHook.myVectorHelicopterPositionDeviation = myVectorZeroVector;
	myHook.myVectorDrawPosition = myCargoDataShared.myVectorHookPosition;


	myCargoDataShared.myDebugValue1 = 0.0;
	myCargoDataShared.myDebugValue2 = 0.0;
	myCargoDataShared.myDebugValue3 = 0.0;
	myCargoDataShared.myDebugValue4 = 0.0;
	myCargoDataShared.myDebugStatement = true;

	UpdateObjects();
	
	myCargo.myRopeConnected = true;
	myCargo.myFollowOnly = true;
	myCargo.myDrawingEnabled = false;

	myHook.myRopeConnected = true;
	myHook.myFollowOnly = false;
	myHook.myDrawingEnabled = true;

}


void HSL_PlugIn::SlingConnect()
{
	CARGO_SHM_SECTION_START
	if (myCargo.myRopeConnected == false)
	{
		vector<double> distanceVector = myCargoDataShared.myVectorHelicopterPosition - myCargo.myVectorPosition - myCargo.myVectorHelicopterPositionDeviation;

		double rope_needed = norm_2(distanceVector);
		if (rope_needed > myCargoDataShared.myRopeLengthNormal) myCargoDataShared.myRopeLengthNormal = rope_needed * 1.05;
		myCargoDataShared.myVectorHookPosition = myCargo.myVectorPosition + myCargo.myVectorHelicopterPositionDeviation;
	}

	myCargo.myRopeConnected = true;
	myCargo.myFollowOnly = false;
	myCargo.myDrawingEnabled = true;

	myHook.myRopeConnected = true;
	myHook.myFollowOnly = true;
	myHook.myDrawingEnabled = false;
}

void HSL_PlugIn::SlingRelease()
{
	CARGO_SHM_SECTION_START
	myCargo.myRopeConnected = false;
	myCargo.myFollowOnly = false;
	myCargo.myDrawingEnabled = true;

	myHook.myRopeConnected = true;
	myHook.myFollowOnly = false;
	myHook.myDrawingEnabled = true;
}

void HSL_PlugIn::UpdateParameters()
{

	UpdateObjects();
}

void HSL_PlugIn::UpdateObjects()
{
	////////////////////////////////
	CARGO_SHM_SECTION_START
	try
	{

		HSLDebugString("Reset: Unloading Instances");

		DrawInstanceDestroy(myHookInstanceRef);
		DrawInstanceDestroy(myCargoInstanceRef);
		DrawInstanceDestroy(myWinchInstanceRef);
		DrawInstanceDestroy(myHookInstanceRef);

		for (int rain_index = 0; rain_index < HSL_RAINDROPS_DRAW_MAX; rain_index++)
		{
			DrawInstanceDestroy(myRaindrops[rain_index].myDrawInstance);
		}

		for (int index = 0; index < HSL_ROPE_POINTS_MAX; index++)
		{
			DrawInstanceDestroy(myRopeInstances[index]);
		}

		// Load Objects
		if (myGroundProbe != NULL)
		{
			XPLMDestroyProbe(myGroundProbe);
			myGroundProbe = NULL;
		}

		if (myWinchObjectRef != NULL)
		{
			XPLMUnloadObject(myWinchObjectRef);
			myWinchObjectRef = NULL;
		}

		if (myRopeObjectRef != NULL)
		{
			XPLMUnloadObject(myRopeObjectRef);
			myRopeObjectRef = NULL;
		}

		if (myHookObjectRef != NULL)
		{
			XPLMUnloadObject(myHookObjectRef);
			myHookObjectRef = NULL;
		}

		if (myCargoObjectRef != NULL)
		{
			XPLMUnloadObject(myCargoObjectRef);
			myCargoObjectRef = NULL;
		}

		if (myRaindropObjectRef != NULL)
		{
			XPLMUnloadObject(myRaindropObjectRef);
			myRaindropObjectRef = NULL;
		}

		HSLDebugString("Reset: Winch Object Lookup Start");
		XPLMLookupObjects(myWinchPath.c_str(), 0, 0, load_cb, &myWinchObjectRef);
		XPLMLookupObjects(myRopePath.c_str(), 0, 0, load_cb, &myRopeObjectRef);
		XPLMLookupObjects(myHookPath.c_str(), 0, 0, load_cb, &myHookObjectRef);
		XPLMLookupObjects(myCargoPath.c_str(), 0, 0, load_cb, &myCargoObjectRef);
		XPLMLookupObjects(myRaindropPath.c_str(), 0, 0, load_cb, &myRaindropObjectRef);

		if (myCargoObjectRef == NULL)		myCargoObjectRef = XPLMLoadObject(myCargoPath.c_str());
		if (myHookObjectRef == NULL)		myHookObjectRef  = XPLMLoadObject(myHookPath.c_str());
		if (myRopeObjectRef == NULL)		myRopeObjectRef  = XPLMLoadObject(myRopePath.c_str());
		if (myWinchObjectRef == NULL)		myWinchObjectRef = XPLMLoadObject(myWinchPath.c_str());
		if (myRaindropObjectRef == NULL)	myRaindropObjectRef = XPLMLoadObject(myRaindropPath.c_str());


		myGroundProbe = XPLMCreateProbe(xplm_ProbeY);
		HSLDebugString("Reset: Winch Object Lookup Finished");

		if ((!myWinchObjectRef) || (!myRopeObjectRef) || (!myHookObjectRef) || (!myCargoObjectRef) || (!myRaindropObjectRef))
		{
			HSLDebugString("Reset: Winch Object Lookup Failed");
			SlingDisable();
		}
	}
	catch (...)
	{
		HSLDebugString("Exception in Object Lookup");
		myWinchInstanceRef = NULL;
		myWinchObjectRef = NULL;
		myRopeObjectRef = NULL;
		myHookObjectRef = NULL;
		myCargoObjectRef = NULL;
		myGroundProbe = NULL;
		SlingDisable();

	}

	/*if (!myWinchObjectRef)
	{
		HSLDebugString("Reset: Winch Object: Nothing found");
	}
	else
	{

		HSLDebugString("Reset: Winch Object Lookup Finished");
		HSLDebugString("Creating Instance");

		const char* drefs[] = { NULL, NULL };
		myWinchInstanceRef = XPLMCreateInstance(myWinchObjectRef, drefs);

		
	}*/
}

void HSL_PlugIn::CargoPlaceOnGround()
{
	CARGO_SHM_SECTION_START
	if (myGroundProbe == NULL) return;

	// Place Load 10m before Aircraft
	vector<double> placement(3);
	placement(0) = 0;
	placement(1) = 0;
	placement(2) = -10;
	placement = AircraftToWorld(placement);

	XPLMProbeInfo_t info;
	info.structSize = sizeof(info);
	XPLMProbeResult	result = XPLMProbeTerrainXYZ(myGroundProbe, (float)placement(0), (float)placement(1), (float)placement(2), &info);
	
	myCargo.myVectorPosition(0) = info.locationX;
	myCargo.myVectorPosition(1) = info.locationY;
	myCargo.myVectorPosition(2) = info.locationZ;

	myCargo.myRopeConnected = false;
	myCargo.myFollowOnly = false;
	myCargo.myDrawingEnabled = true;
}

void HSL_PlugIn::CargoPlaceCoordinates()
{
	CARGO_SHM_SECTION_START
	if (myGroundProbe == NULL) return;

	double zero_x, zero_y, zero_z;
	XPLMWorldToLocal(myCargoDataShared.myCargoSetLatitutde, myCargoDataShared.myCargoSetLongitude, 0, &zero_x, &zero_y, &zero_z);

	XPLMProbeInfo_t info;
	info.structSize = sizeof(info);

	XPLMProbeResult result = XPLMProbeTerrainXYZ(myGroundProbe, (float)zero_x, (float)zero_y, (float)zero_z, &info);

	double local_long;
	double local_lat;
	double local_alt;

	XPLMLocalToWorld(info.locationX, info.locationY, info.locationZ, &local_lat, &local_long, &local_alt);
	XPLMWorldToLocal(myCargoDataShared.myCargoSetLatitutde, myCargoDataShared.myCargoSetLongitude, local_alt, &zero_x, &zero_y, &zero_z); // incorporate elevation 

	myCargo.myVectorPosition(0) = zero_x;
	myCargo.myVectorPosition(2) = zero_z;
	XPLMProbeTerrainXYZ(myGroundProbe, (float) zero_x, (float)zero_y, (float)zero_z, &info); // Once again for improved precision
	myCargo.myVectorPosition(1) = info.locationY;

	myCargo.myRopeConnected = false;
	myCargo.myFollowOnly = false;
	myCargo.myDrawingEnabled = true;
}

void HSL_PlugIn::FirePlaceOnGround()
{
	// Place Load 10m before Aircraft
	vector<double> placement(3);
	placement(0) = 0;
	placement(1) = 0;
	placement(2) = -10;
	placement = AircraftToWorld(placement);

	FirePlaceAtCoordinates(&placement);













	/*int planeTotalCount = 0;
	int planeActiveCount = 0;
	char planeFileName[2048];
	char planePath[2048];
	int aircraftAcquired = -1;


	char loadAircraftPath[2048];
	char nullPath[2048];
	char* pLoadAircraftPath[2];

	pLoadAircraftPath[0] = loadAircraftPath;
	pLoadAircraftPath[1] = NULL;

	strcpy(nullPath, "");
	strcpy(loadAircraftPath, "D:\\X-Plane 11\\Aircraft\\Extra Aircraft\\Fire_Aircraft\\Austins Personal Jet.acf");
	//strcpy(loadAircraftPath, "D:\\X-Plane 11\\Aircraft\\Commercial Aircraft\\A310\\A310.acf");

	XPLMCountAircraft(&planeTotalCount, &planeActiveCount, 0);
	aircraftAcquired = XPLMAcquirePlanes(pLoadAircraftPath, NULL, NULL);
	XPLMSetAircraftModel(2, loadAircraftPath);
	
	
	aircraftAcquired = XPLMAcquirePlanes(pLoadAircraftPath, NULL, NULL);

	FireData newFire;


	newFire.drawState.structSize = sizeof(XPLMPlaneDrawState_t);

	newFire.drawState.flapRatio = 0;
	newFire.drawState.gearPosition = 0;
	newFire.drawState.slatRatio = 0;
	newFire.drawState.speedBrakeRatio = 0;
	newFire.drawState.spoilerRatio = 0;
	newFire.drawState.thrust = 0;
	newFire.drawState.wingSweep = 0;
	newFire.drawState.yokeHeading = 0;
	newFire.posX = myLdLocalX - 1.0;
	newFire.posY = myLdLocalY;
	newFire.posZ = myLdLocalZ + 1.0;

	XPLMDisableAIForPlane(2);

	newFire.drXPos = XPLMFindDataRef("sim/multiplayer/position/plane2_x");
	newFire.drYPos = XPLMFindDataRef("sim/multiplayer/position/plane2_y");
	newFire.drZPos = XPLMFindDataRef("sim/multiplayer/position/plane2_z");


	newFire.drXV = XPLMFindDataRef("sim/multiplayer/position/plane2_v_x");
	newFire.drYV = XPLMFindDataRef("sim/multiplayer/position/plane2_v_y");
	newFire.drZV = XPLMFindDataRef("sim/multiplayer/position/plane2_v_z");

	newFire.drStrobe = XPLMFindDataRef("sim/multiplayer/position/plane2_strobe_lights_on");
	newFire.drNav = XPLMFindDataRef("sim/multiplayer/position/plane2_nav_lights_on");
	newFire.drTaxi = XPLMFindDataRef("sim/multiplayer/position/plane2_taxi_light_on");
	newFire.drLanding = XPLMFindDataRef("sim/multiplayer/position/plane2_landing_lights_on");
	newFire.drBeacon = XPLMFindDataRef("sim/multiplayer/position/plane2_beacon_lights_on");

	newFire.planeIndex = 2;
	newFire.waterPercent = 0;

	//XPLMDrawAircraft(newFire.planeIndex, newFire.posX, newFire.posY, newFire.posZ, 0, 0, 0, 1, &newFire.drawState);

	
	XPLMSetDataf(newFire.drXPos, newFire.posX);
	XPLMSetDataf(newFire.drYPos, newFire.posY);
	XPLMSetDataf(newFire.drZPos, newFire.posZ);

	XPLMSetDataf(newFire.drXV, 0);
	XPLMSetDataf(newFire.drYV, 0);
	XPLMSetDataf(newFire.drZV, 0);

	XPLMSetDatai(newFire.drStrobe, 0);
	XPLMSetDatai(newFire.drNav, 0);
	XPLMSetDatai(newFire.drTaxi, 0);
	XPLMSetDatai(newFire.drLanding, 0);
	XPLMSetDatai(newFire.drBeacon, 0);


	myFires.push_back(newFire);
		
	/*XPLMCountAircraft(&planeTotalCount, &planeActiveCount, 0);

	//if (planeActiveCount < planeTotalCount)
	{
		for (int index = 1; index < planeTotalCount; index++)
		{
			XPLMGetNthAircraftModel(index, planeFileName, planePath);
			//strcpy(SDK210TestsAircraftPath[index - 1], planePath);
		}

	}*/
}

void HSL_PlugIn::FirePlaceCoordinates()
{
	FirePlaceAtCoordinates(NULL);
}

void HSL_PlugIn::BambiBucketRelease()
{
	CARGO_SHM_SECTION_START
	myCargo.myBambiBucketRelease = true;
}

void HSL_PlugIn::FirePlaceAtCoordinates(vector<double> * pinVectorFireObjectPosition) // NULL = place at world coordinates
{
	int planeTotalCount = 0;
	int planeActiveCount = 0;
	//char planeFileName[2048];
	//char planePath[2048];
	int aircraftAcquired = -1;
	int planeIndex = -1;


	char loadAircraftPath[2048];
	char* pLoadAircraftPath[2];

	pLoadAircraftPath[0] = loadAircraftPath;
	pLoadAircraftPath[1] = NULL;

	strcpy(loadAircraftPath, myFireAircraftPath.c_str());

	XPLMCountAircraft(&planeTotalCount, &planeActiveCount, 0);

	for (int index = 1; index < planeTotalCount; index++)
	{
		bool isUsed = false;
		for (auto pFire : myFires)
		{
			if (pFire->myPlaneIndex == index)
				isUsed = true;
		}

		if (isUsed == false)
		{
			planeIndex = index;
			index = planeTotalCount;
		}

	}

	// If Aircraft was found
	if (planeIndex > 0)
	{
		aircraftAcquired = XPLMAcquirePlanes(pLoadAircraftPath, NULL, NULL);
		XPLMSetAircraftModel(planeIndex, loadAircraftPath);
		aircraftAcquired = XPLMAcquirePlanes(pLoadAircraftPath, NULL, NULL);

		XPLMDisableAIForPlane(planeIndex);

		if (pinVectorFireObjectPosition != NULL)
		{
			FireObject* pNewFire = new FireObject(this, planeIndex, *pinVectorFireObjectPosition, false);
			myFires.push_back(pNewFire);
		}
		else
		{
			FireObject* pNewFire = new FireObject(this, planeIndex, myVectorZeroVector, true);
			myFires.push_back(pNewFire);
		}

		myFireCreateFailed = 0;
	}
	else
	{
		myFireCreateFailed = 1;
	}

}

int HSL_PlugIn::WinchUpCallback(XPLMCommandRef cmd, XPLMCommandPhase phase, void* refcon)
{
	if (phase == xplm_CommandBegin) myWinchDirection = HSL::Up;
	return 1;
}

int HSL_PlugIn::WinchDownCallback(XPLMCommandRef cmd, XPLMCommandPhase phase, void* refcon)
{
	if (phase == xplm_CommandBegin) myWinchDirection = HSL::Down;
	return 1;
}

int HSL_PlugIn::WinchStopCallback(XPLMCommandRef cmd, XPLMCommandPhase phase, void* refcon)
{
	if (phase == xplm_CommandBegin) myWinchDirection = HSL::Stop;
	return 1;
}

int HSL_PlugIn::EnableCallback(XPLMCommandRef cmd, XPLMCommandPhase phase, void* refcon)
{
	if (phase == xplm_CommandBegin) SlingEnable();
	return 1;
}

int HSL_PlugIn::DisableCallback(XPLMCommandRef cmd, XPLMCommandPhase phase, void* refcon)
{
	if (phase == xplm_CommandBegin) SlingDisable();
	return 1;
}

int HSL_PlugIn::ResetCallback(XPLMCommandRef cmd, XPLMCommandPhase phase, void* refcon)
{
	if (phase == xplm_CommandBegin) SlingReset();
	return 1;
}

int HSL_PlugIn::ConnectLoadCallback(XPLMCommandRef cmd, XPLMCommandPhase phase, void* refcon)
{
	if (phase == xplm_CommandBegin) SlingConnect();
	return 1;
}

int HSL_PlugIn::ReleaseLoadCallback(XPLMCommandRef cmd, XPLMCommandPhase phase, void* refcon)
{
	if (phase == xplm_CommandBegin) SlingRelease();
	return 1;
}

int HSL_PlugIn::LoadGroundCallback(XPLMCommandRef cmd, XPLMCommandPhase phase, void* refcon)
{
	if (phase == xplm_CommandBegin) CargoPlaceOnGround();
	return 1;
}

int HSL_PlugIn::LoadCoordinatesCallback(XPLMCommandRef cmd, XPLMCommandPhase phase, void* refcon)
{
	if (phase == xplm_CommandBegin) CargoPlaceCoordinates();
	return 1;
}

int HSL_PlugIn::ToggleControlWindowCallback(XPLMCommandRef cmd, XPLMCommandPhase phase, void* refcon)
{
	if (phase == xplm_CommandBegin)
	{
		if (!imguiPtr->GetVisible())
		{
			imguiPtr->Visible(true);
		}
		else
		{
			imguiPtr->Visible(false);
		}
	}
	return 1;
}

int HSL_PlugIn::UpdateObjectsCallback(XPLMCommandRef cmd, XPLMCommandPhase phase, void* refcon)
{
	if (phase == xplm_CommandBegin) UpdateObjects();
	return 1;
}

/*void HSL_PlugIn::RegisterDoubleDataref(double &valueIn, std::string nameIn)
{
	CARGO_SHM_SECTION_START
	myRegisteredDatarefs.push_back(
		XPLMRegisterDataAccessor(nameIn.c_str(), xplmType_Double, 1, NULL, NULL, NULL, NULL, WrapReaddoubleCallback, WrapWritedoubleCallback, NULL, NULL, NULL, NULL, NULL, NULL, &valueIn, &valueIn)
	);


	if (myDataRefEditorPluginID == XPLM_NO_PLUGIN_ID) myDataRefEditorPluginID = XPLMFindPluginBySignature("xplanesdk.examples.DataRefEditor");
	if (myDataRefEditorPluginID != XPLM_NO_PLUGIN_ID)
	{
		XPLMSendMessageToPlugin(myDataRefEditorPluginID, MSG_ADD_DATAREF, (void*)nameIn.c_str());
	}
}*/

void HSL_PlugIn::RegisterDoubleArrayDataref(double *valueIn, std::string nameIn, int sizeIn)
{
	CARGO_SHM_SECTION_START
	myRegisteredDatarefs.push_back(
		XPLMRegisterDataAccessor(nameIn.c_str(), xplmType_FloatArray, 1, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, WrapReaddoubleArrayCallback, WrapWritedoubleArrayCallback, NULL, NULL, valueIn, valueIn)
	);


	if (myDataRefEditorPluginID == XPLM_NO_PLUGIN_ID) myDataRefEditorPluginID = XPLMFindPluginBySignature("xplanesdk.examples.DataRefEditor");
	if (myDataRefEditorPluginID != XPLM_NO_PLUGIN_ID)
	{
		XPLMSendMessageToPlugin(myDataRefEditorPluginID, MSG_ADD_DATAREF, (void*)nameIn.c_str());
	}
}

void HSL_PlugIn::RegisterDoubleDataref(double& valueIn, std::string nameIn)
{
	CARGO_SHM_SECTION_START
	myRegisteredDatarefs.push_back(
		XPLMRegisterDataAccessor(nameIn.c_str(), xplmType_Double, 1, NULL, NULL, NULL, NULL, WrapReadDoubleCallback, WrapWriteDoubleCallback, NULL, NULL, NULL, NULL, NULL, NULL, &valueIn, &valueIn)
	);


	if (myDataRefEditorPluginID == XPLM_NO_PLUGIN_ID) myDataRefEditorPluginID = XPLMFindPluginBySignature("xplanesdk.examples.DataRefEditor");
	if (myDataRefEditorPluginID != XPLM_NO_PLUGIN_ID)
	{
		XPLMSendMessageToPlugin(myDataRefEditorPluginID, MSG_ADD_DATAREF, (void*)nameIn.c_str());
	}
}

void HSL_PlugIn::RegisterVectorDataref(vector<double>& vectorIn, std::string nameIn)
{
	CARGO_SHM_SECTION_START
	myRegisteredDatarefs.push_back(
		XPLMRegisterDataAccessor(nameIn.c_str(), xplmType_FloatArray, 1, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, WrapReadVectordoubleCallback, WrapWriteVectordoubleCallback, NULL, NULL, &vectorIn, &vectorIn)
	);


	if (myDataRefEditorPluginID == XPLM_NO_PLUGIN_ID) myDataRefEditorPluginID = XPLMFindPluginBySignature("xplanesdk.examples.DataRefEditor");
	if (myDataRefEditorPluginID != XPLM_NO_PLUGIN_ID)
	{
		XPLMSendMessageToPlugin(myDataRefEditorPluginID, MSG_ADD_DATAREF, (void*)nameIn.c_str());
	}
}

void HSL_PlugIn::RegisterIntDataref(bool& valueIn, std::string nameIn)
{
	CARGO_SHM_SECTION_START
	myRegisteredDatarefs.push_back(
		XPLMRegisterDataAccessor(nameIn.c_str(), xplmType_Int, 1, WrapReadIntCallback, WrapWriteIntCallback, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, &valueIn, &valueIn)
	);


	if (myDataRefEditorPluginID == XPLM_NO_PLUGIN_ID) myDataRefEditorPluginID = XPLMFindPluginBySignature("xplanesdk.examples.DataRefEditor");
	if (myDataRefEditorPluginID != XPLM_NO_PLUGIN_ID)
	{
		XPLMSendMessageToPlugin(myDataRefEditorPluginID, MSG_ADD_DATAREF, (void*)nameIn.c_str());
	}
}

void HSL_PlugIn::RegisterStringDataref(std::string& valueIn, std::string nameIn)
{
	CARGO_SHM_SECTION_START
	myRegisteredDatarefs.push_back(
		XPLMRegisterDataAccessor(nameIn.c_str(), xplmType_Data, 1, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, WrapReadStringCallback, WrapWriteStringCallback, &valueIn, &valueIn)
	);


	if (myDataRefEditorPluginID == XPLM_NO_PLUGIN_ID) myDataRefEditorPluginID = XPLMFindPluginBySignature("xplanesdk.examples.DataRefEditor");
	if (myDataRefEditorPluginID != XPLM_NO_PLUGIN_ID)
	{
		XPLMSendMessageToPlugin(myDataRefEditorPluginID, MSG_ADD_DATAREF, (void*)nameIn.c_str());
	}
}

vector<double> HSL_PlugIn::TurnWorldToAircraft(vector<double> coordsAircraft, CargoDataShared & cargoDataSharedIn)
{
	CARGO_SHM_SECTION_START
	vector<double> world_coords = vector<double>(3);

	double aircraft_x = coordsAircraft(0);
	double aircraft_y = coordsAircraft(1);
	double aircraft_z = coordsAircraft(2);

	// Calculate Position in OpenGL Coordinates
	double x_phi = aircraft_x * cos((2.0 * M_PI) - cargoDataSharedIn.myLfLocalPhi) + aircraft_y * sin((2.0 * M_PI) - cargoDataSharedIn.myLfLocalPhi);
	double y_phi = aircraft_y * cos((2.0 * M_PI) - cargoDataSharedIn.myLfLocalPhi) - aircraft_x * sin((2.0 * M_PI) - cargoDataSharedIn.myLfLocalPhi);
	double z_phi = aircraft_z;
	double x_the = x_phi;
	double y_the = y_phi * cos((2.0 * M_PI) - cargoDataSharedIn.myLfLocalTheta) - z_phi * sin((2.0 * M_PI) - cargoDataSharedIn.myLfLocalTheta);
	double z_the = z_phi * cos((2.0 * M_PI) - cargoDataSharedIn.myLfLocalTheta) + y_phi * sin((2.0 * M_PI) - cargoDataSharedIn.myLfLocalTheta);
	double x_winch_offset = x_the * cos((2.0 * M_PI) - cargoDataSharedIn.myLfLocalPsi) - z_the * sin((2.0 * M_PI) - cargoDataSharedIn.myLfLocalPsi);
	double y_winch_offset = y_the;
	double z_winch_offset = z_the * cos((2.0 * M_PI) - cargoDataSharedIn.myLfLocalPsi) + x_the * sin((2.0 * M_PI) - cargoDataSharedIn.myLfLocalPsi);

	double x_world = x_winch_offset;
	double y_world = y_winch_offset;
	double z_world = z_winch_offset;

	world_coords(0) = x_world;
	world_coords(1) = y_world;
	world_coords(2) = z_world;


	return world_coords;
}

vector<double> HSL_PlugIn::AircraftToWorld(vector<double> coordsAircraft)
{
	CARGO_SHM_SECTION_START
	vector<double> world_coords = vector<double>(3);

	double aircraft_x = coordsAircraft(0);
	double aircraft_y = coordsAircraft(1);
	double aircraft_z = coordsAircraft(2);

	// Calculate Position in OpenGL Coordinates
	double x_phi = aircraft_x * cos(myCargoDataShared.myLfLocalPhi) + aircraft_y * sin(myCargoDataShared.myLfLocalPhi);
	double y_phi = aircraft_y * cos(myCargoDataShared.myLfLocalPhi) - aircraft_x * sin(myCargoDataShared.myLfLocalPhi);
	double z_phi = aircraft_z;
	double x_the = x_phi;
	double y_the = y_phi * cos(myCargoDataShared.myLfLocalTheta) - z_phi * sin(myCargoDataShared.myLfLocalTheta);
	double z_the = z_phi * cos(myCargoDataShared.myLfLocalTheta) + y_phi * sin(myCargoDataShared.myLfLocalTheta);
	double x_winch_offset = x_the * cos(myCargoDataShared.myLfLocalPsi) - z_the * sin(myCargoDataShared.myLfLocalPsi);
	double y_winch_offset = y_the;
	double z_winch_offset = z_the * cos(myCargoDataShared.myLfLocalPsi) + x_the * sin(myCargoDataShared.myLfLocalPsi);

	double x_world = x_winch_offset + myCargoDataShared.myLdLocalX;
	double y_world = y_winch_offset + myCargoDataShared.myLdLocalY;
	double z_world = z_winch_offset + myCargoDataShared.myLdLocalZ;

	world_coords(0) = x_world;
	world_coords(1) = y_world;
	world_coords(2) = z_world;


	return world_coords;

	/*
	following formula:

INPUTS: (x_plane,y_plane,z_plane) = source location in airplane coordinates.  
        phi = roll, psi = heading, the = pitch.  
        (local_x, local_y, local_z) = plane's location in the world 
OUTPUTS:(x_wrl, y_wrl, z_wrl) = transformed location in world.
x_phi=x_plane*cos(phi) + y_plane*sin(phi)
y_phi=y_plane*cos(phi) - x_plane*sin(phi)
z_phi=z_plane
x_the=x_phi
y_the=y_phi*cos(the) - z_phi*sin(the)
z_the=z_phi*cos(the) + y_phi*sin(the)
x_wrl=x_the*cos(psi) - z_the*sin(psi) + local_x
y_wrl=y_the                           + local_y
z_wrl=z_the*cos(psi) + x_the*sin(psi) + local_z
	
	
	*/
}

vector<double> HSL_PlugIn::AdjustFrameMovement(vector<double> coordsAircraft)
{
	CARGO_SHM_SECTION_START
	vector<double> world_coords = coordsAircraft;

	world_coords(0) += myCargoDataShared.myLdLocalX - myLastLocalX;
	world_coords(1) += myCargoDataShared.myLdLocalY - myLastLocalY;
	world_coords(2) += myCargoDataShared.myLdLocalZ - myLastLocalZ;

	return world_coords;
}

float HSL_PlugIn::PluginFlightLoopCallback(double elapsedMe, double elapsedSim, int counter, void* refcon)
{
	CARGO_SHM_SECTION_START
	HIGH_PERFORMANCE = myCargoDataShared.myHighPerformace;
	SetHighPerformance(HIGH_PERFORMANCE);
	auto time_start = std::chrono::steady_clock::now();
	
	ReadDataRefs();

	if ((mySlingLineEnabled == true) && (myCargoDataShared.myLiPause == 0))
	{
		

		myCargoDataShared.myDebugStatement = true;
		myCargoDataShared.myFrameTime = elapsedMe;

		if (myDropThreadObject.myDropObjectIn.size() > 0) myRainDropOverflow++;

		

		// Move the Winch
		if (myWinchDirection == HSL::Down)
		{
			myCargoDataShared.myRopeLengthNormal += myCargoDataShared.myWinchSpeed * myCargoDataShared.myFrameTime;
		}
		else if (myWinchDirection == HSL::Up)
		{
			myCargoDataShared.myRopeLengthNormal -= myCargoDataShared.myWinchSpeed * myCargoDataShared.myFrameTime;
			
		}
		if (myCargoDataShared.myRopeLengthNormal < 0.1f) myCargoDataShared.myRopeLengthNormal = 0.1;

		// Get the new position of the winch in world coodinates		
		myCargoDataShared.myVectorHelicopterPosition = AircraftToWorld(myCargoDataShared.myVectorWinchPosition);

		if (myCargoDataShared.myComputationRunFlag == false)
		{
			myCargoDataShared.myComputationRunFlag = true;
			myCargo.myVectorHelicopterPositionDeviation = myVectorZeroVector;
			myHook.myVectorHelicopterPositionDeviation = myVectorZeroVector;
			myCargo.myVectorHelicopterPositionApprox = myCargoDataShared.myVectorHelicopterPosition;
			myHook.myVectorHelicopterPositionApprox = myCargoDataShared.myVectorHelicopterPosition;

		}


		//double lat1, lon1, alt;
		//double x1, y1, z1;


		// Get the wind velocity vector
		/*XPLMLocalToWorld(myVectorHelicopterPosition(0), myVectorHelicopterPosition(1), myVectorHelicopterPosition(2), &lat1, &lon1, &alt);

		double lat_offset = 0.1 * cos(myLfWindDirection * M_PI / 180.0);
		double lon_offset = 0.1 * sin(myLfWindDirection * M_PI / 180.0);

		XPLMWorldToLocal(lat1 + lat_offset, lon1 + lon_offset, alt, &x1, &y1, &z1);

		vector<double>  vectorWindDirection = myVectorZeroVector;

		vectorWindDirection(0) = x1 - myVectorHelicopterPosition(0);
		vectorWindDirection(1) = y1 - myVectorHelicopterPosition(1);
		vectorWindDirection(2) = z1 - myVectorHelicopterPosition(2);

		vector<double>  vectorUnitWind = -1 * get_unit_vector(vectorWindDirection);

		myVectorWindVelocity = vectorUnitWind * myLfWindSpeed;*/

		// Check if load is on the ground. Has to be here, as other thread must not use SDK

		// Hook:

		vector<double> vectorHookPosition = myHook.myVectorDrawPosition;
		myHook.myTerrainHit = false;
		myHook.myWaterLevel = 0;
		XPLMProbeInfo_t info;
		info.structSize = sizeof(info);
		XPLMProbeResult	result = XPLMProbeTerrainXYZ(myGroundProbe, (float)vectorHookPosition(0), (float)vectorHookPosition(1), (float)vectorHookPosition(2), &info);
		myHook.myObjectTerrainLevel = info.locationY + myHook.myHeight;
		myHook.myTerrainIsWet = info.is_wet;


		// Cargo:
		vector<double> vectorCargoPosition = myCargo.myVectorDrawPosition;
		myCargo.myTerrainHit = false;
		myCargo.myWaterLevel = 0;
		result = XPLMProbeTerrainXYZ(myGroundProbe, (float)vectorCargoPosition(0), (float)vectorCargoPosition(1), (float)vectorCargoPosition(2), &info);
		myCargo.myObjectTerrainLevel = info.locationY + myCargo.myHeight;
		myCargo.myTerrainIsWet = info.is_wet;


		myCargoDataShared.myNewFrame = true;
		// Calculate Physics for Cargo and Hook
		//myCargo.CalculatePhysics();
		//myHook.CalculatePhysics();


		// Apply Forces to Helicopter
		myVectorHelicopterForceApplied = myVectorZeroVector;
		myVectorHelicopterMomentumApplied = myVectorZeroVector;

		// Calculate Average Forces over all results
		double forceTime = 0;

		for (myCompuationsPerFlightLoop = 0; myCargoDataShared.myHelicopterForceQueue.size() > 0; myCompuationsPerFlightLoop++)
		{
			ForceData forceData = myCargoDataShared.myHelicopterForceQueue.front();
			myCargoDataShared.myHelicopterForceQueue.pop();
			
			forceTime += forceData.myTimeApplied;
			myVectorHelicopterForceApplied += forceData.myVectorForce * forceData.myTimeApplied;
			myVectorHelicopterMomentumApplied += forceData.myVectorMomentum * forceData.myTimeApplied;
		}

		if (forceTime > 0)
		{
			myVectorHelicopterForceApplied = myVectorHelicopterForceApplied / forceTime;
			myVectorHelicopterMomentumApplied = myVectorHelicopterMomentumApplied / forceTime;

			if (myCargoDataShared.myRopeRuptureForce <= norm_2(myVectorHelicopterForceApplied))
			{
				myCargoDataShared.myRopeRuptured = true;
				myCargoDataShared.myRopeForceScalar = 0.0;
				myCargoDataShared.myCurrentRopeLength = myCargoDataShared.myRopeLengthNormal;
			}
			else
			{


				// Apply Forces
				XPLMSetDataf(myDrForceX, (float)(myLfForceX + myVectorHelicopterForceApplied(0)));
				XPLMSetDataf(myDrForceY, (float)(myLfForceY + myVectorHelicopterForceApplied(1)));
				XPLMSetDataf(myDrForceZ, (float)(myLfForceZ + myVectorHelicopterForceApplied(2)));

				// The momentum axis as not as in my mechanics book, checked by experiment
				XPLMSetDataf(myDrMomentumX, (float)(myLfMomentumX - myVectorHelicopterMomentumApplied(2)));
				XPLMSetDataf(myDrMomentumY, (float) (myLfMomentumY + myVectorHelicopterMomentumApplied(0)));
				XPLMSetDataf(myDrMomentumZ, (float)(myLfMomentumZ - myVectorHelicopterMomentumApplied(1)));
			}
		}

		static auto timeStart = std::chrono::steady_clock::now();
		static double countComps = 0;

		countComps += myCompuationsPerFlightLoop;

		auto timeNow = std::chrono::steady_clock::now();

		double timeMs = (double) std::chrono::duration_cast<std::chrono::milliseconds>(timeNow - timeStart).count();

		if (timeMs > 1000)
		{
			myCargoDataShared.myFrameTimeMax = 0;
			timeStart = timeNow;
			myComputationFrequency = (int) (countComps * timeMs / 1000.0);
			countComps = 0;
		}
		



		// Debug for Air Force

		myCargoDataShared.myDebugValue1 = norm_2(myCargo.myVectorForceAir);
		myCargoDataShared.myDebugValue2 = norm_2(myCargo.myVectorForceAirNew);

		vector<double>  myVectorFinalRope = myCargoDataShared.myVectorHelicopterPosition - myCargoDataShared.myVectorHookPosition;
		vector<double> vectorRopeUnit = get_unit_vector(myVectorFinalRope);
		

		/*//////////////////////////////////////////////////////////////////////////////////////////////////
		//                   Graphics Pre-Calculation:
		myRopePoints.clear();
		vector<double>  myVectorFinalRope = myCargoDataShared.myVectorHelicopterPosition - myCargoDataShared.myVectorHookPosition;
		vector<double> vectorRopeUnit = get_unit_vector(myVectorFinalRope);

		// If the rope ruptured, the empty vector will disable 
		if (myCargoDataShared.myRopeRuptured == false)
		{
			

			double ropeStepSize = norm_2(myVectorFinalRope) / ((double)HSL_ROPE_POINTS_MAX);

			if (ropeStepSize < 0.01f) ropeStepSize = 0.01;

			for (double distance = 0; distance < (myCargoDataShared.myNewRopeLength); distance += ropeStepSize)
			{
				vector<double> position = myCargoDataShared.myVectorHookPosition + (distance * vectorRopeUnit);
				myRopePoints.push_back(position);
			}
		}*/

		//////////////////////////////////////////////////////////////////////////////////////////////////
		//                   Bambi Bucket


		if (myCargo.myBambiBucketWaterLevel <= 0.0) myCargo.myBambiBucketRelease = false;

		if ((myCargo.myBambiBucketRelease == true) && (myCargo.myBambiBucketWaterLevel > 0))
		{
			myRainReleaseTime += myCargoDataShared.myFrameTime;

			

			vector<double> vectorWaterReleasePosition = myVectorBambiBucketReleasePosition; //myCargo.myVectorPosition + myCargo.myVectorHelicopterPositionDeviation - (myCargo.myHeight * vectorRopeUnit);

			if (myRainReleaseTime >= myRainReleasePeriod)
			{
				myRainReleaseTime = 0;
				double totalDirections = 0;

				for (int variance = 1; variance <= myRainVariance; variance++)
				{
					int currentDirections = myRainDirections / variance;
					double currentSpeed = myRainSpeed / variance;

					totalDirections += currentDirections;
					for (int direction = 0; direction < currentDirections; direction++)
					{
						vector<double> rainVelocity = myCargo.myVectorVelocity;
						double offset = ((double)direction) / ((double)currentDirections);
						double angle = 2.0 * M_PI * offset;
						double cos_angle = cos(angle);
						double sin_angle = sin(angle);
						double speed_x = currentSpeed * cos_angle;
						double speed_y = currentSpeed * sin_angle;
						double speed_z = rainVelocity(1);
						rainVelocity(0) += speed_x;
						rainVelocity(2) += speed_y;

						DropObject* p_raindrop = new DropObject(vectorWaterReleasePosition, rainVelocity);
						myDropThreadObject.myDropObjectIn.push(p_raindrop);
					}
				}


				myBambiBucketWaterPerDrop = (myCargoDataShared.myBambiBucketWaterFlow * myRainReleasePeriod) / totalDirections;
			}
		}
		else
		{
			//myBambiBucketWaterPerDrop = 0;
			myRainReleaseTime = 0;
		}

		
		// Handle all drops that we are able to draw
		for (int rain_index = 0; rain_index < HSL_RAINDROPS_DRAW_MAX; rain_index++)
		{
			if (myDropThreadObject.myDropObjectOut.size() > 0)
			{
				DropObject* p_raindrop = myDropThreadObject.myDropObjectOut.pop();

				if (check_below_ground(p_raindrop->myVectorPosition, myGroundProbe) == false)
				{
					myRaindrops[rain_index].dataValid = true;
					myRaindrops[rain_index].myVectorPosition = p_raindrop->myVectorPosition;
					myRaindrops[rain_index].myVectorVelocity = p_raindrop->myVectorVelocity;
					myRaindrops[rain_index].myVectorDisplayAngle = p_raindrop->myVectorDisplayAngle;

					myDropThreadObject.myDropObjectIn.push(p_raindrop);
				}
				// Drop hit ground, check fires
				else
				{
					std::list<FireObject*>::iterator it;
					for (it=myFires.begin(); it != myFires.end();)
					{
						FireObject* pFire = *it;
						if (pFire->CheckWaterDrop(myRaindrops[rain_index].myVectorPosition, myBambiBucketWaterPerDrop))
						{
							// Fire Extinguished
							pFire->Remove();
							it = myFires.erase(it);
							delete pFire;
						}
						else
						{
							it++;
						}
					}

					myRaindrops[rain_index].dataValid = false;
					delete p_raindrop;
				}
			}
			else
			{
				myRaindrops[rain_index].dataValid = false;

			}
		}

		// Remove all fires
		if (myRemoveFires == true)
		{
			std::list<FireObject*>::iterator it;
			for (it = myFires.begin(); it != myFires.end();)
			{
				FireObject* pFire = *it;

				// Fire Extinguished
				pFire->Remove();
				it = myFires.erase(it);
				delete pFire;
			}
		}
		myRemoveFires = false;

		// Handle all undrawn drops
		while ((myDropThreadObject.myDropObjectOut.size() > 0))
		{
			DropObject* p_raindrop = myDropThreadObject.myDropObjectOut.pop();
			if (check_below_ground(p_raindrop->myVectorPosition, myGroundProbe) == false)
			{
				myDropThreadObject.myDropObjectIn.push(p_raindrop);
			}
			else
			{
				delete p_raindrop;
			}
		}	

		myRainDropNumber = myDropThreadObject.myDropObjectIn.size();

		// Handle remaining fires
		myFireCount = (double) myFires.size();

		for (int index = 0; index < MAX_FIRES; index++)
		{
			myFireStrength[index] = -1;
		}

		int fireIndex = 0;

		for (auto pFire : myFires)
		{
			if (myUpdateFirePositions) pFire->UpdateWorldCoordinates();
			pFire->SetPosition();

			if (pFire->myFireStrength < myFireStrengthMax)
				pFire->myFireStrength += myFireStrengthIncrease * myCargoDataShared.myFrameTime;

			if (fireIndex < MAX_FIRES)
			{
				myFireStrength[fireIndex] = pFire->myFireStrength;
				fireIndex++;
			}
		}

		myUpdateFirePositions = 0;

		// Update Flight Loop data for drop thread

		DropHSLData newData;

		newData.myFrameTime = myCargoDataShared.myFrameTime;
		newData.myLfAirDensity = myCargoDataShared.myLfAirDensity;
		newData.myLfGravitation = myCargoDataShared.myLfGravitation;
		newData.myVectorWindVelocity = myCargoDataShared.myVectorWindVelocity;

		myDropThreadObject.myHSLDataIn.push(newData);
		myDropThreadObject.myHSLCommandsIn.push(DropThreadData::ThreadNewHSLData);


		// Start Computation
		myDropThreadObject.myHSLCommandsIn.push(DropThreadData::ThreadCompute);

		// Store to be able to detect movement between flight loop and draw callback
		myLastLocalX = myCargoDataShared.myLdLocalX;
		myLastLocalY = myCargoDataShared.myLdLocalY;
		myLastLocalZ = myCargoDataShared.myLdLocalZ;
	}

	auto time_end = std::chrono::steady_clock::now();
	myProcessingTimeFlightLoop = (int) std::chrono::duration_cast<std::chrono::microseconds>(time_end - time_start).count();

	return -1;
}

void HSL_PlugIn::DrawObjects()
{
	CARGO_SHM_SECTION_START
	vector<double> vectorWinchWorld = AircraftToWorld(myCargoDataShared.myVectorWinchPosition);

	XPLMDrawInfo_t		dr;
	dr.structSize = sizeof(dr);
	dr.x = (float)vectorWinchWorld(0);
	dr.y = (float)vectorWinchWorld(1);
	dr.z = (float)vectorWinchWorld(2);
	dr.pitch = 0;
	dr.heading = 0;
	dr.roll = 0;

	XPLMInstanceSetPosition(myWinchInstanceRef, &dr, &myDummyAngle);
}

void HSL_PlugIn::ReadDataRefs()
{
	CARGO_SHM_SECTION_START

	myCargoDataShared.myLdLocalX = XPLMGetDatad(myDrLocalX);
	myCargoDataShared.myLdLocalY = XPLMGetDatad(myDrLocalY);
	myCargoDataShared.myLdLocalZ = XPLMGetDatad(myDrLocalZ);

	myCargoDataShared.myVectorHelicopterVelocity(0) = myCargoDataShared.myLfLocalXv = XPLMGetDataf(myDrLocalXv);
	myCargoDataShared.myVectorHelicopterVelocity(1) = myCargoDataShared.myLfLocalYv = XPLMGetDataf(myDrLocalYv);
	myCargoDataShared.myVectorHelicopterVelocity(2) = myCargoDataShared.myLfLocalZv = XPLMGetDataf(myDrLocalZv);

	myCargoDataShared.myVectorHelicopterAcceleration(0) = myCargoDataShared.myLfLocalXa = XPLMGetDataf(myDrLocalXa);
	myCargoDataShared.myVectorHelicopterAcceleration(1) = myCargoDataShared.myLfLocalYa = XPLMGetDataf(myDrLocalYa);
	myCargoDataShared.myVectorHelicopterAcceleration(2) = myCargoDataShared.myLfLocalZa = XPLMGetDataf(myDrLocalZa);

	myCargoDataShared.myLfLocalPhi = XPLMGetDataf(myDrLocalPhi) *  M_PI / 180.0;
	myCargoDataShared.myLfLocalPsi = XPLMGetDataf(myDrLocalPsi) * M_PI / 180.0;
	myCargoDataShared.myLfLocalTheta = XPLMGetDataf(myDrLocalTheta) * M_PI / 180.0;

	myCargoDataShared.myLiPause = XPLMGetDatai(myDrPause);
	myCargoDataShared.myLfTimeActual = XPLMGetDatai(myDrTimeActual);
	myCargoDataShared.myLfWindDirection = XPLMGetDataf(myDrWindDirection) * M_PI / 180.0;
	myCargoDataShared.myLfWindSpeed = XPLMGetDataf(myDrWindSpeed) * 1852.0 / 3600.0;


	myLfWindX = XPLMGetDataf(myDrWindX);
	myLfWindY = XPLMGetDataf(myDrWindY);
	myLfWindZ = XPLMGetDataf(myDrWindZ);

	myCargoDataShared.myVectorWindVelocity(0) = myLfWindX;
	myCargoDataShared.myVectorWindVelocity(1) = myLfWindY;
	myCargoDataShared.myVectorWindVelocity(2) = myLfWindZ;

	myLfForceX = XPLMGetDataf(myDrForceX);
	myLfForceY = XPLMGetDataf(myDrForceY);
	myLfForceZ = XPLMGetDataf(myDrForceZ);

	myLfMomentumX = XPLMGetDataf(myDrMomentumX);
	myLfMomentumY = XPLMGetDataf(myDrMomentumY);
	myLfMomentumZ = XPLMGetDataf(myDrMomentumZ);


	myCargoDataShared.myLfAirDensity = XPLMGetDataf(myDrAirDensity);
	myCargoDataShared.myLfGravitation = -1 * XPLMGetDataf(myDrGravitation);


	
}

