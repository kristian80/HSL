#include "HSL_PlugIn.h"


vector<float> get_unit_vector(vector<float>& v_in)
{
	vector<float> ret = v_in;
	float length = norm_2(v_in);
	if (length != 0) ret = v_in / length;
	return ret;
}

vector<float> cross_product(vector<float>& a, vector<float>& b)
{
	vector<float> ret(3);

	ret(0) = a(1) * b(2) - a(2) * b(1);
	ret(1) = a(2) * b(0) - a(0) * b(2);
	ret(2) = a(0) * b(1) - a(1) * b(0);
	return ret;
}


HSL_PlugIn::HSL_PlugIn()
{
	for (unsigned i = 0; i < myVectorZeroVector.size(); ++i) myVectorZeroVector(i) = 0;

	myVectorHelicopterPosition = myVectorZeroVector;
	myVectorHookPosition = myVectorZeroVector;
	myVectorHookVelocity = myVectorZeroVector;
	myVectorForceGravity = myVectorZeroVector;
	myVectorWindVelocity = myVectorZeroVector;
	myVectorWinchPosition = myVectorZeroVector;
	myVectorCargoOffset = myVectorZeroVector;
	myVectorZeroVector = myVectorZeroVector;
	myVectorObjectDisplayOffset = myVectorZeroVector;
	myVectorObjectDisplayAngle = myVectorZeroVector;
	myVectorRotationTest = myVectorZeroVector;
	
	myVectorDefaultWinchPosition = myVectorZeroVector;

	myVectorRope = myVectorZeroVector;
	myVectorForceRope = myVectorZeroVector;
	myVectorAirVelocity = myVectorZeroVector;
	myVectorForceAir = myVectorZeroVector;
	myVectorForceTotal = myVectorZeroVector;
	myVectorHorizontalVelocity = myVectorZeroVector;
	myVectorForceFriction = myVectorZeroVector;
	myVectorAccTotal = myVectorZeroVector;
	myVectorVelocityDelta = myVectorZeroVector;
	myVectorForceChopper = myVectorZeroVector;
	myVectorMomentumChopper = myVectorZeroVector;



	myVectorDefaultWinchPosition(0) = 0.0f;
	myVectorDefaultWinchPosition(1) = -0.75f;
	myVectorDefaultWinchPosition(2) = 0.0f;

	memset(myRopeInstances, 0, sizeof(XPLMInstanceRef) * HSL_ROPE_POINTS_MAX);
	
}

HSL_PlugIn::~HSL_PlugIn()
{
}

void HSL_PlugIn::PluginStart()
{
	myDS = XPLMGetDirectorySeparator();

	char buffer[2048];
	XPLMGetSystemPath(buffer);
	mySystemPath = buffer;

	myConfigPath = mySystemPath + "Resources" + myDS + "plugins" + myDS + "HSL" + myDS;

	
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
	myCmdRefToggleControlWindow = XPLMCreateCommand("HSL/ToogleControlWindow", "Toggle Control Window");
	myCmdRefUpdateObjects = XPLMCreateCommand("HSL/UpdateObjects", "Update Objects");

	XPLMRegisterCommandHandler(myCmdRefWinchUp, WrapWinchUpCallback, 0, 0);
	XPLMRegisterCommandHandler(myCmdRefWinchDown, WrapWinchDownCallback, 0, 0);
	XPLMRegisterCommandHandler(myCmdRefWinchStop, WrapWinchStopCallback, 0, 0);
	XPLMRegisterCommandHandler(myCmdRefEnable, WrapEnableCallback, 0, 0);
	XPLMRegisterCommandHandler(myCmdRefDisable, WrapDisableCallback, 0, 0);
	XPLMRegisterCommandHandler(myCmdRefReset, WrapResetCallback, 0, 0);
	XPLMRegisterCommandHandler(myCmdRefConnectLoad, WrapConnectLoadCallback, 0, 0);
	XPLMRegisterCommandHandler(myCmdRefReleaseLoad, WrapReleaseLoadCallback, 0, 0);
	XPLMRegisterCommandHandler(myCmdRefToggleControlWindow, WrapToggleControlWindowCallback, 0, 0);
	XPLMRegisterCommandHandler(myCmdRefUpdateObjects, WrapUpdateObjectCallback, 0, 0);

	//myDrShRopeLengthStart = XPLMRegisterDataAccessor("HSL/RopeLengthStart", xplmType_Float, 1, NULL, NULL, WrapReadFloatCallback, WrapWriteFloatCallback, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, &myRopeLengthStart, &myRopeLengthStart);

	RegisterFloatDataref(myRopeLengthStart, "HSL/RopeLengthStart");
	



	RegisterFloatDataref(myRopeLengthNormal, "HSL/RopeLengthNormal");
	RegisterFloatDataref(myRopeDamping, "HSL/RopeDamping");
	RegisterFloatDataref(myRopeK, "HSL/RopeK");
	RegisterFloatDataref(myRopeRuptureForce, "HSL/RopeRuptureForce");

	RegisterFloatDataref(myMaxAccRopeFactor, "HSL/MaxAccRopeFactor");

	// Loaded Object
	RegisterFloatDataref(myObjectHeight, "HSL/CurrentLoadHeight");
	RegisterFloatDataref(myObjectMass, "HSL/CurrentLoadMass");
	RegisterFloatDataref(myObjectCrossSection, "HSL/CurrentLoadCrossSection");
	RegisterFloatDataref(myObjectCWFront, "HSL/CurrentLoadCWFront");
	RegisterFloatDataref(myObjectFrictionGlide, "HSL/CurrentLoadFrictionGlide");
	RegisterFloatDataref(myObjectFrictionStatic, "HSL/CurrentLoadFrictionStatic");

	

	// Hook 
	RegisterFloatDataref(myHookHeight, "HSL/HookHeight");
	RegisterFloatDataref(myHookMass, "HSL/HookMass");
	RegisterFloatDataref(myHookCrossSection, "HSL/HookCrossSection");
	RegisterFloatDataref(myHookCWFront, "HSL/HookCWFront");
	RegisterFloatDataref(myHookFrictionGlide, "HSL/HookFrictionGlide");
	RegisterFloatDataref(myHookFrictionStatic, "HSL/HookFrictionStatic");

	// Cargo
	RegisterFloatDataref(myCargoHeight, "HSL/CargoHeight");
	RegisterFloatDataref(myCargoMass, "HSL/CargoMass");
	RegisterFloatDataref(myCargoCrossSection, "HSL/CargoCrossSection");
	RegisterFloatDataref(myCargoCWFront, "HSL/CargoCWFront");
	RegisterFloatDataref(myCargoFrictionGlide, "HSL/CargoFrictionGlide");
	RegisterFloatDataref(myCargoFrictionStatic, "HSL/CargoFrictionStatic");

	RegisterFloatDataref(myWinchSpeed, "HSL/WinchSpeed");
	
	
	RegisterVectorDataref(myVectorWinchPosition, "HSL/VectorWinchPosition");




	RegisterFloatDataref(myObjectSpeedStaticFriction, "HSL/Calculated/CurrentLoadSpeedStaticFriction");
	RegisterVectorDataref(myVectorHelicopterPosition, "HSL/Calculated/VectorHelicopterPosition");
	RegisterVectorDataref(myVectorHookPosition, "HSL/Calculated/VectorHookPosition");
	RegisterVectorDataref(myVectorHookVelocity, "HSL/Calculated/VectorHookVelocity");
	RegisterVectorDataref(myVectorForceGravity, "HSL/Calculated/VectorForceGravity");
	RegisterVectorDataref(myVectorWindVelocity, "HSL/Calculated/VectorWindVelocity");
	RegisterVectorDataref(myVectorWinchPosition, "HSL/Calculated/VectorWinchPosition");
	RegisterVectorDataref(myVectorCargoOffset, "HSL/Calculated/VectorCargoOffset");
	RegisterVectorDataref(myVectorZeroVector, "HSL/Calculated/VectorZeroVector");
	RegisterVectorDataref(myVectorObjectDisplayOffset, "HSL/Calculated/VectorObjectDisplayOffset");
	RegisterVectorDataref(myVectorObjectDisplayAngle, "HSL/Calculated/VectorObjectDisplayAngle");
	RegisterVectorDataref(myVectorRotationTest, "HSL/Calculated/VectorRotationTest");


	/*RegisterVectorDataref(myVectorRope, "HSL/Calculated/VectorRope");
	RegisterVectorDataref(myVectorForceRope, "HSL/Calculated/VectorForceRope");
	RegisterVectorDataref(myVectorAirVelocity, "HSL/Calculated/VectorAirVelocity");
	RegisterVectorDataref(myVectorForceAir, "HSL/Calculated/VectorForceAir");
	RegisterVectorDataref(myVectorForceTotal, "HSL/Calculated/VectorForceTotal");
	RegisterVectorDataref(myVectorHorizontalVelocity, "HSL/Calculated/VectorHorizontalVelocity");
	RegisterVectorDataref(myVectorForceFriction, "HSL/Calculated/VectorForceFriction");
	RegisterVectorDataref(myVectorAccTotal, "HSL/Calculated/VectorAccTotal");
	RegisterVectorDataref(myVectorVelocityDelta, "HSL/Calculated/VectorVelocityDelta");
	RegisterVectorDataref(myVectorForceChopper, "HSL/Calculated/VectorForceChopper");
	RegisterVectorDataref(myVectorMomentumChopper, "HSL/Calculated/VectorMomentumChopper");*/




	RegisterFloatDataref(myFrameTime, "HSL/FrameTime");
	RegisterFloatDataref(myObjectTerrainLevel, "HSL/ObjectTerrainLevel");
	RegisterFloatDataref(myNewRopeLength, "HSL/NewRopeLength");
	RegisterFloatDataref(myRopeStretchRelative, "HSL/RopeStretchRelative");
	RegisterFloatDataref(myRopeForceScalar, "HSL/RopeForceScalar");
	RegisterFloatDataref(myRopeLengthDelta, "HSL/RopeLengthDelta");
	RegisterFloatDataref(myRopeStretchSpeed, "HSL/RopeStretchSpeed");
	RegisterFloatDataref(myRopeCorrectedD, "HSL/RopeCorrectedD");
	RegisterFloatDataref(myAirSpeed, "HSL/AirSpeed");
	RegisterFloatDataref(myAirResistance, "HSL/AirResistance");

	RegisterIntDataref(myRopeRuptured, "HSL/RopeRuptured");
	RegisterIntDataref(myCargoConnected, "HSL/CargoConnected");


	RegisterStringDataref(myWinchPath, "HSL/WinchObjectPath");
	RegisterStringDataref(myRopePath, "HSL/RopeObjectPath");
	RegisterStringDataref(myHookPath, "HSL/HookObjectPath");
	RegisterStringDataref(myCargoPath, "HSL/CargoObjectPath");
	
	/*strcpy(buffer, "RescueX/objects/Bergwacht_Luftrettungssack.obj");
	XPLMDataRef test = XPLMFindDataRef("HSL/RopeObjectPath");

	XPLMSetDatab(test, buffer, 0, 2048);*/


	// Drawing
	XPLMRegisterDrawCallback(WrapDrawCallback, xplm_Phase_Airplanes, 0, NULL);

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

	myDrLocalPhi = XPLMFindDataRef("sim/flightmodel/position/phi");
	myDrLocalPsi = XPLMFindDataRef("sim/flightmodel/position/psi");
	myDrLocalTheta = XPLMFindDataRef("sim/flightmodel/position/theta");

	myDrPause = XPLMFindDataRef("sim/time/paused");
	myDrWindDirection = XPLMFindDataRef("sim/weather/wind_direction_degt");
	myDrWindSpeed = XPLMFindDataRef("sim/weather/wind_speed_kt");

	myDrForceX = XPLMFindDataRef("sim/flightmodel/forces/fside_plug_acf");
	myDrForceY = XPLMFindDataRef("sim/flightmodel/forces/fnrml_plug_acf");
	myDrForceZ = XPLMFindDataRef("sim/flightmodel/forces/faxil_plug_acf");

	myDrMomentumX = XPLMFindDataRef("sim/flightmodel/forces/L_plug_acf");
	myDrMomentumY = XPLMFindDataRef("sim/flightmodel/forces/M_plug_acf");
	myDrMomentumZ = XPLMFindDataRef("sim/flightmodel/forces/N_plug_acf");

	myDrAirDensity = XPLMFindDataRef("sim/weather/rho");
	myDrGravitation = XPLMFindDataRef("sim/weather/gravity_mss");

	ConfigRead();

	myInitialized = true;


}

void HSL_PlugIn::PluginStop()
{
	if (myInitialized == false) return;
	ConfigSave();


	XPLMDestroyMenu(myPluginMenuID);
	for (auto dataRef: myRegisteredDatarefs) XPLMUnregisterDataAccessor(dataRef);



	
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
		if (inMessage == XPLM_MSG_PLANE_LOADED)
		{
			HSLDebugString("Plane loaded");

			myAircraftLoaded = 1;

			

			SlingDisable();

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
		ReadDataRefs();

		vector<float> vectorWinchWorld = AdjustFrameMovement(myVectorHelicopterPosition);
		//vector<float> vectorWinchWorld = myVectorHelicopterPosition;
		DrawInstanceCreate(myWinchInstanceRef, myWinchObjectRef);
		DrawInstanceSetPosition(myWinchInstanceRef, myWinchObjectRef, vectorWinchWorld, false);

		int index;

		for (index = 0; index < HSL_ROPE_POINTS_MAX; index++)
		{
			if (index < myRopePoints.size())
			{
				vector<float> vectorRopePointOpenGL = AdjustFrameMovement(myRopePoints[index]);
				DrawInstanceCreate(myRopeInstances[index], myRopeObjectRef);
				DrawInstanceSetPosition(myRopeInstances[index], myRopeObjectRef, vectorRopePointOpenGL, true);
			}
			else
			{
				DrawInstanceDestroy(myRopeInstances[index]);
			}
		}

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
				vector<float> vectorRopePointOpenGL = AdjustFrameMovement(myRopePoints[index]);

				drawInfo[index].x = vectorRopePointOpenGL(0);
				drawInfo[index].y = vectorRopePointOpenGL(1);
				drawInfo[index].z = vectorRopePointOpenGL(2);
			}

			XPLMDrawObjects(myRopeObjectRef, myRopePoints.size(), drawInfo, 0, 0);
		}*/

		vector<float> vectorCargoPointOpenGL = AdjustFrameMovement(myVectorHookPosition);
		if (myCargoConnected == true)
		{
			DrawInstanceDestroy(myHookInstanceRef);

			vectorCargoPointOpenGL -= myVectorObjectDisplayOffset;

			DrawInstanceCreate(myCargoInstanceRef, myCargoObjectRef);
			DrawInstanceSetPosition(myCargoInstanceRef, myCargoObjectRef, vectorCargoPointOpenGL, myVectorObjectDisplayAngle, true);
			

		}
		else
		{
			DrawInstanceDestroy(myCargoInstanceRef);

			DrawInstanceCreate(myHookInstanceRef, myHookObjectRef);
			DrawInstanceSetPosition(myHookInstanceRef, myHookObjectRef, vectorCargoPointOpenGL, false);
		}
	}

	auto time_end = std::chrono::steady_clock::now();
	myProcessingTimeDrawRoutine = std::chrono::duration_cast<std::chrono::microseconds>(time_end - time_start).count();

	return 1;
}

void HSL_PlugIn::ConfigSave()
{
	try
	{



			boost::property_tree::ptree pt;

			pt.put("Rope.library_object", myRopePath);
			pt.put("Rope.length", myRopeLengthStart);
			pt.put("Rope.damping", myRopeDamping);
			pt.put("Rope.k_factor", myRopeK);
			pt.put("Rope.rupture_force", myRopeRuptureForce);
			pt.put("Rope.max_acc_factor", myMaxAccRopeFactor);

			pt.put("Winch.library_object", myWinchPath);
			pt.put("Winch.speed", myWinchSpeed);

			pt.put("Hook.library_object", myHookPath);
			pt.put("Hook.height", myHookHeight);
			pt.put("Hook.mass", myHookMass);
			pt.put("Hook.cross_section_front", myHookCrossSection);
			pt.put("Hook.cw_front", myHookCWFront);
			pt.put("Hook.friction_glide", myHookFrictionGlide);
			pt.put("Hook.friction_static", myHookFrictionStatic);

			pt.put("Cargo.library_object", myCargoPath);
			pt.put("Cargo.height", myCargoHeight);
			pt.put("Cargo.mass", myCargoMass);
			pt.put("Cargo.cross_section_front", myCargoCrossSection);
			pt.put("Cargo.cw_front", myCargoCWFront);
			pt.put("Cargo.friction_glide", myCargoFrictionGlide);
			pt.put("Cargo.friction_static", myCargoFrictionStatic);
			ConfigWriteVector(pt, myVectorCargoOffset, "Cargo.offset");
			
			ConfigWriteVector(pt, myVectorWinchPosition, "HSL_Aircraft.winch_position");
			
			boost::property_tree::ini_parser::write_ini(myConfigPath + myConfigIniFile, pt);

	}
	catch (...)
	{
		HSLDebugString("Writing aircraft ini file crashed");
	}
}

void HSL_PlugIn::ConfigRead()
{
	boost::property_tree::ptree pt;
	try
	{
		boost::property_tree::ini_parser::read_ini(myConfigPath + myConfigIniFile, pt);

		ConfigReadString(pt, "Rope.library_object", myRopePath);
		ConfigReadFloat(pt, "Rope.length", myRopeLengthStart);
		ConfigReadFloat(pt, "Rope.damping", myRopeDamping);
		ConfigReadFloat(pt, "Rope.k_factor", myRopeK);
		ConfigReadFloat(pt, "Rope.rupture_force", myRopeRuptureForce);
		ConfigReadFloat(pt, "Rope.max_acc_factor", myMaxAccRopeFactor);

		ConfigReadString(pt, "Winch.library_object", myWinchPath);
		ConfigReadFloat(pt, "Winch.speed", myWinchSpeed);

		ConfigReadString(pt, "Hook.library_object", myHookPath);
		ConfigReadFloat(pt, "Hook.height", myHookHeight);
		ConfigReadFloat(pt, "Hook.mass", myHookMass);
		ConfigReadFloat(pt, "Hook.cross_section_front", myHookCrossSection);
		ConfigReadFloat(pt, "Hook.cw_front", myHookCWFront);
		ConfigReadFloat(pt, "Hook.friction_glide", myHookFrictionGlide);
		ConfigReadFloat(pt, "Hook.friction_static", myHookFrictionStatic);

		ConfigReadString(pt, "Cargo.library_object", myCargoPath);
		ConfigReadFloat(pt, "Cargo.height", myCargoHeight);
		ConfigReadFloat(pt, "Cargo.mass", myCargoMass);
		ConfigReadFloat(pt, "Cargo.cross_section_front", myCargoCrossSection);
		ConfigReadFloat(pt, "Cargo.cw_front", myCargoCWFront);
		ConfigReadFloat(pt, "Cargo.friction_glide", myCargoFrictionGlide);
		ConfigReadFloat(pt, "Cargo.friction_static", myCargoFrictionStatic);
		ConfigReadVector(pt, myVectorCargoOffset, "Cargo.offset");
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
			ConfigWriteVector(pt, myVectorWinchPosition, "HSL_Aircraft.winch_position");
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

			ConfigReadVector(pt, myVectorWinchPosition, "HSL_Aircraft.winch_position");
		}
	}
	catch (...)
	{
		HSLDebugString("Writing aircraft ini file crashed");
	}
}

void HSL_PlugIn::ConfigWriteVector(boost::property_tree::ptree& pt, vector<float>& vectorIn, std::string nameIn)
{
	pt.put(nameIn + "_1", vectorIn(0));
	pt.put(nameIn + "_2", vectorIn(1));
	pt.put(nameIn + "_3", vectorIn(2));
}

void HSL_PlugIn::ConfigReadVector(boost::property_tree::ptree& pt, vector<float>& vectorOut, std::string nameIn)
{
	ConfigReadFloat(pt, nameIn + "_1", vectorOut(0));
	ConfigReadFloat(pt, nameIn + "_2", vectorOut(1));
	ConfigReadFloat(pt, nameIn + "_3", vectorOut(2));
}

void HSL_PlugIn::ConfigReadFloat(boost::property_tree::ptree& pt, std::string nameIn, float& floatOut)
{
	try { floatOut = pt.get<float>(nameIn); }
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
	mySlingLineEnabled = true;
	SlingReset();
}

void HSL_PlugIn::SlingDisable()
{
	XPLMCheckMenuItem(myPluginMenuID, myEnableSlingMenu, xplm_Menu_Unchecked);
	mySlingLineEnabled = false;
}

void HSL_PlugIn::SlingReset()
{
	for (unsigned i = 0; i < myVectorHelicopterPosition.size(); ++i) myVectorHelicopterPosition(i) = 0;
	for (unsigned i = 0; i < myVectorHookPosition.size(); ++i) myVectorHookPosition(i) = 0;
	for (unsigned i = 0; i < myVectorHookVelocity.size(); ++i) myVectorHookVelocity(i) = 0;
	for (unsigned i = 0; i < myVectorForceGravity.size(); ++i) myVectorForceGravity(i) = 0;
	for (unsigned i = 0; i < myVectorWindVelocity.size(); ++i) myVectorWindVelocity(i) = 0;
	
	myVectorWinchPosition = myVectorDefaultWinchPosition;
	myVectorObjectDisplayAngle = myVectorZeroVector;
	myVectorObjectDisplayOffset = myVectorZeroVector;

	for (unsigned i = 0; i < myVectorZeroVector.size(); ++i) myVectorZeroVector(i) = 0;

	// reset rope length

	myRopeLengthNormal = myRopeLengthStart;

	// load hook
	myObjectMass = myHookMass;
	myObjectCrossSection = myHookCrossSection;
	myObjectCWFront = myHookCWFront;
	myObjectFrictionGlide = myHookFrictionGlide;
	myObjectFrictionStatic = myHookFrictionStatic;
	myObjectHeight = myHookHeight;


	myCargoConnected = false;
	myRopeRuptured = false;

	ReadDataRefs();

	myLastLocalX = myLdLocalX;
	myLastLocalY = myLdLocalY;
	myLastLocalZ = myLdLocalZ;

	myCurrentRopeLength = myRopeLengthNormal;

	myVectorHookPosition = AircraftToWorld(myVectorWinchPosition);
	myVectorHookPosition(VERT_AXIS) -= myRopeLengthNormal * 0.5;
	myWinchDirection = HSL::Stop;

	//////////////////////////////////////// DEBUG

	//myVectorCargoOffset(0) = 0.0f;
	//myVectorCargoOffset(1) = 0.9f;
	//myVectorCargoOffset(2) =  0.2f;

	AircraftConfigRead();

	myDebugValue1 = 0.0f;
	myDebugValue2 = 0.0f;
	myDebugValue3 = 0.0f;
	myDebugValue4 = 0.0f;
	myDebugStatement = true;



	UpdateObjects();
	SlingRelease();

}

void HSL_PlugIn::SlingConnect()
{
	myCargoConnected = true;
	myObjectMass = myCargoMass;
	myObjectHeight = myCargoHeight;
	myObjectCrossSection = myCargoCrossSection;
	myObjectCWFront = myCargoCWFront;
	myObjectFrictionGlide = myCargoFrictionGlide;
	myObjectFrictionStatic = myCargoFrictionStatic;
}

void HSL_PlugIn::SlingRelease()
{
	myCargoConnected = false;
	myObjectMass = myHookMass;
	myObjectHeight = myHookHeight;
	myObjectCrossSection = myHookCrossSection;
	myObjectCWFront = myHookCWFront;
	myObjectFrictionGlide = myHookFrictionGlide;
	myObjectFrictionStatic = myHookFrictionStatic;
}

void HSL_PlugIn::UpdateParameters()
{

	UpdateObjects();
}

void HSL_PlugIn::UpdateObjects()
{
	////////////////////////////////

	try
	{

		HSLDebugString("Reset: Unloading Instances");

		DrawInstanceDestroy(myHookInstanceRef);
		DrawInstanceDestroy(myCargoInstanceRef);
		DrawInstanceDestroy(myWinchInstanceRef);
		DrawInstanceDestroy(myHookInstanceRef);

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

		HSLDebugString("Reset: Winch Object Lookup Start");
		XPLMLookupObjects(myWinchPath.c_str(), 0, 0, load_cb, &myWinchObjectRef);
		XPLMLookupObjects(myRopePath.c_str(), 0, 0, load_cb, &myRopeObjectRef);
		XPLMLookupObjects(myHookPath.c_str(), 0, 0, load_cb, &myHookObjectRef);
		XPLMLookupObjects(myCargoPath.c_str(), 0, 0, load_cb, &myCargoObjectRef);

		if (myCargoObjectRef == NULL)	myCargoObjectRef = XPLMLoadObject(myCargoPath.c_str());
		if (myHookObjectRef == NULL)	myHookObjectRef  = XPLMLoadObject(myHookPath.c_str());
		if (myRopeObjectRef == NULL)	myRopeObjectRef  = XPLMLoadObject(myRopePath.c_str());
		if (myWinchObjectRef == NULL)	myWinchObjectRef = XPLMLoadObject(myWinchPath.c_str());


		myGroundProbe = XPLMCreateProbe(xplm_ProbeY);
		HSLDebugString("Reset: Winch Object Lookup Finished");

		if ((!myWinchObjectRef) || (!myRopeObjectRef) || (!myHookObjectRef) || (!myCargoObjectRef))
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

void HSL_PlugIn::RegisterFloatDataref(float &valueIn, std::string nameIn)
{
	myRegisteredDatarefs.push_back(
		XPLMRegisterDataAccessor(nameIn.c_str(), xplmType_Float, 1, NULL, NULL, WrapReadFloatCallback, WrapWriteFloatCallback, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, &valueIn, &valueIn)
	);


	if (myDataRefEditorPluginID == XPLM_NO_PLUGIN_ID) myDataRefEditorPluginID = XPLMFindPluginBySignature("xplanesdk.examples.DataRefEditor");
	if (myDataRefEditorPluginID != XPLM_NO_PLUGIN_ID)
	{
		XPLMSendMessageToPlugin(myDataRefEditorPluginID, MSG_ADD_DATAREF, (void*)nameIn.c_str());
	}
}

void HSL_PlugIn::RegisterVectorDataref(vector<float>& vectorIn, std::string nameIn)
{
	myRegisteredDatarefs.push_back(
		XPLMRegisterDataAccessor(nameIn.c_str(), xplmType_FloatArray, 1, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, WrapReadVectorFloatCallback, WrapWriteVectorFloatCallback, NULL, NULL, &vectorIn, &vectorIn)
	);


	if (myDataRefEditorPluginID == XPLM_NO_PLUGIN_ID) myDataRefEditorPluginID = XPLMFindPluginBySignature("xplanesdk.examples.DataRefEditor");
	if (myDataRefEditorPluginID != XPLM_NO_PLUGIN_ID)
	{
		XPLMSendMessageToPlugin(myDataRefEditorPluginID, MSG_ADD_DATAREF, (void*)nameIn.c_str());
	}
}

void HSL_PlugIn::RegisterIntDataref(bool& valueIn, std::string nameIn)
{
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
	myRegisteredDatarefs.push_back(
		XPLMRegisterDataAccessor(nameIn.c_str(), xplmType_Data, 1, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, WrapReadStringCallback, WrapWriteStringCallback, &valueIn, &valueIn)
	);


	if (myDataRefEditorPluginID == XPLM_NO_PLUGIN_ID) myDataRefEditorPluginID = XPLMFindPluginBySignature("xplanesdk.examples.DataRefEditor");
	if (myDataRefEditorPluginID != XPLM_NO_PLUGIN_ID)
	{
		XPLMSendMessageToPlugin(myDataRefEditorPluginID, MSG_ADD_DATAREF, (void*)nameIn.c_str());
	}
}

vector<float> HSL_PlugIn::TurnWorldToAircraft(vector<float> coordsAircraft)
{
	vector<float> world_coords = vector<float>(3);

	double aircraft_x = coordsAircraft(0);
	double aircraft_y = coordsAircraft(1);
	double aircraft_z = coordsAircraft(2);

	// Calculate Position in OpenGL Coordinates
	double x_phi = aircraft_x * cos((2.0 * M_PI) - myLfLocalPhi) + aircraft_y * sin((2.0 * M_PI) -  myLfLocalPhi);
	double y_phi = aircraft_y * cos((2.0 * M_PI) -  myLfLocalPhi) - aircraft_x * sin((2.0 * M_PI) -  myLfLocalPhi);
	double z_phi = aircraft_z;
	double x_the = x_phi;
	double y_the = y_phi * cos((2.0 * M_PI) -  myLfLocalTheta) - z_phi * sin((2.0 * M_PI) -  myLfLocalTheta);
	double z_the = z_phi * cos((2.0 * M_PI) -  myLfLocalTheta) + y_phi * sin((2.0 * M_PI) -  myLfLocalTheta);
	double x_winch_offset = x_the * cos((2.0 * M_PI) -  myLfLocalPsi) - z_the * sin((2.0 * M_PI) -  myLfLocalPsi);
	double y_winch_offset = y_the;
	double z_winch_offset = z_the * cos((2.0 * M_PI) -  myLfLocalPsi) + x_the * sin((2.0 * M_PI) -  myLfLocalPsi);

	double x_world = x_winch_offset;
	double y_world = y_winch_offset;
	double z_world = z_winch_offset;

	world_coords(0) = x_world;
	world_coords(1) = y_world;
	world_coords(2) = z_world;


	return world_coords;
}

vector<float> HSL_PlugIn::AircraftToWorld(vector<float> coordsAircraft)
{
	vector<float> world_coords = vector<float>(3);

	double aircraft_x = coordsAircraft(0);
	double aircraft_y = coordsAircraft(1);
	double aircraft_z = coordsAircraft(2);

	// Calculate Position in OpenGL Coordinates
	double x_phi = aircraft_x * cos(myLfLocalPhi) + aircraft_y * sin(myLfLocalPhi);
	double y_phi = aircraft_y * cos(myLfLocalPhi) - aircraft_x * sin(myLfLocalPhi);
	double z_phi = aircraft_z;
	double x_the = x_phi;
	double y_the = y_phi * cos(myLfLocalTheta) - z_phi * sin(myLfLocalTheta);
	double z_the = z_phi * cos(myLfLocalTheta) + y_phi * sin(myLfLocalTheta);
	double x_winch_offset = x_the * cos(myLfLocalPsi) - z_the * sin(myLfLocalPsi);
	double y_winch_offset = y_the;
	double z_winch_offset = z_the * cos(myLfLocalPsi) + x_the * sin(myLfLocalPsi);

	double x_world = x_winch_offset + myLdLocalX;
	double y_world = y_winch_offset + myLdLocalY;
	double z_world = z_winch_offset + myLdLocalZ;

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

vector<float> HSL_PlugIn::AdjustFrameMovement(vector<float> coordsAircraft)
{
	vector<float> world_coords = coordsAircraft;

	world_coords(0) += myLdLocalX - myLastLocalX;
	world_coords(1) += myLdLocalY - myLastLocalY;
	world_coords(2) += myLdLocalZ - myLastLocalZ;

	return world_coords;
}

float HSL_PlugIn::PluginFlightLoopCallback(float elapsedMe, float elapsedSim, int counter, void* refcon)
{
	auto time_start = std::chrono::steady_clock::now();
	

	if ((mySlingLineEnabled == true) && (myLiPause == 0))
	{
		myDebugStatement = true;
		myFrameTime = elapsedMe;

		ReadDataRefs();

		// Move the Winch
		if (myWinchDirection == HSL::Down)
		{
			myRopeLengthNormal += myWinchSpeed * myFrameTime;
		}
		else if (myWinchDirection == HSL::Up)
		{
			myRopeLengthNormal -= myWinchSpeed * myFrameTime;
			
		}
		if (myRopeLengthNormal < 0.1f) myRopeLengthNormal = 0.1f;

		// Get the new position of the winch in world coodinates		
		myVectorHelicopterPosition = AircraftToWorld(myVectorWinchPosition);

		// Check if load is on the ground
		myTerrainHit = false;
		XPLMProbeInfo_t info;
		info.structSize = sizeof(info);
		XPLMProbeResult	result = XPLMProbeTerrainXYZ(myGroundProbe, myVectorHookPosition(0), myVectorHookPosition(1), myVectorHookPosition(2), &info);
		myObjectTerrainLevel = info.locationY + myObjectHeight;

		// If we are below the ground, correct vertical position and velocity
		if (myVectorHookPosition(VERT_AXIS) <= myObjectTerrainLevel)
		{
			myTerrainHit = true;

			myVectorHookPosition(VERT_AXIS) = myObjectTerrainLevel;
			if (myVectorHookVelocity(VERT_AXIS) < 0) myVectorHookVelocity(VERT_AXIS) = 0; //no velocity targeting below ground

		}

		// Get the rope vector and length. Store length in 2nd variable, as we need old+new values to compute the rope stretch speed for the damping factor
		myVectorRope = myVectorHelicopterPosition - myVectorHookPosition;
		myNewRopeLength = norm_2(myVectorRope);

		// Compute the length stretch
		if (myRopeLengthNormal == 0.0f) return -1;
		myRopeStretchRelative = (myNewRopeLength / myRopeLengthNormal) - 1;
		myRopeForceScalar = 0.0f;
		

		// if rope is streched, calcuate the rope force
		if ((myRopeStretchRelative > 0.0f) && (myRopeRuptured == false))
		{
			myRopeLengthDelta = myNewRopeLength - myCurrentRopeLength;

			if (myFrameTime == 0.0f) return -1;
			
			myRopeStretchSpeed = myRopeLengthDelta / myFrameTime;

			if (myRopeStretchSpeed < 0.0f) myDebugValue3 = 100;
			myRopeCorrectedD = myRopeDamping * 2 * sqrt(myObjectMass * myRopeK);

			check_nan(myRopeCorrectedD);

			float ropeForceDamping = myRopeCorrectedD * myRopeStretchSpeed;
			float ropeForceStrech = myRopeK * myRopeStretchRelative;

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

			myRopeForceScalar = ropeForceStrech + ropeForceDamping;
			if (myRopeForceScalar < 0.0f)
			{
				myRopeForceScalar = 0;  // Rope can never apply negative forces, our damping could ;-)
				myDebugValue2 = 100.f;
			}

			// If we are still in the stretching phase, we want to make sure we do not shoot down the helicopter after an fps lag
			// Hence, we make sure that we will have at least another computation in the negative stretching phase

			// Debug: We just put critical damping on the rope. FPS Stuttering is just too much
			//if (myRopeStretchSpeed > 0.0f)
			{
				// force to stop the object at the current point in time
				float ropeStopForce = myRopeStretchSpeed * myObjectMass;

				// calcuate the required speed to reach the unstreched rope length in a single frame (= excactly a bit too much):
				float ropeTotalStretchMeters = myNewRopeLength - myRopeLengthNormal;
				float ropeEscapeSpeed = ropeTotalStretchMeters / myFrameTime;

				
				if (myMaxAccRopeFactor != 0.0f)
				{
					// divide this speed by the myMaxAccRopeFactor to be within limits
					ropeEscapeSpeed /= myMaxAccRopeFactor;

					//cacluate the max force to stop and accelerate in the other direction and still be within the streched rope
					float ropeMaxEscapeForce = ropeStopForce + (ropeEscapeSpeed * myObjectMass);

					myDebugValue1 = ropeMaxEscapeForce;

					//limit the force to this factor
					if ((myRopeForceScalar > ropeMaxEscapeForce))
					{
						myDebugStatement = false;
						myRopeForceScalar = ropeMaxEscapeForce;
					}
				}

			}

			// Did we rupture the rope?
			if (myRopeRuptureForce <= myRopeForceScalar)
			{
				myRopeRuptured = true;
				myRopeForceScalar = 0.0f;
				myCurrentRopeLength = myRopeLengthNormal;
			}
			else
			{
				// update the last-cycle value
				myCurrentRopeLength = myNewRopeLength;
			}
		}
		else
		{
			myCurrentRopeLength = myRopeLengthNormal;
		}

		// get the vector for the rope force
		myVectorForceRope = get_unit_vector(myVectorRope) * myRopeForceScalar;

		//get the air velocity vector: wind velocity is in opposite direction to our own velocity. If we move with the wind, this vector must be zero.
		myVectorAirVelocity = -1 * (myVectorHookVelocity - myVectorWindVelocity); 
		myAirSpeed = norm_2(myVectorAirVelocity);

		// Get the force of the air
		myAirResistance = myLfAirDensity * myObjectCWFront * myObjectCrossSection * myAirSpeed * myAirSpeed / 2.0; // If we are in the water, we would just need to correct for the density (+ split into air and water part).
		myVectorForceAir = get_unit_vector(myVectorAirVelocity) * myAirResistance;

		// Get the force of the gravity
		myVectorForceGravity(VERT_AXIS) = myLfGravitation * myObjectMass;


		// Sum up the forces
		myVectorForceTotal = myVectorForceRope + myVectorForceAir + myVectorForceGravity;

		// If we are on the ground and not pulled up, we need to compute the friction
		if (myTerrainHit == true)
		{
			if (myVectorForceTotal(VERT_AXIS) < 0)
			{
				// we did that before, right?
				myVectorHorizontalVelocity = myVectorHookVelocity;
				myVectorHorizontalVelocity(VERT_AXIS) = 0;

				// compute glide friction
				myVectorForceFriction =  get_unit_vector(myVectorHorizontalVelocity) * myVectorForceTotal(VERT_AXIS) * myObjectFrictionGlide;

				// Compute the speed, where the static friction would stop us within a frame. Multiply with 3 to be on the safe side for fps drop
				if (myObjectMass == 0.0f) return -1;
				myObjectSpeedStaticFriction = std::abs(3 * myFrameTime * myVectorForceTotal(VERT_AXIS) * myObjectFrictionStatic / myObjectMass); // adapt stop speed to frame rate

				// if we are below static friction speed
				if (norm_2(myVectorHorizontalVelocity) < myObjectSpeedStaticFriction)
				{
					// compute the static friction (again...)
					myVectorForceFriction = get_unit_vector(myVectorHorizontalVelocity) * myVectorForceTotal(VERT_AXIS) * myObjectFrictionStatic;
					myVectorForceTotal(VERT_AXIS) = 0; //Stop forces pulling below ground

					// If all forces are less than the static friction, we will not move
					if (norm_2(myVectorForceFriction) > norm_2(myVectorForceTotal))
					{
						// Stop movement
						myVectorHookVelocity = myVectorZeroVector;
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
		if (myObjectMass == 0.0f) return -1;
		myVectorAccTotal = myVectorForceTotal / myObjectMass;

		// calcualte the new velocity
		myVectorVelocityDelta = myVectorAccTotal * myFrameTime;
		myVectorHookVelocity += myVectorVelocityDelta;

		check_nan(myVectorHookVelocity);
		limit_max(myVectorHookVelocity, MAX_OBJ_SPEED);

		if (myPhysicsEnabled == true)
		{

			myVectorHookPosition += myVectorHookVelocity * myFrameTime;

			check_nan(myVectorHookPosition);

			// Compute the force in aircraft coordinates. This just turns the vector in the right direction.
			myVectorForceChopper = TurnWorldToAircraft(-1 * myVectorForceRope);

			// Momentum in Center of Gravity = rxF
			myVectorMomentumChopper = cross_product(myVectorWinchPosition, myVectorForceChopper);
		}

		check_nan(myVectorForceChopper);
		check_nan(myVectorMomentumChopper);

		// Apply Forces
		XPLMSetDataf(myDrForceX, myLfForceX + myVectorForceChopper(0));
		XPLMSetDataf(myDrForceY, myLfForceY + myVectorForceChopper(1));
		XPLMSetDataf(myDrForceZ, myLfForceZ + myVectorForceChopper(2));

		// The momentum axis as not as in my mechanics book, checked by experiment
		XPLMSetDataf(myDrMomentumX, myLfMomentumX - myVectorMomentumChopper(2));
		XPLMSetDataf(myDrMomentumY, myLfMomentumY + myVectorMomentumChopper(0));
		XPLMSetDataf(myDrMomentumZ, myLfMomentumZ - myVectorMomentumChopper(1));

		//////////////////////////////////////////////////////////////////////////////////////////////////
		//                   Graphics Pre-Calculation:
		myRopePoints.clear();
		vector<float>  myVectorFinalRope = myVectorHelicopterPosition - myVectorHookPosition;
		vector<float> vectorRopeUnit = get_unit_vector(myVectorFinalRope);

		// If the rope ruptured, the empty vector will disable 
		if (myRopeRuptured == false)
		{
			

			float ropeStepSize = norm_2(myVectorFinalRope) / ((float)HSL_ROPE_POINTS_MAX);

			if (ropeStepSize < 0.01f) ropeStepSize = 0.01f;

			for (float distance = 0; distance < (myNewRopeLength); distance += ropeStepSize)
			{
				vector<float> position = myVectorHookPosition + (distance * vectorRopeUnit);
				myRopePoints.push_back(position);
			}
		}
		
		// Calculate Object Offset
		float scalarObjectOffset = norm_2(myVectorCargoOffset);

		// If ruptured, keep offset and angle
		
		if (myRopeRuptured == false)
		{
			vector<float> normalPosition(3);

			normalPosition(0) = 0;
			normalPosition(1) = 1;
			normalPosition(2) = 0;

			vector<float> normalPositionSphere = XPlaneCartToSphere(normalPosition);
			vector<float> ropeUnitSphere = XPlaneCartToSphere(vectorRopeUnit);

			normalPositionSphere(1) = ropeUnitSphere(1) - normalPositionSphere(1);
			normalPositionSphere(2) = ropeUnitSphere(2) - normalPositionSphere(2);


			if (myTerrainHit == false)
			{
				// Using Pitch/Roll
				myVectorObjectDisplayAngle(0) = ropeUnitSphere(1) * 180.0f / M_PI; // Pitch
				myVectorObjectDisplayAngle(1) = ropeUnitSphere(2) * 180.0f / M_PI; // Roll
				myVectorObjectDisplayAngle(2) = 0; //heading

				myDebugValue1 = myVectorObjectDisplayAngle(0);
				myDebugValue2 = myVectorObjectDisplayAngle(1);
				myDebugValue3 = myVectorObjectDisplayAngle(2);

				
			}
			else
			{
				myVectorObjectDisplayAngle(0) = 0;
				myVectorObjectDisplayAngle(1) = 0;
				myVectorObjectDisplayAngle(2) = 0;
			}

			

			// Object Offset
			if (scalarObjectOffset > 0)
			{

				vector<float> myVectorCargoOffsetRotated = myVectorCargoOffset;

				float length = sqrt((myVectorCargoOffset(0) * myVectorCargoOffset(0)) + (myVectorCargoOffset(2) * myVectorCargoOffset(2)));
				float angle = atan2(myVectorCargoOffset(2), myVectorCargoOffset(0));

				angle += myVectorObjectDisplayAngle(2) * M_PI / 180.0f;

				myVectorCargoOffsetRotated(0) = length * cos(angle);
				myVectorCargoOffsetRotated(2) = length * sin(angle);

				vector<float> vectorObjectOffsetSphere = XPlaneCartToSphere(myVectorCargoOffsetRotated);
				if (myTerrainHit == false)
				{
					vectorObjectOffsetSphere(1) += ropeUnitSphere(1);
					vectorObjectOffsetSphere(2) += ropeUnitSphere(2);
				}

				myVectorObjectDisplayOffset = XPlaneSphereToCart(vectorObjectOffsetSphere);
			}
			else
			{
				myVectorObjectDisplayOffset = myVectorZeroVector;
			}

		}
		

		// Store to be able to detect movement between flight loop and draw callback
		myLastLocalX = myLdLocalX;
		myLastLocalY = myLdLocalY;
		myLastLocalZ = myLdLocalZ;
	}

	auto time_end = std::chrono::steady_clock::now();
	myProcessingTimeFlightLoop = std::chrono::duration_cast<std::chrono::microseconds>(time_end - time_start).count();

	return -1;
}

void HSL_PlugIn::DrawObjects()
{
	vector<float> vectorWinchWorld = AircraftToWorld(myVectorWinchPosition);

	XPLMDrawInfo_t		dr;
	dr.structSize = sizeof(dr);
	dr.x = vectorWinchWorld(0);
	dr.y = vectorWinchWorld(1);
	dr.z = vectorWinchWorld(2);
	dr.pitch = 0;
	dr.heading = 0;
	dr.roll = 0;

	XPLMInstanceSetPosition(myWinchInstanceRef, &dr, &myDummyAngle);
}

void HSL_PlugIn::ReadDataRefs()
{
	myLdLocalX = XPLMGetDatad(myDrLocalX);
	myLdLocalY = XPLMGetDatad(myDrLocalY);
	myLdLocalZ = XPLMGetDatad(myDrLocalZ);

	myLfLocalPhi = XPLMGetDataf(myDrLocalPhi) *  M_PI / 180.0f;
	myLfLocalPsi = XPLMGetDataf(myDrLocalPsi) * M_PI / 180.0f;
	myLfLocalTheta = XPLMGetDataf(myDrLocalTheta) * M_PI / 180.0f;

	myLiPause = XPLMGetDatai(myDrPause);
	myLfWindDirection = XPLMGetDataf(myDrWindDirection) * M_PI / 180.0f;
	myLfWindSpeed = XPLMGetDataf(myDrWindSpeed);


	myLfForceX = XPLMGetDataf(myDrForceX);
	myLfForceY = XPLMGetDataf(myDrForceY);
	myLfForceZ = XPLMGetDataf(myDrForceZ);

	myLfMomentumX = XPLMGetDataf(myDrMomentumX);
	myLfMomentumY = XPLMGetDataf(myDrMomentumY);
	myLfMomentumZ = XPLMGetDataf(myDrMomentumZ);


	myLfAirDensity = XPLMGetDataf(myDrAirDensity);
	myLfGravitation = -1 * XPLMGetDataf(myDrGravitation);


	
}

