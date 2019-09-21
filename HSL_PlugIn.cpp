#include "HSL_PlugIn.h"





HSL_PlugIn::HSL_PlugIn() :
	myCargo(*this),
	myHook(*this)
{
	myCargo.myHeight = 0.9;
	myCargo.myMass = 75.0f;
	myCargo.myCrossSection = 1.1f; //m2
	myCargo.myCWFront = 0.9; //cube
	myCargo.myFrictionGlide = 0.35;
	myCargo.myFrictionStatic = 0.65;

	myHook.myHeight = 0.1;
	myHook.myMass = 5.0f;
	myHook.myCrossSection = 1.1f; //m2
	myHook.myCWFront = 0.9; //cube
	myHook.myFrictionGlide = 0.35;
	myHook.myFrictionStatic = 0.65;

	for (unsigned i = 0; i < myVectorZeroVector.size(); ++i) myVectorZeroVector(i) = 0;

	myVectorHelicopterPosition = myVectorZeroVector;
	myVectorHookPosition = myVectorZeroVector;
	myVectorWinchPosition = myVectorZeroVector;
	
	myVectorDefaultWinchPosition = myVectorZeroVector;

	myVectorRope = myVectorZeroVector;

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
	/*RegisterFloatDataref(myObjectHeight, "HSL/CurrentLoadHeight");
	RegisterFloatDataref(myObjectMass, "HSL/CurrentLoadMass");
	RegisterFloatDataref(myObjectCrossSection, "HSL/CurrentLoadCrossSection");
	RegisterFloatDataref(myObjectCWFront, "HSL/CurrentLoadCWFront");
	RegisterFloatDataref(myObjectFrictionGlide, "HSL/CurrentLoadFrictionGlide");
	RegisterFloatDataref(myObjectFrictionStatic, "HSL/CurrentLoadFrictionStatic");*/

	

	// Hook 
	RegisterFloatDataref(myHook.myHeight, "HSL/HookHeight");
	RegisterFloatDataref(myHook.myMass, "HSL/HookMass");
	RegisterFloatDataref(myHook.myCrossSection, "HSL/HookCrossSection");
	RegisterFloatDataref(myHook.myCWFront, "HSL/HookCWFront");
	RegisterFloatDataref(myHook.myFrictionGlide, "HSL/HookFrictionGlide");
	RegisterFloatDataref(myHook.myFrictionStatic, "HSL/HookFrictionStatic");

	// Cargo
	RegisterFloatDataref(myCargo.myHeight, "HSL/CargoHeight");
	RegisterFloatDataref(myCargo.myMass, "HSL/CargoMass");
	RegisterFloatDataref(myCargo.myCrossSection, "HSL/CargoCrossSection");
	RegisterFloatDataref(myCargo.myCWFront, "HSL/CargoCWFront");
	RegisterFloatDataref(myCargo.myFrictionGlide, "HSL/CargoFrictionGlide");
	RegisterFloatDataref(myCargo.myFrictionStatic, "HSL/CargoFrictionStatic");

	RegisterFloatDataref(myWinchSpeed, "HSL/WinchSpeed");
	
	
	RegisterVectorDataref(myVectorWinchPosition, "HSL/VectorWinchPosition");

	RegisterVectorDataref(myVectorHelicopterPosition, "HSL/Calculated/VectorHelicopterPosition");
	RegisterVectorDataref(myVectorHookPosition, "HSL/Calculated/VectorHookPosition");
	
	
	/*RegisterVectorDataref(myVectorHookVelocity, "HSL/Calculated/VectorHookVelocity");
	RegisterVectorDataref(myVectorForceGravity, "HSL/Calculated/VectorForceGravity");
	RegisterVectorDataref(myVectorWindVelocity, "HSL/Calculated/VectorWindVelocity");
	RegisterVectorDataref(myVectorWinchPosition, "HSL/Calculated/VectorWinchPosition");
	RegisterVectorDataref(myVectorCargoOffset, "HSL/Calculated/VectorCargoOffset");
	RegisterVectorDataref(myVectorZeroVector, "HSL/Calculated/VectorZeroVector");
	RegisterVectorDataref(myVectorObjectDisplayOffset, "HSL/Calculated/VectorObjectDisplayOffset");
	RegisterVectorDataref(myVectorObjectDisplayAngle, "HSL/Calculated/VectorObjectDisplayAngle");
	RegisterVectorDataref(myVectorRotationTest, "HSL/Calculated/VectorRotationTest");*/


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
	//RegisterFloatDataref(myObjectTerrainLevel, "HSL/ObjectTerrainLevel");
	RegisterFloatDataref(myNewRopeLength, "HSL/NewRopeLength");
	RegisterFloatDataref(myRopeStretchRelative, "HSL/RopeStretchRelative");
	RegisterFloatDataref(myRopeForceScalar, "HSL/RopeForceScalar");
	RegisterFloatDataref(myRopeLengthDelta, "HSL/RopeLengthDelta");
	RegisterFloatDataref(myRopeStretchSpeed, "HSL/RopeStretchSpeed");
	RegisterFloatDataref(myRopeCorrectedD, "HSL/RopeCorrectedD");
	//RegisterFloatDataref(myAirSpeed, "HSL/AirSpeed");
	//RegisterFloatDataref(myAirResistance, "HSL/AirResistance");

	RegisterIntDataref(myRopeRuptured, "HSL/RopeRuptured");
	//RegisterIntDataref(myCargoConnected, "HSL/CargoConnected");


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
		if (myCargo.myDrawingEnabled == true)
		{
			vectorCargoPointOpenGL -= myCargo.myVectorDisplayOffset;

			DrawInstanceCreate(myCargoInstanceRef, myCargoObjectRef);
			DrawInstanceSetPosition(myCargoInstanceRef, myCargoObjectRef, vectorCargoPointOpenGL, myCargo.myVectorDisplayAngle, true);
			

		}
		else
		{
			DrawInstanceDestroy(myCargoInstanceRef);
		}


		if (myHook.myDrawingEnabled == true)
		{
			DrawInstanceCreate(myHookInstanceRef, myHookObjectRef);
			DrawInstanceSetPosition(myHookInstanceRef, myHookObjectRef, vectorCargoPointOpenGL, false);
		}
		else
		{
			DrawInstanceDestroy(myHookInstanceRef);
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
			pt.put("Hook.height", myHook.myHeight);
			pt.put("Hook.mass", myHook.myMass);
			pt.put("Hook.cross_section_front", myHook.myCrossSection);
			pt.put("Hook.cw_front", myHook.myCWFront);
			pt.put("Hook.friction_glide", myHook.myFrictionGlide);
			pt.put("Hook.friction_static", myHook.myFrictionStatic);

			pt.put("Cargo.library_object", myCargoPath);
			pt.put("Cargo.height", myCargo.myHeight);
			pt.put("Cargo.mass", myCargo.myMass);
			pt.put("Cargo.cross_section_front", myCargo.myCrossSection);
			pt.put("Cargo.cw_front", myCargo.myCWFront);
			pt.put("Cargo.friction_glide", myCargo.myFrictionGlide);
			pt.put("Cargo.friction_static", myCargo.myFrictionStatic);
			ConfigWriteVector(pt, myCargo.myVectorCargoOffset, "Cargo.offset");
			
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
		ConfigReadFloat(pt, "Hook.height", myHook.myHeight);
		ConfigReadFloat(pt, "Hook.mass", myHook.myMass);
		ConfigReadFloat(pt, "Hook.cross_section_front", myHook.myCrossSection);
		ConfigReadFloat(pt, "Hook.cw_front", myHook.myCWFront);
		ConfigReadFloat(pt, "Hook.friction_glide", myHook.myFrictionGlide);
		ConfigReadFloat(pt, "Hook.friction_static", myHook.myFrictionStatic);

		ConfigReadString(pt, "Cargo.library_object", myCargoPath);
		ConfigReadFloat(pt, "Cargo.height", myCargo.myHeight);
		ConfigReadFloat(pt, "Cargo.mass", myCargo.myMass);
		ConfigReadFloat(pt, "Cargo.cross_section_front", myCargo.myCrossSection);
		ConfigReadFloat(pt, "Cargo.cw_front", myCargo.myCWFront);
		ConfigReadFloat(pt, "Cargo.friction_glide", myCargo.myFrictionGlide);
		ConfigReadFloat(pt, "Cargo.friction_static", myCargo.myFrictionStatic);
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
	
	myVectorWinchPosition = myVectorDefaultWinchPosition;
	myCargo.myVectorDisplayAngle = myVectorZeroVector;
	myHook.myVectorDisplayAngle = myVectorZeroVector;
	
	myCargo.myVectorDisplayOffset = myVectorZeroVector;
	myHook.myVectorDisplayOffset = myVectorZeroVector;

	for (unsigned i = 0; i < myVectorZeroVector.size(); ++i) myVectorZeroVector(i) = 0;

	// reset rope length

	myRopeLengthNormal = myRopeLengthStart;

	// load hook
	myCargo.myRopeConnected = true;
	myCargo.myFollowOnly = true;
	myCargo.myDrawingEnabled = false;

	myHook.myRopeConnected = true;
	myHook.myFollowOnly = false;
	myHook.myDrawingEnabled = true;

	myRopeRuptured = false;

	ReadDataRefs();

	myLastLocalX = myLdLocalX;
	myLastLocalY = myLdLocalY;
	myLastLocalZ = myLdLocalZ;

	myCurrentRopeLength = myRopeLengthNormal;

	myVectorHookPosition = AircraftToWorld(myVectorWinchPosition);
	myVectorHookPosition(VERT_AXIS) -= myRopeLengthNormal * 0.5;
	myWinchDirection = HSL::Stop;

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
	myCargo.myRopeConnected = true;
	myCargo.myFollowOnly = false;
	myCargo.myDrawingEnabled = true;

	myHook.myRopeConnected = true;
	myHook.myFollowOnly = true;
	myHook.myDrawingEnabled = false;
}

void HSL_PlugIn::SlingRelease()
{
	myCargo.myRopeConnected = true;
	myCargo.myFollowOnly = true;
	myCargo.myDrawingEnabled = false;

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

