#include "HSL_PlugIn.h"


vector<float> get_unit_vector(vector<float>& v_in)
{
	vector<float> ret = v_in;
	float length = norm_2(v_in);
	if (length != 0) ret = v_in / norm_2(v_in);
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
	for (unsigned i = 0; i < myVectorHelicopterPosition.size(); ++i) myVectorHelicopterPosition(i) = 0;
	for (unsigned i = 0; i < myVectorHookPosition.size(); ++i) myVectorHookPosition(i) = 0;
	for (unsigned i = 0; i < myVectorHookVelocity.size(); ++i) myVectorHookVelocity(i) = 0;
	for (unsigned i = 0; i < myVectorForceGravity.size(); ++i) myVectorForceGravity(i) = 0;
	for (unsigned i = 0; i < myVectorWindVelocity.size(); ++i) myVectorWindVelocity(i) = 0;
	for (unsigned i = 0; i < myVectorWinchPosition.size(); ++i) myVectorWinchPosition(i) = 0;

	for (unsigned i = 0; i < myVectorZeroVector.size(); ++i) myVectorZeroVector(i) = 0;

	myVectorObjectOffset = myVectorZeroVector;

	myVectorDefaultWinchPosition(0) = 0.0f;
	myVectorDefaultWinchPosition(1) = -1.6f;
	myVectorDefaultWinchPosition(2) = 0.0f;


	
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

	XPLMRegisterFlightLoopCallback(WrapFlightLoopCallback, 1, 0);

	// Menu;
	myPluginMenu = XPLMAppendMenuItem(XPLMFindPluginsMenu(), "Sling Line", 0, 1);
	myPluginMenuID = XPLMCreateMenu("Sling Line", XPLMFindPluginsMenu(), myPluginMenu, WrapMenuHandler, 0);
	myEnableSlingMenu = XPLMAppendMenuItem(myPluginMenuID, "Enable Sling Line", "ItemEnable", 1);
	XPLMAppendMenuItem(myPluginMenuID, "Toggle Control Window", "ItemWindow", 2);

	// Drawing
	XPLMRegisterDrawCallback(WrapDrawCallback, xplm_Phase_Airplanes, 0, NULL);

	int left, top, right, bot;
	XPLMGetScreenBoundsGlobal(&left, &top, &right, &bot);


	int width = 1000;
	int height = 550;
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

	myInitialized = true;


}

void HSL_PlugIn::PluginStop()
{
	if (myInitialized == false) return;

	XPLMDestroyMenu(myPluginMenuID);
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
			SlingReset();

		}
		if (inMessage == XPLM_MSG_AIRPORT_LOADED)
		{
			HSLDebugString("Airport loaded");
		}


		if (inMessage == XPLM_MSG_SCENERY_LOADED)
		{
			HSLDebugString("Scenery loaded");


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

	if (mySlingLineEnabled == true)
	{
		

		ReadDataRefs();

		vector<float> vectorWinchWorld = AdjustFrameMovement(myVectorHelicopterPosition);

		XPLMDrawInfo_t		dr;
		dr.structSize = sizeof(dr);
		dr.x = vectorWinchWorld(0);
		dr.y = vectorWinchWorld(1);
		dr.z = vectorWinchWorld(2);
		dr.pitch = 0;
		dr.heading = 0;
		dr.roll = 0;

		//XPLMInstanceSetPosition(myWinchInstanceRef, &dr, &myDummyAngle);

		XPLMDrawObjects(myWinchObjectRef, 1, &dr, 0, 0 );

		int index;

		for (index = 0; (index < myRopePoints.size()) && (index < HSL_ROPE_POINTS_MAX); index++)
		{
			vector<float> vectorRopePointOpenGL = AdjustFrameMovement(myRopePoints[index]);
			
			ropePositions[index].structSize = sizeof(dr);
			ropePositions[index].x = vectorRopePointOpenGL(0);
			ropePositions[index].y = vectorRopePointOpenGL(1);
			ropePositions[index].z = vectorRopePointOpenGL(2);
			ropePositions[index].pitch = 0;
			ropePositions[index].heading = 0;
			ropePositions[index].roll = 0;
		}

		if (index > 0)
		{
			XPLMDrawObjects(myRopeObjectRef, index-1, ropePositions, 0, 0);
		}


		vector<float> vectorCargoPointOpenGL = AdjustFrameMovement(myVectorHookPosition);

		XPLMDrawInfo_t		diCargo;
		diCargo.structSize = sizeof(diCargo);
		diCargo.x = vectorCargoPointOpenGL(0) + myVectorCargoOffset(0);
		diCargo.y = vectorCargoPointOpenGL(1) + myVectorCargoOffset(1);
		diCargo.z = vectorCargoPointOpenGL(2) + myVectorCargoOffset(2);
		diCargo.pitch = myVectorCargoAngle(0);
		diCargo.heading = myVectorCargoAngle(1);
		diCargo.roll = myVectorCargoAngle(2);


		XPLMDrawObjects(myCargoObjectRef, 1, &diCargo, 0, 0);
		

	}


	return 1;
}

void HSL_PlugIn::ConfigSave()
{
	boost::property_tree::ptree pt;

	//pt.put("HRM.gfp_file_path", m_gfp_path);

	boost::property_tree::ini_parser::write_ini(myConfigPath + "HSL.ini", pt);
}

void HSL_PlugIn::ConfigRead()
{
	boost::property_tree::ptree pt;
	try
	{
		boost::property_tree::ini_parser::read_ini(myConfigPath + "HSL.ini", pt);
	}
	catch (...)
	{
		HSLDebugString("Could not read config file");
		return;
	}

	//try { m_global_path_index = pt.get<int>("HRM.global_path_index"); }
	//catch (...) { HRMDebugString("Ini File: Entry not found."); }
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
	myVectorCargoAngle = myVectorZeroVector;
	myVectorCargoOffset = myVectorZeroVector;

	for (unsigned i = 0; i < myVectorZeroVector.size(); ++i) myVectorZeroVector(i) = 0;


	myRopeLengthNormal = myDefaultRopeLengthNormal;
	myRopeDamping = myDefaultRopeDamping;
	myRopeK = myDefaultRopeK;
	myObjectMass = myDefaultObjectMass;
	myObjectCrossSection = myDefaultObjectCrossSection;
	myObjectCWFront = myDefaultObjectCWFront;
	myObjectFrictionGlide = myDefaultObjectFrictionGlide;
	myObjectFrictionStatic = myDefaultObjectFrictionStatic;
	myObjectSpeedStaticFriction = myDefaultObjectSpeedStaticFriction;
	myObjectHeight = myDefaultObjectHeight;

	ReadDataRefs();

	myLastLocalX = myLdLocalX;
	myLastLocalY = myLdLocalY;
	myLastLocalZ = myLdLocalZ;

	myCurrentRopeLength = myRopeLengthNormal;

	myVectorHookPosition = AircraftToWorld(myVectorWinchPosition);
	myVectorHookPosition(VERT_AXIS) -= myRopeLengthNormal * 0.9;


	//////////////////////////////////////// DEBUG

	myVectorObjectOffset(0) = -0.1f;
	myVectorObjectOffset(1) = -0.7f;
	myVectorObjectOffset(2) =  0.2f;



	////////////////////////////////


	HSLDebugString("Reset: Unloading Instances");
	// Load Objects
	if (myWinchInstanceRef != NULL)
	{
		XPLMDestroyInstance(myWinchInstanceRef);
		myWinchInstanceRef = NULL;
	}

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

	if (myCargoObjectRef != NULL)
	{
		XPLMUnloadObject(myCargoObjectRef);
		myCargoObjectRef = NULL;
	}

	HSLDebugString("Reset: Winch Object Lookup Start");
	XPLMLookupObjects(myWinchPath.c_str(), 0, 0, load_cb, &myWinchObjectRef);
	XPLMLookupObjects(myRopePath.c_str(), 0, 0, load_cb, &myRopeObjectRef);
	XPLMLookupObjects(myCargoPath.c_str(), 0, 0, load_cb, &myCargoObjectRef);

	if (!myWinchObjectRef)
	{
		HSLDebugString("Reset: Winch Object: Nothing found");
	}
	else
	{

		HSLDebugString("Reset: Winch Object Lookup Finished");
		HSLDebugString("Creating Instance");

		const char* drefs[] = { NULL, NULL };
		myWinchInstanceRef = XPLMCreateInstance(myWinchObjectRef, drefs);

		myGroundProbe = XPLMCreateProbe(xplm_ProbeY);
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
	

	if ((mySlingLineEnabled == true) && (myLiPause == 0))
	{
		myFrameTime = elapsedMe;
		ReadDataRefs();


		vector<float> test1 = AircraftToWorld(myVectorWinchPosition);
		myVectorRotationTest = TurnWorldToAircraft(test1);

		//Move Object Relative to Helicopter Movement (we calculate in Aircraft coordinates to compensate movement between flight loop and drawing)
		//Movement is obviously in inverted direction 
		//myVectorHookPosition(0) += myLastLocalX - myLdLocalX;
		//myVectorHookPosition(1) += myLastLocalY - myLdLocalY;
		//myVectorHookPosition(2) += myLastLocalZ - myLdLocalZ;
		
		myVectorHelicopterPosition = AircraftToWorld(myVectorWinchPosition);

		myTerrainHit = false;

		
		//vectorHookWorld(VERT_AXIS) -= myObjectHeight;

		XPLMProbeInfo_t info;
		info.structSize = sizeof(info);
		XPLMProbeResult result = XPLMProbeTerrainXYZ(myGroundProbe, myVectorHookPosition(0), myVectorHookPosition(1), myVectorHookPosition(2), &info);

		myObjectTerrainLevel = info.locationY - myObjectHeight;

		if (myVectorHookPosition(VERT_AXIS) <= myObjectTerrainLevel)
		{
			myTerrainHit = true;
			myVectorHookPosition(VERT_AXIS) = myObjectTerrainLevel;
			if (myVectorHookVelocity(VERT_AXIS) < 0) myVectorHookVelocity(VERT_AXIS) = 0; //no velocity targeting below ground

		}

		
		myVectorRope = myVectorHelicopterPosition - myVectorHookPosition;

		myNewRopeLength = norm_2(myVectorRope);


		myRopeStretchRelative = (myNewRopeLength / myRopeLengthNormal) - 1;
		myRopeForceScalar = 0.0f;


		//f_seil_obj  = (k_seil * seil_dehnung_relative - d_seil * l_seil_delta) * seil_richtung_normal_vektor // wenn l_seil_delta negativ, kraft größer als reine dehnung -> wärme entwicklung


		// if we have a rope force
		if (myRopeStretchRelative > 0.0f)
		{

			myRopeLengthDelta = myNewRopeLength - myCurrentRopeLength;
			myRopeStretchSpeed = myRopeLengthDelta / myFrameTime;


			myRopeCorrectedD = myRopeDamping * 4 * sqrt(myObjectMass * myRopeK);


			myRopeForceScalar = (myRopeK * myRopeStretchRelative) + (myRopeCorrectedD * myRopeStretchSpeed);


			myCurrentRopeLength = myNewRopeLength;
		}
		else
		{
			myCurrentRopeLength = myRopeLengthNormal;
		}


		myVectorForceRope = get_unit_vector(myVectorRope) * myRopeForceScalar;
		myVectorAirVelocity = -1 * (myVectorHookVelocity - myVectorWindVelocity); //wind velocity is in opposite direction

		myAirSpeed = norm_2(myVectorAirVelocity);
		myAirResistance = 1.2f * myObjectCWFront * myObjectCrossSection * myAirSpeed * myAirSpeed / 2.0; // 1.2 is the air density (should we correct this by altitude - dataref?)


		myVectorForceAir = get_unit_vector(myVectorAirVelocity) * myAirResistance;
		myVectorForceGravity(VERT_AXIS) = myGravitation * myObjectMass;

		myVectorForceTotal = myVectorForceRope + myVectorForceAir + myVectorForceGravity;


		if (myTerrainHit == true)
		{
			if (myVectorForceTotal(VERT_AXIS) < 0)
			{




				myVectorHorizontalVelocity = myVectorHookVelocity;
				myVectorHorizontalVelocity(VERT_AXIS) = 0;

				myVectorForceFriction =  get_unit_vector(myVectorHorizontalVelocity) * myVectorForceTotal(VERT_AXIS) * myObjectFrictionGlide;

				myObjectSpeedStaticFriction = std::abs(3 * myFrameTime * myVectorForceTotal(VERT_AXIS) * myObjectFrictionStatic / myObjectMass); // adapt stop speed to frame rate



				if (norm_2(myVectorHorizontalVelocity) < myObjectSpeedStaticFriction)
				{
					myVectorForceFriction = get_unit_vector(myVectorHorizontalVelocity) * myVectorForceTotal(VERT_AXIS) * myObjectFrictionStatic;
					myVectorForceTotal(VERT_AXIS) = 0; //Stop forces pulling below ground

					if (norm_2(myVectorForceFriction) > norm_2(myVectorForceTotal))
					{
						// Stop movement
						myVectorHookVelocity = myVectorZeroVector;
						myVectorForceTotal = myVectorZeroVector;
						myVectorForceFriction = myVectorZeroVector;
					}
				}

				myVectorForceTotal += myVectorForceFriction;
				myVectorForceTotal(VERT_AXIS) = 0; //Stop forces pulling below ground
			}


		}

		myVectorAccTotal = myVectorForceTotal / myObjectMass;

		myVectorVelocityDelta = myVectorAccTotal * myFrameTime;
		myVectorHookVelocity += myVectorVelocityDelta;
		myVectorHookPosition += myVectorHookVelocity * myFrameTime;

		myVectorForceChopper = TurnWorldToAircraft(-1 * myVectorForceRope);
		myVectorMomentumChopper = cross_product(myVectorWinchPosition, myVectorForceChopper);


		XPLMSetDataf(myDrForceX, myLfForceX + myVectorForceChopper(0));
		XPLMSetDataf(myDrForceY, myLfForceY + myVectorForceChopper(1));
		XPLMSetDataf(myDrForceZ, myLfForceZ + myVectorForceChopper(2));

		XPLMSetDataf(myDrMomentumX, myLfMomentumX - myVectorMomentumChopper(2));
		XPLMSetDataf(myDrMomentumY, myLfMomentumY + myVectorMomentumChopper(0));
		XPLMSetDataf(myDrMomentumZ, myLfMomentumZ - myVectorMomentumChopper(1));

		//////////////////////////////////////////////////////////////////////////////////////////////////
		//                   Graphics Pre-Calculation:
		myRopePoints.clear();

		vector<float>  myVectorFinalRope = myVectorHelicopterPosition - myVectorHookPosition;

		vector<float> vectorRopeUnit = get_unit_vector(myVectorFinalRope);

		float ropeStepSize = norm_2(myVectorFinalRope) / ((float) HSL_ROPE_POINTS_MAX);

		if (ropeStepSize < 0.01f) ropeStepSize = 0.01f;

		for (float distance = 0; distance < (myNewRopeLength); distance += ropeStepSize)
		{
			vector<float> position = myVectorHookPosition + (distance * vectorRopeUnit);
			myRopePoints.push_back(position);
		}

		
		//vector<float> vectorObjectOffsetUnit = get_unit_vector(myVectorObjectOffset);
		
		
		float scalarObjectOffset = norm_2(myVectorObjectOffset);

		if (scalarObjectOffset > 0)
		{
			vector<float> vectorObjectOffsetSphere(3);


			vectorObjectOffsetSphere(0) = norm_2(myVectorObjectOffset);
			vectorObjectOffsetSphere(1) = acos(myVectorObjectOffset(2) / vectorObjectOffsetSphere(0));
			vectorObjectOffsetSphere(2) = atan2(myVectorObjectOffset(1), myVectorObjectOffset(0));

			vector<float> normalPosition(3);

			normalPosition(0) = 0;
			normalPosition(1) = 1;
			normalPosition(2) = 0;

			vector<float> normalPositionSphere(3);

			normalPositionSphere(0) = 1;
			normalPositionSphere(1) = acos(normalPosition(2));
			normalPositionSphere(2) = atan2(normalPosition(1), normalPosition(0));

			float angle_1_rope_unit = acos(vectorRopeUnit(2));
			float angle_2_rope_unit = atan2(vectorRopeUnit(1), vectorRopeUnit(0));

			normalPositionSphere(1) = angle_1_rope_unit - normalPositionSphere(1);
			normalPositionSphere(2) = angle_2_rope_unit - normalPositionSphere(2);

			/*if (myTerrainHit == false)
			{

				vectorObjectOffsetSphere(1) += normalPositionSphere(1);
				vectorObjectOffsetSphere(2) += normalPositionSphere(2);
			}*/

			myVectorCargoOffset(0) = vectorObjectOffsetSphere(0) * sin(vectorObjectOffsetSphere(1)) * cos(vectorObjectOffsetSphere(2));
			myVectorCargoOffset(1) = vectorObjectOffsetSphere(0) * sin(vectorObjectOffsetSphere(1)) * sin(vectorObjectOffsetSphere(2));
			myVectorCargoOffset(2) = vectorObjectOffsetSphere(0) * cos(vectorObjectOffsetSphere(1));
		}
		else
		{
			myVectorCargoOffset = myVectorZeroVector;
		}




		/*if (myTerrainHit == false)
		{
			myVectorCargoAngle(0) = -1 * ((acos(vectorRopeUnit(2)) * 180.0f / M_PI) - 90.0f);
			myVectorCargoAngle(1) = 0;
			myVectorCargoAngle(2) = -1 * ((atan2(vectorRopeUnit(1), vectorRopeUnit(0)) * 180.0f / M_PI) - 90.0f);
		}
		else
		{
			myVectorCargoAngle(0) = 0;
			myVectorCargoAngle(1) = 0;
			myVectorCargoAngle(2) = 0;
		}*/


		/*vectorObjectOffsetAngle(0) = asin(vectorObjectOffsetUnit(2));
		vectorObjectOffsetAngle(1) = 0;
		vectorObjectOffsetAngle(2) = asin(vectorObjectOffsetUnit(0));
		
		myVectorCargoAngle(0) = asin(vectorRopeUnit(2));
		myVectorCargoAngle(1) = 0;
		myVectorCargoAngle(2) = asin(vectorRopeUnit(0));

		vectorObjectOffsetAngle += myVectorCargoAngle;

		myVectorCargoOffset(2) = scalarObjectOffset * sin(vectorObjectOffsetAngle(0));
		myVectorCargoOffset(0) = scalarObjectOffset * sin(vectorObjectOffsetAngle(1));
		myVectorCargoOffset(1) = sqrt((scalarObjectOffset * scalarObjectOffset) - (myVectorCargoOffset(2) * myVectorCargoOffset(2)) - (myVectorCargoOffset(0) * myVectorCargoOffset(0)));


		myVectorCargoAngle(0) = myVectorCargoAngle(0) * 180.0f / M_PI;
		myVectorCargoAngle(1) = 0;
		myVectorCargoAngle(2) = myVectorCargoAngle(2) * 180.0f / M_PI;

		if ()*/



		//myVectorCargoOffset = -1 * vectorRopeUnit * myObjectHeight;


		//myVectorCargoOffset = vectorRopeUnit * myObjectHeight;
		


		//DrawObjects();



		myLastLocalX = myLdLocalX;
		myLastLocalY = myLdLocalY;
		myLastLocalZ = myLdLocalZ;
	}



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


	
}

