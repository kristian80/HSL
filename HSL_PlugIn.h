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

#pragma once


#include "HSL.h"
#include "HSLImguiWidget.h"
#include "CargoObject.h"
#include "DropObject.h"
#include "TSQueue.h"
#include "DropThread.h"
#include "FireObject.h"
#include "PhysicsThread.h"


/*
ToDo Thread:

Make Dataref Access function thread safe -> Stop Computation, write value, start computation again
Make Reset properly for out functions! Stop Computation on Reset

*/

struct CargoDataShared
{
	bool myThreadRunFlag = true; // Only to terminate computation
	bool myComputationRunFlag = false;
	bool myHighPerformace = false;

	double myFrameTime = 0;
	
	double myFrameTimeMax = 0;
	bool myNewFrame = false;

	

	vector<double> myVectorRope = vector<double>(3);

	//vector<double> myVectorHelicopterPositionFlightLoop = vector<double>(3);
	vector<double> myVectorHelicopterVelocity = vector<double>(3);
	vector<double> myVectorHelicopterAcceleration = vector<double>(3);

	vector<double> myVectorHelicopterPosition = vector<double>(3);
	vector<double> myVectorHookPosition = vector<double>(3);
	vector<double> myVectorWinchPosition = vector<double>(3);
	vector<double> myVectorWindVelocity = vector<double>(3);

	
	

	vector<double> myVectorDebug = vector<double>(3);

	std::queue<ForceData> myHelicopterForceQueue;

	bool mySlingLineEnabled = false;
	bool myPhysicsEnabled = true;

	double myRopeLengthStart = 2.0;
	double myRopeLengthNormal = 2.2;
	double myRopeDamping = 0.2;//0.03;
	double myRopeK = 55000.0;
	double myRopeRuptureForce = 100000.0; //10t max
	bool   myRopeRuptured = false;
	double myMaxAccRopeFactor = 2.0;

	double myRopeOperatorDampingLength = 1.5;
	double myRopeOperatorDampingForce = 200.0;

	double myRopeArtificialDampingForce = 5.0;

	double myCargoSetLatitutde = 0.0;
	double myCargoSetLongitude = 0.0;
	double myCargoSetHeading = 0.0;
	bool   myCargoSetPosition = false;
	double myWinchSpeed = 0.5;
	double myCurrentRopeLength = 2.2;
	double myBambiBucketWaterFlow = 250.0;

	double myNewRopeLength = 0;
	double myRopeStretchRelative = 0;
	double myRopeForceScalar = 0;
	double myRopeLengthDelta = 0;
	double myRopeStretchSpeed = 0;
	double myRopeCorrectedD = 0;

	double myDebugValue1 = 0.0;
	double myDebugValue2 = 0.0;
	double myDebugValue3 = 0.0;
	double myDebugValue4 = 0.0;
	bool myDebugStatement = true;


	double myLdLocalX = 0;
	double myLdLocalY = 0;
	double myLdLocalZ = 0;

	double myLfLocalXv = 0;
	double myLfLocalYv = 0;
	double myLfLocalZv = 0;

	double myLfLocalXa = 0;
	double myLfLocalYa = 0;
	double myLfLocalZa = 0;

	double myLfGravitation = 0;
	double myLfAirDensity = 0;

	double myLfLocalPhi = 0;
	double myLfLocalPsi = 0;
	double myLfLocalTheta = 0;

	int   myLiPause = 0;
	int   myIsInReplay = 0;
	double myLfTimeActual = 1.0;
	double myLfWindDirection = 0;
	double myLfWindSpeed = 0;

};

struct ReplayData
{
	float mySimTime = 0;

	double myRopeLengthNormal = 0;
	vector<double> myVectorLocalPosition = vector<double>(3);
	vector<double> myVectorWinchPosition = vector<double>(3);
	vector<double> myVectorHookPosition = vector<double>(3);
	vector<double> myVectorCargoPosition = vector<double>(3);
	bool myRopeRuptured = false;
	bool myCargoDrawingEnabled = false;
	bool myHookDrawingEnabled = false;

	vector<double> myVectorCargoDisplayAngle = vector<double>(3);
	vector<double> myVectorHookDisplayAngle = vector<double>(3);

	vector<double> myVectorCargoDisplayOffset = vector<double>(3);
	vector<double> myVectorHookDisplayOffset = vector<double>(3);

	vector<double> myVectorCargoHelicopterPositionDeviation = vector<double>(3);
	vector<double> myVectorHookHelicopterPositionDeviation = vector<double>(3);
	

	RainDropDrawData* myRaindrops = NULL;

	//ToDo: Add Cargo Draw, Hook draw(?)
};

class HSL_PlugIn
{
	static bool HIGH_PERFORMANCE;
public:
	std::string myDS = "";
	std::string mySystemPath = "";
	std::string myConfigPath = "";
	std::string myAircraftPath = "";
	std::string myAircraftIniFile = "HSLAircraft.ini";
	std::string myConfigIniFile = "HSL.ini";

	std::vector<std::string> myProfileNames;
	std::vector<std::string> myProfilePaths;
	std::list<int> myProfileNumbers;

	std::string mySelectedProfileName = "";
	std::string mySelectedProfilePath = "";
	std::string myNewProfileName = "";
	int mySelectedProfileIndex = 0;
	

	std::shared_ptr<HSLImguiWidget> imguiPtr;
	
	DropThread myDropThreadObject;
	std::thread *myDropThreadPtr = NULL;

	PhysicsThread myPhysicsThreadObject;
	std::thread* myPhysicsThreadPtr = NULL;

	std::list<FireObject *> myFires;

	bool myInitialized = false;
	int myPluginEnabled = 0;
	int myAircraftLoaded = 0;
	bool mySimpleMode = true;
	bool myShowWinch = true;
	bool myUpdateObjectError = false;
	bool myObjectHasAnimation = false;

	XPLMPluginID myDataRefEditorPluginID = XPLM_NO_PLUGIN_ID;
	std::vector<XPLMDataRef> myRegisteredDatarefs;

	CargoDataShared myCargoDataShared;
	
	ReplayData* myReplayDataPtr = NULL;
	int myReplayDataSize = 0;
	



	double myDataRate = -1;
	int myCompuationsPerFlightLoop = 0;
	int myComputationFrequency = 0;
	bool mySlingLineEnabled = false;

	int myPluginMenu = 0;
	int myEnableSlingMenu = 0;
	XPLMMenuID myPluginMenuID = 0;

	XPLMCommandRef myCmdRefWinchUp = NULL;
	XPLMCommandRef myCmdRefWinchDown = NULL;
	XPLMCommandRef myCmdRefWinchStop = NULL;

	XPLMCommandRef myCmdRefEnable = NULL;
	XPLMCommandRef myCmdRefDisable = NULL;
	XPLMCommandRef myCmdRefReset = NULL;
	XPLMCommandRef myCmdRefConnectLoad = NULL;
	XPLMCommandRef myCmdRefReleaseLoad = NULL;
	XPLMCommandRef myCmdRefToggleControlWindow = NULL;
	XPLMCommandRef myCmdRefUpdateObjects = NULL;
	XPLMCommandRef myCmdRefPlaceCargoGround = NULL;
	XPLMCommandRef myCmdRefPlaceCoordinates = NULL;
	XPLMCommandRef myCmdRefFirePlaceGround = NULL;
	XPLMCommandRef myCmdRefFirePlaceCoordinates = NULL;
	XPLMCommandRef myCmdRefBambiBucketRelease = NULL;


	std::string myWinchPath = "./Resources/plugins/HSL/objects/sphere_white_1.obj";
	std::string myRopePath = "./Resources/plugins/HSL/objects/rope.obj";
	std::string myHookPath = "./Resources/plugins/HSL/objects/hook_hawk.obj";
	std::string myRaindropPath = "./Resources/plugins/HSL/objects/raindrop.obj";
	std::string myCargoPath = "./Resources/plugins/HSL/objects/cube.obj";

	std::string myRopePath_00_1 = "./Resources/plugins/HSL/objects/rope_cyl_0_1.obj";
	std::string myRopePath_01_0 = "./Resources/plugins/HSL/objects/rope_cyl_1_0.obj";
	std::string myRopePath_10_0 = "./Resources/plugins/HSL/objects/rope_cyl_10_0.obj";

	XPLMObjectRef myRopeObj_00_1 = NULL;
	XPLMObjectRef myRopeObj_01_0 = NULL;
	XPLMObjectRef myRopeObj_10_0 = NULL;

	XPLMInstanceRef myRopeInstances_00_1[HSL_ROPE_SEGMENTS];
	XPLMInstanceRef myRopeInstances_01_0[HSL_ROPE_SEGMENTS];
	XPLMInstanceRef myRopeInstances_10_0[HSL_ROPE_SEGMENTS];
	

	XPLMObjectRef myWinchObjectRef = NULL;
	XPLMObjectRef myRopeObjectRef = NULL;
	XPLMObjectRef myHookObjectRef = NULL;
	XPLMObjectRef myRaindropObjectRef = NULL;
	XPLMObjectRef myCargoObjectRef = NULL;
	
	XPLMInstanceRef myWinchInstanceRef = NULL;
	XPLMInstanceRef myHookInstanceRef = NULL;
	XPLMInstanceRef myCargoInstanceRef = NULL;
	
	XPLMProbeRef myGroundProbe = NULL;
	XPLMFlightLoopID myFlightLoopID = NULL;
	
	float myDummyAngle = 0;
	double myRopeDrawLength = 0;
	bool myRopeDrawSphere = false;



	vector<double> myRopePoints[HSL_ROPE_POINTS_MAX];
	XPLMInstanceRef myRopeInstances[HSL_ROPE_POINTS_MAX];
	XPLMInstanceRef myDebugInstances[HSL_ROPE_POINTS_MAX];
	HSL::WinchDirection myWinchDirection = HSL::Stop;

	vector<double> myVectorHelicopterForceApplied = vector<double>(3);
	vector<double> myVectorHelicopterMomentumApplied = vector<double>(3);

	/////////////////////////////////////////////////////////
	// Fire Variables

	//std::string myFireObjectPath = "lib/airport/Common_Elements/Miscellaneous/Tree.obj";
	std::string myFireAircraftPath = "Fire_Aircraft\\fire_aircraft.acf";
	std::string myNoFireAircraftPath = "NoFire_Aircraft\\nofire_aircraft.acf";
	XPLMObjectRef myFireObjectRef = NULL;

	vector<double> myVectorFireEmitterOffset = vector<double>(3);
	vector<double> myVectorBambiBucketReleasePosition = vector<double>(3);
	vector<double> myVectorEndOfRopePosition = vector<double>(3);


	double myFireWaterRadius = 3.0;
	double myFireLiftNom = 77.0;
	double myFireLiftDenom = 5.7;
	double myFireDistDemon = 20.0;

	double myFireStrengthStart = 1.0;
	double myFireStrengthMax = 1000.0;
	double myFireStrengthIncrease = 0;


	double myFireSetLatitutde = 0.0;
	double myFireSetLongitude = 0.0;
	double myFireSetElevation = 0.0;

	double myFireCount = 0;
	double myFireStrength[MAX_FIRES];

	bool myFireCreateFailed = 0;
	bool myUpdateFirePositions = 0;
	bool myRemoveFires = 0;
	/////////////////////////////////////////////////////////
	// Sling Variables

	CargoObject myCargo;
	CargoObject myHook;

	vector<double> myVectorZeroVector = vector<double>(3);

	

	int myProcessingTimeFlightLoop = 0;
	int myProcessingTimeDrawRoutine = 0;
	int myReplaySaveCounter = 0;

	// Rope 
	


	double myBambiBucketWaterPerDrop = 0;

	

	


	double myRainSpeed = 3.5;
	int myRainDirections = 10;
	int myRainVariance = 3;
	double myRainReleasePeriod = 0.1;
	double myRainReleaseTime = 0.0;

	int myRainDropNumber = 0;
	int myRainDropOverflow = 0;

	RainDropDrawData *myRaindrops = NULL;

	
	
	




	vector<double> myVectorDefaultWinchPosition = vector<double>(3);

	

	

	


	///////////////////////////////////////////////////
	// DataRefs X-Plane


	XPLMDataRef myDrSimTime;
	XPLMDataRef myDrIsInReplay;
	XPLMDataRef myDrLocalX;
	XPLMDataRef myDrLocalY;
	XPLMDataRef myDrLocalZ;

	XPLMDataRef myDrLocalXv;
	XPLMDataRef myDrLocalYv;
	XPLMDataRef myDrLocalZv;

	XPLMDataRef myDrLocalXa;
	XPLMDataRef myDrLocalYa;
	XPLMDataRef myDrLocalZa;

	XPLMDataRef myDrLocalPhi;
	XPLMDataRef myDrLocalPsi;
	XPLMDataRef myDrLocalTheta;

	XPLMDataRef myDrPause;
	XPLMDataRef myDrTimeActual;
	XPLMDataRef myDrWindDirection;
	XPLMDataRef myDrWindSpeed;

	XPLMDataRef myDrForceX;
	XPLMDataRef myDrForceY;
	XPLMDataRef myDrForceZ;

	XPLMDataRef myDrWindX;
	XPLMDataRef myDrWindY;
	XPLMDataRef myDrWindZ;

	XPLMDataRef myDrMomentumX;
	XPLMDataRef myDrMomentumY;
	XPLMDataRef myDrMomentumZ;

	XPLMDataRef myDrAirDensity;
	XPLMDataRef myDrGravitation;

	///////////////////////////////////////////////////
	// DataRefs X-Plane Local Copies

	float mySimTime = 0;
	int myIsInReplay = 0;

	double myLastLocalX = 0;
	double myLastLocalY = 0;
	double myLastLocalZ = 0;



	double myLfForceX = 0;
	double myLfForceY = 0;
	double myLfForceZ = 0;

	double myLfWindX = 0;
	double myLfWindY = 0;
	double myLfWindZ = 0;

	double myLfMomentumX = 0;
	double myLfMomentumY = 0;
	double myLfMomentumZ = 0;



	///////////////////////////////////////////////////
	// DataRef Shared Data


	///////////////////////////////////////////////////
	// Methods

	HSL_PlugIn();
	~HSL_PlugIn();

	void PluginStart();
	void PluginStop();
	void PluginEnable();
	void PluginDisable();

	

	void PluginReceiveMessage(XPLMPluginID inFromWho, int inMessage, void* inParam);
	void PluginMenuHandler(void* in_menu_ref, void* inItemRef);

	void PluginKeyCallback(XPLMWindowID inWindowID, char inKey, XPLMKeyFlags inFlags, char inVirtualKey, void* inRefcon, int losingFocus);
	int PluginMouseClickCallback(XPLMWindowID inWindowID, int x, int y, XPLMMouseStatus inMouse, void* inRefcon);

	int DrawCallback(XPLMDrawingPhase inPhase, int inIsBefore, void* inRefcon);

	void ConfigSave(std::string inIniFile);
	void ConfigRead(std::string inIniFile);

	bool SearchForObjectAnimation(std::string filenameIn);
	void ReadProfiles();

	void AircraftConfigSave();
	void AircraftConfigRead();
	void ConfigWriteVector(boost::property_tree::ptree &pt, vector<double> &vectorIn, std::string nameIn);
	void ConfigReadVector(boost::property_tree::ptree& pt, vector<double> &vectorOut, std::string nameIn);
	void ConfigReaddouble(boost::property_tree::ptree& pt, std::string nameIn, double& doubleOut);
	void ConfigReadInt(boost::property_tree::ptree& pt, std::string nameIn, int& intOut);
	void ConfigReadString(boost::property_tree::ptree& pt, std::string nameIn, std::string& stringOut);
	void ConfigReadBool(boost::property_tree::ptree& pt, std::string nameIn, bool& boolOut);

	void SlingEnable();
	void SlingDisable();
	void SlingReset();
	void SlingRepairRope();
	void SlingConnect();
	void SlingRelease();
	void UpdateParameters();
	void UpdateObjects();
	void CargoPlaceOnGround();
	void CargoPlaceCoordinates();
	void FirePlaceOnGround();
	void FirePlaceCoordinates();
	void BambiBucketRelease();
	void FirePlaceAtCoordinates(vector<double> * pinVectorFireObjectPosition);

	int WinchUpCallback(XPLMCommandRef cmd, XPLMCommandPhase phase, void* refcon);
	int WinchDownCallback(XPLMCommandRef cmd, XPLMCommandPhase phase, void* refcon);
	int WinchStopCallback(XPLMCommandRef cmd, XPLMCommandPhase phase, void* refcon);
	int EnableCallback(XPLMCommandRef cmd, XPLMCommandPhase phase, void* refcon);
	int DisableCallback(XPLMCommandRef cmd, XPLMCommandPhase phase, void* refcon);
	int ResetCallback(XPLMCommandRef cmd, XPLMCommandPhase phase, void* refcon);
	int ConnectLoadCallback(XPLMCommandRef cmd, XPLMCommandPhase phase, void* refcon);
	int ReleaseLoadCallback(XPLMCommandRef cmd, XPLMCommandPhase phase, void* refcon);
	int LoadGroundCallback(XPLMCommandRef cmd, XPLMCommandPhase phase, void* refcon);
	int LoadCoordinatesCallback(XPLMCommandRef cmd, XPLMCommandPhase phase, void* refcon);
	int ToggleControlWindowCallback(XPLMCommandRef cmd, XPLMCommandPhase phase, void* refcon);
	int UpdateObjectsCallback(XPLMCommandRef cmd, XPLMCommandPhase phase, void* refcon);

	//void RegisterDoubleDataref(double &valueIn, std::string nameIn);
	void RegisterDoubleArrayDataref(double *valueIn, std::string nameIn, int sizeIn);
	void RegisterDoubleDataref(double& valueIn, std::string nameIn);
	void RegisterVectorDataref(vector<double> & vectorIn, std::string nameIn);

	void RegisterIntDataref(bool& valueIn, std::string nameIn);
	void RegisterStringDataref(std::string& valueIn, std::string nameIn);


	static vector<double> TurnWorldToAircraft(vector<double> coordsAircraft, CargoDataShared& cargoDataSharedIn);
	vector<double> AircraftToWorld(vector<double> coordsAircraft);
	vector<double> AdjustFrameMovement(vector<double> coordsAircraft);
	vector<double> AdjustFrameMovementReplay(vector<double> coordsAircraft, vector<double> coordsOldAircraft);


	float PluginFlightLoopCallback(double elapsedMe, double elapsedSim, int counter, void* refcon);
	void DrawObjects();
	void ReadDataRefs();

	
};

