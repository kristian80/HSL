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

	float myFrameTime = 0;
	
	float myFrameTimeMax = 0;
	bool myNewFrame = false;

	

	vector<float> myVectorRope = vector<float>(3);

	//vector<float> myVectorHelicopterPositionFlightLoop = vector<float>(3);
	vector<float> myVectorHelicopterVelocity = vector<float>(3);
	vector<float> myVectorHelicopterAcceleration = vector<float>(3);

	vector<float> myVectorHelicopterPosition = vector<float>(3);
	vector<float> myVectorHookPosition = vector<float>(3);
	vector<float> myVectorWinchPosition = vector<float>(3);
	vector<float> myVectorWindVelocity = vector<float>(3);

	std::queue<ForceData> myHelicopterForceQueue;

	bool mySlingLineEnabled = false;
	bool myPhysicsEnabled = true;

	float myRopeLengthStart = 2.0f;
	float myRopeLengthNormal = 2.2f;
	float myRopeDamping = 0.03f;
	float myRopeK = 55000.0f;
	float myRopeRuptureForce = 100000.0f; //10t max
	bool  myRopeRuptured = false;
	float myMaxAccRopeFactor = 2.0f;

	double myCargoSetLatitutde = 0.0;
	double myCargoSetLongitude = 0.0;
	float myCargoSetHeading = 0.0;
	bool myCargoSetPosition = false;
	float myWinchSpeed = 0.5f;
	float myCurrentRopeLength = 2.2f;
	float myBambiBucketWaterFlow = 250.0f;

	float myNewRopeLength = 0;
	float myRopeStretchRelative = 0;
	float myRopeForceScalar = 0;
	float myRopeLengthDelta = 0;
	float myRopeStretchSpeed = 0;
	float myRopeCorrectedD = 0;

	float myDebugValue1 = 0.0f;
	float myDebugValue2 = 0.0f;
	float myDebugValue3 = 0.0f;
	float myDebugValue4 = 0.0f;
	bool myDebugStatement = true;


	double myLdLocalX = 0;
	double myLdLocalY = 0;
	double myLdLocalZ = 0;

	float myLfLocalXv = 0;
	float myLfLocalYv = 0;
	float myLfLocalZv = 0;

	float myLfLocalXa = 0;
	float myLfLocalYa = 0;
	float myLfLocalZa = 0;

	float myLfGravitation = 0;
	float myLfAirDensity = 0;

	float myLfLocalPhi = 0;
	float myLfLocalPsi = 0;
	float myLfLocalTheta = 0;

	int   myLiPause = 0;
	float myLfTimeActual = 1.0f;
	float myLfWindDirection = 0;
	float myLfWindSpeed = 0;

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

	XPLMPluginID myDataRefEditorPluginID = XPLM_NO_PLUGIN_ID;
	std::vector<XPLMDataRef> myRegisteredDatarefs;

	CargoDataShared myCargoDataShared;
	
	



	float myDataRate = -1;
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


	std::string myWinchPath = "./Resources/plugins/HSL/sphere_white_1.obj";
	std::string myRopePath = "./Resources/plugins/HSL/rope2.obj";
	std::string myHookPath = "./Resources/plugins/HSL/hook_hawk.obj";
	std::string myRaindropPath = "./Resources/plugins/HSL/raindrop4.obj";
	std::string myCargoPath = "./oggetti elitrasportabili/Swisscreations_bambi_bucket_8ft.obj";
	

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

	vector<float> myRopePoints[HSL_ROPE_POINTS_MAX];
	XPLMInstanceRef myRopeInstances[HSL_ROPE_POINTS_MAX];
	HSL::WinchDirection myWinchDirection = HSL::Stop;

	vector<float> myVectorHelicopterForceApplied = vector<float>(3);
	vector<float> myVectorHelicopterMomentumApplied = vector<float>(3);

	/////////////////////////////////////////////////////////
	// Fire Variables

	//std::string myFireObjectPath = "lib/airport/Common_Elements/Miscellaneous/Tree.obj";
	std::string myFireAircraftPath = "Fire_Aircraft\\fire_aircraft.acf";
	std::string myNoFireAircraftPath = "NoFire_Aircraft\\nofire_aircraft.acf";
	XPLMObjectRef myFireObjectRef = NULL;

	vector<float> myVectorFireEmitterOffset = vector<float>(3);

	float myFireWaterRadius = 3.0f;
	float myFireLiftNom = 77.0f;
	float myFireLiftDenom = 5.7f;
	float myFireDistDemon = 20.0f;

	float myFireStrengthStart = 1.0f;
	float myFireStrengthMax = 1000.0f;
	float myFireStrengthIncrease = 0;


	double myFireSetLatitutde = 0.0;
	double myFireSetLongitude = 0.0;
	float myFireSetElevation = 0.0;

	float myFireCount = 0;
	float myFireStrength[MAX_FIRES];

	bool myFireCreateFailed = 0;
	bool myUpdateFirePositions = 0;
	bool myRemoveFires = 0;
	/////////////////////////////////////////////////////////
	// Sling Variables

	CargoObject myCargo;
	CargoObject myHook;

	vector<float> myVectorZeroVector = vector<float>(3);

	

	int myProcessingTimeFlightLoop = 0;
	int myProcessingTimeDrawRoutine = 0;

	// Rope 
	


	float myBambiBucketWaterPerDrop = 0;

	

	


	float myRainSpeed = 3.5f;
	int myRainDirections = 10;
	int myRainVariance = 3;
	float myRainReleasePeriod = 0.1f;
	float myRainReleaseTime = 0.0f;

	int myRainDropNumber = 0;
	int myRainDropOverflow = 0;

	RainDropDrawData *myRaindrops = NULL;

	
	
	




	vector<float> myVectorDefaultWinchPosition = vector<float>(3);

	

	

	


	///////////////////////////////////////////////////
	// DataRefs X-Plane

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



	double myLastLocalX = 0;
	double myLastLocalY = 0;
	double myLastLocalZ = 0;



	float myLfForceX = 0;
	float myLfForceY = 0;
	float myLfForceZ = 0;

	float myLfWindX = 0;
	float myLfWindY = 0;
	float myLfWindZ = 0;

	float myLfMomentumX = 0;
	float myLfMomentumY = 0;
	float myLfMomentumZ = 0;



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

	void ConfigSave();
	void ConfigRead();

	void AircraftConfigSave();
	void AircraftConfigRead();
	void ConfigWriteVector(boost::property_tree::ptree &pt, vector<float> &vectorIn, std::string nameIn);
	void ConfigReadVector(boost::property_tree::ptree& pt, vector<float> &vectorOut, std::string nameIn);
	void ConfigReadFloat(boost::property_tree::ptree& pt, std::string nameIn, float& floatOut);
	void ConfigReadInt(boost::property_tree::ptree& pt, std::string nameIn, int& intOut);
	void ConfigReadString(boost::property_tree::ptree& pt, std::string nameIn, std::string& stringOut);

	void SlingEnable();
	void SlingDisable();
	void SlingReset();
	void SlingConnect();
	void SlingRelease();
	void UpdateParameters();
	void UpdateObjects();
	void CargoPlaceOnGround();
	void CargoPlaceCoordinates();
	void FirePlaceOnGround();
	void FirePlaceCoordinates();
	void BambiBucketRelease();
	void FirePlaceAtCoordinates(vector<float> * pinVectorFireObjectPosition);

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

	void RegisterFloatDataref(float &valueIn, std::string nameIn);
	void RegisterFloatArrayDataref(float *valueIn, std::string nameIn, int sizeIn);
	void RegisterDoubleDataref(double& valueIn, std::string nameIn);
	void RegisterVectorDataref(vector<float> & vectorIn, std::string nameIn);

	void RegisterIntDataref(bool& valueIn, std::string nameIn);
	void RegisterStringDataref(std::string& valueIn, std::string nameIn);


	static vector<float> TurnWorldToAircraft(vector<float> coordsAircraft, CargoDataShared& cargoDataSharedIn);
	vector<float> AircraftToWorld(vector<float> coordsAircraft);
	vector<float> AdjustFrameMovement(vector<float> coordsAircraft);


	float PluginFlightLoopCallback(float elapsedMe, float elapsedSim, int counter, void* refcon);
	void DrawObjects();
	void ReadDataRefs();

	
};

