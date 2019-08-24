#pragma once


#include "HSL.h"
#include "HSLImguiWidget.h"
#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/io.hpp> 
using namespace boost::numeric::ublas;

class HSL_PlugIn
{
public:
	std::string myDS = "";
	std::string mySystemPath = "";
	std::string myConfigPath = "";
	std::string myAircraftPath = "";

	std::shared_ptr<HSLImguiWidget> imguiPtr;

	bool myInitialized = false;
	int myPluginEnabled = 0;
	int myAircraftLoaded = 0;

	bool mySlingLineEnabled = false;

	float myDataRate = -1;

	int myPluginMenu = 0;
	int myEnableSlingMenu = 0;
	XPLMMenuID myPluginMenuID = 0;

	std::string myWinchPath = "pm_library/Tennis_Ball.obj";
	std::string myRopePath = "pm_library/Tennis_Ball.obj";
	std::string myCargoPath = "swisscreations/oggetti_eli/benna_beton_fissa.obj";

	XPLMObjectRef myWinchObjectRef = NULL;
	XPLMObjectRef myRopeObjectRef = NULL;
	XPLMObjectRef myCargoObjectRef = NULL;
	XPLMInstanceRef myWinchInstanceRef = NULL;
	XPLMProbeRef myGroundProbe = NULL;
	float myDummyAngle = 0;

	std::vector<vector<float>> myRopePoints;

	/////////////////////////////////////////////////////////
	// Sling Variables

	vector<float> myVectorHelicopterPosition = vector<float>(3);
	vector<float> myVectorHookPosition = vector<float>(3);
	vector<float> myVectorHookVelocity = vector<float>(3);
	vector<float> myVectorForceGravity = vector<float>(3);
	vector<float> myVectorWindVelocity = vector<float>(3);
	vector<float> myVectorWinchPosition = vector<float>(3);
	vector<float> myVectorObjectOffset = vector<float>(3);
	vector<float> myVectorZeroVector = vector<float>(3);

	vector<float> myVectorCargoOffset = vector<float>(3);
	vector<float> myVectorCargoAngle = vector<float>(3);

	vector<float> myVectorRotationTest = vector<float>(3);

	float myFrameTime;
	float myObjectTerrainLevel;

	float myRopeLengthNormal = 2.2f;
	float myRopeDamping = 0.03f;
	float myRopeK = 55000.0f;
	float myObjectMass = 75; //kg
	float myObjectCrossSection = 1.1f; //m2
	float myObjectCWFront = 0.9; //cube
	float myObjectFrictionGlide = 0.35;
	float myObjectFrictionStatic = 0.65;
	float myObjectSpeedStaticFriction = 0.3;
	float myGravitation = -9.8;
	float myObjectHeight = 0.0f;

	float myCurrentRopeLength = 2.2f;

	float myDefaultRopeLengthNormal = 2.0f;
	float myDefaultRopeDamping = 0.03f;
	float myDefaultRopeK = 55000.0f;
	float myDefaultObjectMass = 75; //kg
	float myDefaultObjectCrossSection = 1.1f; //m2
	float myDefaultObjectCWFront = 0.9; //cube
	float myDefaultObjectFrictionGlide = 0.35;
	float myDefaultObjectFrictionStatic = 0.65;
	float myDefaultObjectSpeedStaticFriction = 0.3;
	float myDefaultObjectHeight = -0.7f;

	vector<float> myVectorDefaultWinchPosition = vector<float>(3);


	bool myTerrainHit = false;
	vector<float> myVectorRope;
	float myNewRopeLength = 0;
	float myRopeStretchRelative;
	float myRopeForceScalar;
	float myRopeLengthDelta;
	float myRopeStretchSpeed;
	float myRopeCorrectedD;
	vector<float> myVectorForceRope;
	vector<float> myVectorAirVelocity;
	float myAirSpeed;
	float myAirResistance;
	vector<float> myVectorForceAir;
	vector<float> myVectorForceTotal;
	vector<float> myVectorHorizontalVelocity;
	vector<float> myVectorForceFriction;
	vector<float> myVectorAccTotal;
	vector<float> myVectorVelocityDelta;
	vector<float> myVectorForceChopper;
	vector<float> myVectorMomentumChopper;


	///////////////////////////////////////////////////
	// DataRefs

	XPLMDataRef myDrLocalX;
	XPLMDataRef myDrLocalY;
	XPLMDataRef myDrLocalZ;

	XPLMDataRef myDrLocalPhi;
	XPLMDataRef myDrLocalPsi;
	XPLMDataRef myDrLocalTheta;

	XPLMDataRef myDrPause;
	XPLMDataRef myDrWindDirection;
	XPLMDataRef myDrWindSpeed;

	XPLMDataRef myDrForceX;
	XPLMDataRef myDrForceY;
	XPLMDataRef myDrForceZ;

	XPLMDataRef myDrMomentumX;
	XPLMDataRef myDrMomentumY;
	XPLMDataRef myDrMomentumZ;

	///////////////////////////////////////////////////
	// DataRefs Local Copies

	double myLdLocalX;
	double myLdLocalY;
	double myLdLocalZ;

	double myLastLocalX;
	double myLastLocalY;
	double myLastLocalZ;

	float myLfLocalPhi;
	float myLfLocalPsi;
	float myLfLocalTheta;

	int myLiPause;
	float myLfWindDirection;
	float myLfWindSpeed;

	float myLfForceX;
	float myLfForceY;
	float myLfForceZ;

	float myLfMomentumX;
	float myLfMomentumY;
	float myLfMomentumZ;



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

	void SlingEnable();
	void SlingDisable();
	void SlingReset();

	vector<float> HSL_PlugIn::TurnWorldToAircraft(vector<float> coordsAircraft);
	vector<float> AircraftToWorld(vector<float> coordsAircraft);
	vector<float> AdjustFrameMovement(vector<float> coordsAircraft);


	float PluginFlightLoopCallback(float elapsedMe, float elapsedSim, int counter, void* refcon);
	void DrawObjects();
	void ReadDataRefs();

	
};

