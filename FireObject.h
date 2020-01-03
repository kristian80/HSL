#pragma once
#include "HSL.h"
class FireObject
{
public:
	int myPlaneIndex = 1;

	vector<float> myVectorFireObjectPosition = vector<float>(3);
	vector<float> myVectorFireEmitterOffset = vector<float>(3);

	float myFireWaterRadius = 3.0f;
	float myFireLiftNom = 77.0f;
	float myFireLiftDenom = 5.7f;
	float myFireDistDemon = 20.0f;

	float myFireStrength = 1.0f;

	double myLat = HSL_Data::coord_invalid;
	double myLon = HSL_Data::coord_invalid;
	float myElev = 0;

	XPLMDataRef myDrXPos = NULL;
	XPLMDataRef myDrYPos = NULL;
	XPLMDataRef myDrZPos = NULL;

	XPLMDataRef myDrXV = NULL;
	XPLMDataRef myDrYV = NULL;
	XPLMDataRef myDrZV = NULL;

	XPLMDataRef myDrXA = NULL;
	XPLMDataRef myDrYA = NULL;
	XPLMDataRef myDrZA = NULL;

	XPLMDataRef myDrL1 = NULL;
	XPLMDataRef myDrL2 = NULL;
	XPLMDataRef myDrL3 = NULL;
	XPLMDataRef myDrL4 = NULL;
	XPLMDataRef myDrL5 = NULL;



	FireObject(HSL_PlugIn* pHSL, int inPlaneIndex, vector<float> inVectorFireObjectPosition, bool atCoordinates);

	XPLMDataRef GetPlaneDataRef(std::string prefix, int planeIndex, std::string postfix);
	void UpdateWorldCoordinates(void);
	void SetPosition(void);
	void Remove(void);
	bool CheckWaterDrop(vector<float> inDropPosition, float inWaterAmount);
	bool operator == (const FireObject& s);
};

