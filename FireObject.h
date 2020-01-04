#pragma once
#include "HSL.h"
class FireObject
{
public:
	int myPlaneIndex = 1;

	vector<double> myVectorFireObjectPosition = vector<double>(3);
	vector<double> myVectorFireEmitterOffset = vector<double>(3);

	double myFireWaterRadius = 3.0;
	double myFireLiftNom = 77.0;
	double myFireLiftDenom = 5.7;
	double myFireDistDemon = 20.0;

	double myFireStrength = 1.0;

	double myLat = HSL_Data::coord_invalid;
	double myLon = HSL_Data::coord_invalid;
	double myElev = 0;

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



	FireObject(HSL_PlugIn* pHSL, int inPlaneIndex, vector<double> inVectorFireObjectPosition, bool atCoordinates);

	XPLMDataRef GetPlaneDataRef(std::string prefix, int planeIndex, std::string postfix);
	void UpdateWorldCoordinates(void);
	void SetPosition(void);
	void Remove(void);
	bool CheckWaterDrop(vector<double> inDropPosition, double inWaterAmount);
	bool operator == (const FireObject& s);
};

