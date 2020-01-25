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

