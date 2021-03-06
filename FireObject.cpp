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

#include "FireObject.h"
#include "HSL_PlugIn.h"

FireObject::FireObject(HSL_PlugIn *pHSL, int inPlaneIndex, vector<double> inVectorFireObjectPosition, bool inAtCoordinates)
{
	myPlaneIndex = inPlaneIndex;

	myFireWaterRadius = pHSL->myFireWaterRadius;
	myFireLiftNom = pHSL->myFireLiftNom;
	myFireLiftDenom = pHSL->myFireLiftDenom;
	myFireDistDemon = pHSL->myFireDistDemon;
	myFireStrength = pHSL->myFireStrengthStart;

	myVectorFireObjectPosition = inVectorFireObjectPosition;
	myVectorFireEmitterOffset = pHSL->myVectorFireEmitterOffset;

	

	myDrXV = GetPlaneDataRef("sim/multiplayer/position/plane", myPlaneIndex, "_v_x");
	myDrYV = GetPlaneDataRef("sim/multiplayer/position/plane", myPlaneIndex, "_v_y");
	myDrZV = GetPlaneDataRef("sim/multiplayer/position/plane", myPlaneIndex, "_v_z");

	myDrXA = GetPlaneDataRef("sim/multiplayer/position/plane", myPlaneIndex, "_phi");
	myDrYA = GetPlaneDataRef("sim/multiplayer/position/plane", myPlaneIndex, "_psi");
	myDrZA = GetPlaneDataRef("sim/multiplayer/position/plane", myPlaneIndex, "_the");

	myDrL1 = GetPlaneDataRef("sim/multiplayer/position/plane", myPlaneIndex, "_strobe_lights_on");
	myDrL2 = GetPlaneDataRef("sim/multiplayer/position/plane", myPlaneIndex, "_nav_lights_on");
	myDrL3 = GetPlaneDataRef("sim/multiplayer/position/plane", myPlaneIndex, "_taxi_light_on");
	myDrL4 = GetPlaneDataRef("sim/multiplayer/position/plane", myPlaneIndex, "_landing_lights_on");
	myDrL5 = GetPlaneDataRef("sim/multiplayer/position/plane", myPlaneIndex, "_beacon_lights_on");

	myDrXPos = GetPlaneDataRef("sim/multiplayer/position/plane", myPlaneIndex, "_x");
	myDrYPos = GetPlaneDataRef("sim/multiplayer/position/plane", myPlaneIndex, "_y");
	myDrZPos = GetPlaneDataRef("sim/multiplayer/position/plane", myPlaneIndex, "_z");

	if (inAtCoordinates == true)
	{
		myLat = pHSL->myFireSetLatitutde;
		myLon = pHSL->myFireSetLongitude;
		myElev = pHSL->myFireSetElevation;
		UpdateWorldCoordinates();
	}
	SetPosition();
}

XPLMDataRef FireObject::GetPlaneDataRef(std::string prefix, int planeIndex, std::string postfix)
{
	std::string fullDr = prefix + std::to_string(planeIndex) + postfix;
	return XPLMFindDataRef(fullDr.c_str());
}

void FireObject::UpdateWorldCoordinates(void)
{
	if (pHSL->myGroundProbe == NULL) return;
	if (myLat == HSL_Data::coord_invalid) return;
	if (myLon == HSL_Data::coord_invalid) return;

	double zero_x, zero_y, zero_z;
	XPLMWorldToLocal(myLat, myLon, 0, &zero_x, &zero_y, &zero_z);

	XPLMProbeInfo_t info;
	info.structSize = sizeof(info);

	XPLMProbeResult result = XPLMProbeTerrainXYZ(pHSL->myGroundProbe, (float)zero_x, (float)zero_y, (float)zero_z, &info);

	double local_long;
	double local_lat;
	double local_alt;

	XPLMLocalToWorld(info.locationX, info.locationY, info.locationZ, &local_lat, &local_long, &local_alt);
	XPLMWorldToLocal(myLat, myLon, local_alt, &zero_x, &zero_y, &zero_z); 

	myVectorFireObjectPosition(0) = zero_x;
	myVectorFireObjectPosition(2) = zero_z;
	
	XPLMProbeTerrainXYZ(pHSL->myGroundProbe, (float)zero_x, (float)zero_y, (float)zero_z, &info); // Once again for improved precision
	myVectorFireObjectPosition(1) = info.locationY + myElev;
	SetPosition();
}

void FireObject::SetPosition(void)
{
	XPLMSetDataf(myDrXPos, (float)myVectorFireObjectPosition(0));
	XPLMSetDataf(myDrYPos, (float)myVectorFireObjectPosition(1));
	XPLMSetDataf(myDrZPos, (float)myVectorFireObjectPosition(2) + 0.5f);

	// Does not always work on first try, so we do it every time. 
	XPLMSetDataf(myDrXV, 0);
	XPLMSetDataf(myDrYV, 0);
	XPLMSetDataf(myDrZV, 0);

	XPLMSetDataf(myDrXA, 0);
	XPLMSetDataf(myDrYA, 0);
	XPLMSetDataf(myDrZA, 0);

	XPLMSetDatai(myDrL1, 0);
	XPLMSetDatai(myDrL2, 0);
	XPLMSetDatai(myDrL3, 0);
	XPLMSetDatai(myDrL4, 0);
	XPLMSetDatai(myDrL5, 0);


}

void FireObject::SetPosition(vector<double> inVectorFireObjectPosition, vector<double> inVectorFireObjectVelocity)
{
	XPLMSetDataf(myDrXPos, (float)inVectorFireObjectPosition(0));
	XPLMSetDataf(myDrYPos, (float)inVectorFireObjectPosition(1));
	XPLMSetDataf(myDrZPos, (float)inVectorFireObjectPosition(2));

	// Does not always work on first try, so we do it every time. 
	XPLMSetDataf(myDrXV, (float)inVectorFireObjectVelocity(0));
	XPLMSetDataf(myDrYV, (float)inVectorFireObjectVelocity(1));
	XPLMSetDataf(myDrZV, (float)inVectorFireObjectVelocity(2));

	XPLMSetDataf(myDrXA, 0);
	XPLMSetDataf(myDrYA, 0);
	XPLMSetDataf(myDrZA, 0);

	XPLMSetDatai(myDrL1, 0);
	XPLMSetDatai(myDrL2, 0);
	XPLMSetDatai(myDrL3, 0);
	XPLMSetDatai(myDrL4, 0);
	XPLMSetDatai(myDrL5, 0);


}

void FireObject::Remove(void)
{
	XPLMSetDataf(myDrXPos, 0);
	XPLMSetDataf(myDrYPos, 0);
	XPLMSetDataf(myDrZPos, 0);

	/*char loadAircraftPath[2048];
	char* pLoadAircraftPath[2];

	pLoadAircraftPath[0] = loadAircraftPath;
	pLoadAircraftPath[1] = NULL;

	strcpy(loadAircraftPath, inNoAircraftPath);*/

}

bool FireObject::CheckWaterDrop(vector<double> inDropPosition, double inWaterAmount)
{
	double distance_x = myVectorFireObjectPosition(0) - inDropPosition(0);
	double distance_z = myVectorFireObjectPosition(2) - inDropPosition(2);

	double distance_h = sqrt((distance_x * distance_x) + (distance_z * distance_z));

	if (distance_h <= myFireWaterRadius)
	{
		myFireStrength -= inWaterAmount;
	}

	if (myFireStrength <= 0)
		return true;

	return false;
}

bool FireObject::operator==(const FireObject& s)
{
	return s.myPlaneIndex == myPlaneIndex;
}
