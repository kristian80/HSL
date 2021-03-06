/*
 * This file is part of the HSL distribution (https://github.com/kristian80/HSL).
 * Copyright (c) 2019 Kristian80, based on the Imgui Starter Window for X-Plane
 * by William Good
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

#include "HSLImguiWidget.h"
#include "HSL_PlugIn.h"
#include "LoadBaseObject.h"
#include "CargoObject.h"
#include <boost/range/adaptor/reversed.hpp>

//#include "MyIvyConfigAircraft.h"

#include <iomanip> // setprecision
#include <sstream> // stringstream

#include <misc/cpp/imgui_stdlib.h>


HSLImguiWidget::HSLImguiWidget(HSL_PlugIn *pHSLNew, int left, int top, int right, int bot, int decoration) :
	ImgWindow(left, top, right, bot, decoration)
{ 
	pHSL = pHSLNew;
	SetWindowTitle("Helicopter Sling Line");
	SetVisible(false);
	configureImguiContext();
}


HSLImguiWidget::~HSLImguiWidget()
{
}

void HSLImguiWidget::configureImguiContext()
{

}

void HSLImguiWidget::Visible(bool visible)
{
	SetVisible(visible);
	if (visible)
	{
		selected_radio = 0;
	}
}

void HSLImguiWidget::InputVector(vector<double>& vectorIn, std::string nameIn)
{
	for (int i = 0; i < vectorIn.size(); i++)
	{
		std::string name = nameIn + std::to_string(i + 1);
		//double* var = &(vectorIn(i));
		ImGui::InputDouble(name.c_str(), &(vectorIn(i)), 0.01, 0.01,"%.3f");
	}
}

void HSLImguiWidget::OutputVector(vector<double>& vectorOut, std::string nameOut)
{
	ImGui::PushItemWidth(50);
	ImGui::Text(nameOut.c_str());
	ImGui::PopItemWidth();

	ImGui::PushItemWidth(30);
	for (int i = 0; i < vectorOut.size(); i++)
	{
		ImGui::SameLine();
		ImGui::Text("%3.3f", vectorOut(i));
	}

	ImGui::PopItemWidth();


}

void HSLImguiWidget::buildInterface()
{
	static bool HIGH_PERFORMANCE = false;
	CARGO_SHM_SECTION_START

	HIGH_PERFORMANCE = pHSL->myCargoDataShared.myHighPerformace;

	win_width = ImGui::GetWindowWidth();
	win_height = ImGui::GetWindowHeight();

	ImGui::Columns(4, 0, false);



	
	ImGui::PushItemWidth(100);
	ImGui::Checkbox("Simple Mode", &(pHSL->mySimpleMode));
	ImGui::Checkbox("High Performace", &(pHSL->myCargoDataShared.myHighPerformace));

	double loadFreq = 0;
	
	if ((pHSL->myHook.myFollowOnly == false) && (pHSL->myHook.myRopeConnected == true))
		loadFreq = sqrt(pHSL->myCargoDataShared.myRopeK / pHSL->myHook.myMass) / (2.0f * M_PI);
	else if ((pHSL->myCargo.myFollowOnly == false) && (pHSL->myCargo.myRopeConnected == true))
		loadFreq = sqrt(pHSL->myCargoDataShared.myRopeK / (pHSL->myCargo.myMass + pHSL->myCargo.myBambiBucketWaterWeight)) / (2.0f * M_PI);

	if (250 <= pHSL->myComputationFrequency)
	{
		ImVec4 col = ImColor(0, 255, 0);
		ImGui::PushStyleColor(ImGuiCol_Text, col);
	}
	else if (100 <= pHSL->myComputationFrequency)
	{
		ImVec4 col = ImColor(255, 255, 0);
		ImGui::PushStyleColor(ImGuiCol_Text, col);
	}
	else
	{
		ImVec4 col = ImColor(255, 0, 0);
		ImGui::PushStyleColor(ImGuiCol_Text, col);
	}

	ImGui::Text("Computation Freq: %d Hz", pHSL->myComputationFrequency);
	//ImGui::Text("Load Resonance: %d Hz", (int) loadFreq);

	ImGui::PopStyleColor();


	////////////////////////////////////////////////////////////////////////////
	// Simple Mode
	if (pHSL->mySimpleMode == true)
	{
		ImGui::Text("Winch:");
		InputVector(pHSL->myCargoDataShared.myVectorWinchPosition, "Winch Position");
		if (ImGui::Button("Write Aircraft Ini File")) pHSL->AircraftConfigSave();

		ImGui::Text("Select Profile:");
		if (ImGui::BeginCombo("##Folder", pHSL->mySelectedProfileName.c_str()))
		{
			for (int index = 0; index < pHSL->myProfileNames.size(); index++)
			{
				bool is_selected = (pHSL->mySelectedProfileIndex == index);
				std::string profileName = pHSL->myProfileNames[index];

				if (ImGui::Selectable(profileName.c_str(), is_selected))
				{
					pHSL->mySelectedProfileIndex = index;
					pHSL->mySelectedProfileName = profileName;
					pHSL->mySelectedProfilePath = pHSL->myProfilePaths[index];
				}
				if (is_selected)
					ImGui::SetItemDefaultFocus();
			}
			ImGui::EndCombo();
		}
		ImGui::PushItemWidth(80);
		if (ImGui::Button("Read Profile"))
		{
			pHSL->ConfigRead(pHSL->mySelectedProfilePath);
			pHSL->UpdateObjects();
		}
		ImGui::SameLine();
		if (ImGui::Button("Save Profile")) pHSL->ConfigSave(pHSL->mySelectedProfilePath);
		ImGui::PopItemWidth();
		ImGui::InputText("New Profile", &(pHSL->myNewProfileName));
		if (ImGui::Button("Create Profile"))
		{
			pHSL->ConfigSave(pHSL->myNewProfileName + ".ini");
			pHSL->ReadProfiles();

			for (int index = 0; index < pHSL->myProfileNames.size(); index++)
			{
				if (pHSL->myProfileNames[index].compare(pHSL->myNewProfileName) == 0)
				{
					pHSL->mySelectedProfileIndex = index;
					pHSL->mySelectedProfileName = pHSL->myProfileNames[index];
					pHSL->mySelectedProfilePath = pHSL->myProfilePaths[index];
				}
			}
		}


		/*
		if (ImGui::BeginCombo("##Folder", pHRM->m_global_path.c_str()))
		{
			for (int index = 0; index < pHRM->m_path_vector.size(); index++)
			{
				bool is_selected = (pHRM->m_global_path_index == index);
				std::string folder_name = pHRM->m_path_vector[index];

				if (ImGui::Selectable(folder_name.c_str(), is_selected))
				{
					pHRM->m_global_path_index = index;
					pHRM->m_global_path = folder_name;
				}
				if (is_selected)
					ImGui::SetItemDefaultFocus();
			}
			ImGui::EndCombo();
		}
		*/

		
		//ImGui::Checkbox("Bambi Bucket Release", &(pHSL->myCargo.myBambiBucketRelease));

		ImGui::NextColumn();
		ImGui::PushItemWidth(100);

		
		ImGui::Text("Rope Parameters:");
		ImGui::InputDouble("Rope Length Start [m]", &(pHSL->myCargoDataShared.myRopeLengthStart), 0.01, 0.01, "%.3f", 0);
		ImGui::InputDouble("Rope Length [m]", &(pHSL->myCargoDataShared.myRopeLengthNormal), 0.01, 0.01, "%.3f", 0);
		ImGui::InputDouble("Rope Rupture Force [N]", &(pHSL->myCargoDataShared.myRopeRuptureForce), 0.01, 0.01, "%.3f", 0);
		ImGui::InputDouble("Rope Damping [0...1]", &(pHSL->myCargoDataShared.myRopeDamping), 0.01, 0.01, "%.3f", 0);
		ImGui::InputDouble("Rope Stiffness [N/rel_stretch]", &(pHSL->myCargoDataShared.myRopeK), 0.01, 0.01, "%.3f", 0);
		ImGui::InputDouble("Winch Speed [m/s]", &(pHSL->myCargoDataShared.myWinchSpeed), 0.01, 0.01, "%.3f", 0);

		
		/*
		ImGui::Text("Hook Parameters:");
		ImGui::InputDouble("Hook Mass [kg]", &(pHSL->myHook.myMass), 0.01, 0.01, "%.3f", 0);
		InputVector(pHSL->myHook.myVectorSize, "Hook Size L/W/H [m]");
		InputVector(pHSL->myHook.myVectorCW, "Hook CW F/S/T [m]");
		ImGui::InputDouble("Hook Height [m]", &(pHSL->myHook.myHeight), 0.01, 0.01, "%.3f", 0);
		ImGui::InputDouble("Hook Friction Glide", &(pHSL->myHook.myFrictionGlide), 0.01, 0.01, "%.3f", 0);
		ImGui::InputDouble("Hook Friction Static", &(pHSL->myHook.myFrictionStatic), 0.01, 0.01, "%.3f", 0);
		*/
		ImGui::NextColumn();
		ImGui::PushItemWidth(100);

		ImGui::Text("Cargo Parameters:");
		ImGui::InputDouble("Cargo Mass [kg]", &(pHSL->myCargo.myMass), 0.01, 0.01, "%.3f", 0);

		InputVector(pHSL->myCargo.myVectorSize, "Cargo Size L/W/H [m]");
		InputVector(pHSL->myCargo.myVectorCW, "Cargo CW F/T/S [m]");

		ImGui::InputDouble("Cargo Height [m]", &(pHSL->myCargo.myHeight), 0.01, 0.01, "%.3f", 0);
		ImGui::InputDouble("Cargo Friction Glide", &(pHSL->myCargo.myFrictionGlide), 0.01, 0.01, "%.3f", 0);
		ImGui::InputDouble("Cargo Friction Static", &(pHSL->myCargo.myFrictionStatic), 0.01, 0.01, "%.3f", 0);
		InputVector(pHSL->myCargo.myVectorCargoOffset, "Cargo Offset");

		ImGui::Checkbox("Cargo Is Bambi Bucket", &(pHSL->myCargo.myIsBambiBucket));


	}
	

	////////////////////////////////////////////////////////////////////////////
	// Expert Mode
	else
	{
		ImGui::PushItemWidth(100);
		ImGui::Checkbox("Use Rope Sphere", &(pHSL->myRopeDrawSphere));
		ImGui::Checkbox("Physics Enabled", &(pHSL->myCargoDataShared.myPhysicsEnabled));
		

		if (pHSL->myCargoDataShared.myPhysicsEnabled == false)
		{
			ImGui::Text("Object:");
			InputVector(pHSL->myCargo.myVectorPosition, "Cargo Position");
			InputVector(pHSL->myHook.myVectorPosition, "Hook Position");
		}

		//if (ImGui::Button("Load Settings")) pHSL->ConfigRead();
		//if (ImGui::Button("Save Settings")) pHSL->ConfigSave();

		ImGui::Text("Winch:");
		InputVector(pHSL->myCargoDataShared.myVectorWinchPosition, "Winch Position");
		if (ImGui::Button("Write Aircraft Ini File")) pHSL->AircraftConfigSave();

		ImGui::Text("Set Cargo Coordinates:");
		ImGui::InputDouble("Latitude", &(pHSL->myCargoDataShared.myCargoSetLatitutde), 0.01, 0.01, "%.9f");
		ImGui::InputDouble("Longitude", &(pHSL->myCargoDataShared.myCargoSetLongitude), 0.01, 0.01, "%.9f");

		ImGui::Checkbox("Cargo Is Bambi Bucket", &(pHSL->myCargo.myIsBambiBucket));
		ImGui::Checkbox("Bambi Bucket Release", &(pHSL->myCargo.myBambiBucketRelease));
		ImGui::InputDouble("Water Flow [kg/s]", &(pHSL->myCargoDataShared.myBambiBucketWaterFlow), 1, 10, "%.3f", 0);
		ImGui::InputDouble("Water Speed [m/s]", &(pHSL->myRainSpeed), 0.1, 0.1, "%.3f", 0);
		ImGui::InputInt("Drop Directions", &(pHSL->myRainDirections), 1, 1);
		ImGui::InputInt("Drop Variance", &(pHSL->myRainVariance), 1, 1);
		ImGui::InputDouble("Release Period [s]", &(pHSL->myRainReleasePeriod), 0.01, 0.1, "%.3f", 0);

		ImGui::Text("Rope Parameters:");
		ImGui::InputDouble("Rope Length Start [m]", &(pHSL->myCargoDataShared.myRopeLengthStart), 0.01, 0.01, "%.3f", 0);
		ImGui::InputDouble("Rope Length [m]", &(pHSL->myCargoDataShared.myRopeLengthNormal), 0.01, 0.01, "%.3f", 0);
		ImGui::InputDouble("Rope Rupture Force [N]", &(pHSL->myCargoDataShared.myRopeRuptureForce), 0.01, 0.01, "%.3f", 0);
		ImGui::InputDouble("Rope Damping", &(pHSL->myCargoDataShared.myRopeDamping), 0.01, 0.01, "%.3f", 0);
		ImGui::InputDouble("Rope Artifical Damping [N]", &(pHSL->myCargoDataShared.myRopeArtificialDampingForce), 0.01, 0.01, "%.3f", 0);
		ImGui::InputDouble("Rope K [N/relative_stretch]", &(pHSL->myCargoDataShared.myRopeK), 0.01, 0.01, "%.3f", 0);
		ImGui::InputDouble("MaxRopeAcc", &(pHSL->myCargoDataShared.myMaxAccRopeFactor), 0.01, 0.01, "%.3f", 0);
		ImGui::InputDouble("Winch Speed [m/s]", &(pHSL->myCargoDataShared.myWinchSpeed), 0.01, 0.01, "%.3f", 0);
		ImGui::InputDouble("Winch Operator Strength [N]", &(pHSL->myCargoDataShared.myRopeOperatorDampingForce), 0.01, 0.01, "%.3f", 0);
		ImGui::InputDouble("Winch Operator Start [m]", &(pHSL->myCargoDataShared.myRopeOperatorDampingLength), 0.01, 0.01, "%.3f", 0);

		ImGui::Text("Hook Parameters:");
		ImGui::InputDouble("Hook Mass [kg]", &(pHSL->myHook.myMass), 0.01, 0.01, "%.3f", 0);
		InputVector(pHSL->myHook.myVectorSize, "Hook Size L/W/H [m]");
		InputVector(pHSL->myHook.myVectorCW, "Hook CW F/S/T [m]");
		ImGui::InputDouble("Hook Height [m]", &(pHSL->myHook.myHeight), 0.01, 0.01, "%.3f", 0);
		ImGui::InputDouble("Hook Friction Glide", &(pHSL->myHook.myFrictionGlide), 0.01, 0.01, "%.3f", 0);
		ImGui::InputDouble("Hook Friction Static", &(pHSL->myHook.myFrictionStatic), 0.01, 0.01, "%.3f", 0);

		ImGui::Text("Cargo Parameters:");
		ImGui::InputDouble("Cargo Mass [kg]", &(pHSL->myCargo.myMass), 0.01, 0.01, "%.3f", 0);

		InputVector(pHSL->myCargo.myVectorSize, "Cargo Size L/W/H [m]");
		InputVector(pHSL->myCargo.myVectorCW, "Cargo CW F/T/S [m]");

		ImGui::InputDouble("Cargo Height [m]", &(pHSL->myCargo.myHeight), 0.01, 0.01, "%.3f", 0);
		ImGui::InputDouble("Cargo Friction Glide", &(pHSL->myCargo.myFrictionGlide), 0.01, 0.01, "%.3f", 0);
		ImGui::InputDouble("Cargo Friction Static", &(pHSL->myCargo.myFrictionStatic), 0.01, 0.01, "%.3f", 0);
		InputVector(pHSL->myCargo.myVectorCargoOffset, "Cargo Offset");
		InputVector(pHSL->myCargo.myVectorCargoRotation, "Cargo Rotation");

		ImGui::NextColumn();

		ImGui::Text("Fires:");
		ImGui::Text(pHSL->myFireAircraftPath.c_str());
		for (auto pFire : pHSL->myFires)
		{
			ImGui::Text("plane index  %i", pFire->myPlaneIndex);
			ImGui::Text("strength     %.3f", pFire->myFireStrength);
		}

		ImGui::Text("Sling:");

		//ImGui::Text("CargoDevX:      %.9f", pHSL->myCargoDataShared.myVectorHelicopterVelocity(0));
		//ImGui::Text("CargoDevY:      %.9f", pHSL->myCargoDataShared.myVectorHelicopterVelocity(1));
		//ImGui::Text("CargoDevZ:      %.9f", pHSL->myCargoDataShared.myVectorHelicopterVelocity(2));

		OutputVector(pHSL->myCargo.myVectorHelicopterPositionDeviation, "Cargo Dev");
		OutputVector(pHSL->myHook.myVectorHelicopterPositionDeviation, "Hook Dev");
		OutputVector(pHSL->myCargo.myVectorHelicopterVelocityApprox, "Cargo Vel");
		OutputVector(pHSL->myHook.myVectorHelicopterVelocityApprox, "Hook Vel");
		OutputVector(pHSL->myCargoDataShared.myVectorHookPosition, "Hook Pos");
		OutputVector(pHSL->myCargoDataShared.myVectorRope, "Rope");
		OutputVector(pHSL->myCargoDataShared.myVectorWinchPosition, "WinchPosition");

		OutputVector(pHSL->myCargo.myVectorPosition, "CargoPosition");
		OutputVector(pHSL->myCargo.myVectorVelocity, "Cargo:Velocity");
		OutputVector(pHSL->myCargo.myVectorCrossSection, "Cargo:CrossSection");

		OutputVector(pHSL->myCargo.myVectorDisplayOffset, "Cargo:Object Offset");
		OutputVector(pHSL->myCargo.myVectorDisplayAngle, "Cargo:Object Angle");

		OutputVector(pHSL->myCargo.myVectorForceGravity, "Cargo:ForceGravity");
		OutputVector(pHSL->myCargo.myVectorWindVelocity, "Cargo:WindVelocity");
		OutputVector(pHSL->myCargo.myVectorForceRope, "Cargo:ForceRope");
		OutputVector(pHSL->myCargo.myVectorAirVelocity, "Cargo:AirVelocity");
		OutputVector(pHSL->myCargo.myVectorWaterVelocity, "Cargo:WaterVelocity");
		OutputVector(pHSL->myCargo.myVectorForceAir, "Cargo:ForceAir");
		OutputVector(pHSL->myCargo.myVectorForceAirNew, "Cargo:ForceAirNew");
		OutputVector(pHSL->myCargo.myVectorForceAirCart, "Cargo:ForceAirCart");
		OutputVector(pHSL->myCargo.myVectorForceWater, "Cargo:ForceWater");
		OutputVector(pHSL->myCargo.myVectorForceSwim, "Cargo:ForceSwim");
		OutputVector(pHSL->myCargo.myVectorForceOperator, "Cargo:ForceOperator");
		OutputVector(pHSL->myCargo.myVectorForceTotal, "Cargo:ForceTotal");
		OutputVector(pHSL->myCargo.myVectorHorizontalVelocity, "Cargo:HorizontalVelocity");
		OutputVector(pHSL->myCargo.myVectorForceFriction, "Cargo:ForceFriction");
		OutputVector(pHSL->myCargo.myVectorAccTotal, "Cargo:AccTotal");
		OutputVector(pHSL->myCargo.myVectorVelocityDelta, "Cargo:VelocityDelta");
		OutputVector(pHSL->myCargo.myVectorForceChopper, "Cargo:ForceChopper");
		OutputVector(pHSL->myCargo.myVectorMomentumChopper, "Cargo:MomentumChopper");

		OutputVector(pHSL->myHook.myVectorPosition, "HookPosition");
		OutputVector(pHSL->myHook.myVectorVelocity, "Hook:Velocity");
		OutputVector(pHSL->myHook.myVectorCrossSection, "Hook:CrossSection");
		OutputVector(pHSL->myHook.myVectorDisplayOffset, "Hook:Object Offset");
		OutputVector(pHSL->myHook.myVectorDisplayAngle, "Hook:Object Angle");

		OutputVector(pHSL->myHook.myVectorForceGravity, "Hook:ForceGravity");
		OutputVector(pHSL->myHook.myVectorWindVelocity, "Hook:WindVelocity");
		OutputVector(pHSL->myHook.myVectorForceRope, "Hook:ForceRope");
		OutputVector(pHSL->myHook.myVectorAirVelocity, "Hook:AirVelocity");
		OutputVector(pHSL->myHook.myVectorWaterVelocity, "Hook:WaterVelocity");
		OutputVector(pHSL->myHook.myVectorForceAir, "Hook:ForceAir");
		
		OutputVector(pHSL->myHook.myVectorForceWater, "Hook:ForceWater");
		OutputVector(pHSL->myHook.myVectorForceSwim, "Hook:ForceSwim");
		OutputVector(pHSL->myHook.myVectorForceOperator, "Hook:ForceOperator");
		OutputVector(pHSL->myHook.myVectorForceTotal, "Hook:ForceTotal");
		OutputVector(pHSL->myHook.myVectorHorizontalVelocity, "Hook:HorizontalVelocity");
		OutputVector(pHSL->myHook.myVectorForceFriction, "Hook:ForceFriction");
		OutputVector(pHSL->myHook.myVectorAccTotal, "Hook:AccTotal");
		OutputVector(pHSL->myHook.myVectorVelocityDelta, "Hook:VelocityDelta");
		OutputVector(pHSL->myHook.myVectorForceChopper, "Hook:ForceChopper");
		OutputVector(pHSL->myHook.myVectorMomentumChopper, "Hook:MomentumChopper");


		ImGui::NextColumn();



		ImGui::Text("FrameTime [s]:      %.3f", pHSL->myCargoDataShared.myFrameTime);
		ImGui::Text("FlightLoopTime [us]:%d", pHSL->myProcessingTimeFlightLoop);
		ImGui::Text("CargoFrameMax [ns]: %.1f", pHSL->myCargoDataShared.myFrameTimeMax);
		ImGui::Text("DrawTime [us]:      %d", pHSL->myProcessingTimeDrawRoutine);
		ImGui::Text("Comp per FL:        %d", pHSL->myCompuationsPerFlightLoop);
		ImGui::Text("Cargo per FL:       %d", pHSL->myCompuationsPerFlightLoop);

		ImGui::Text("Raindrops:          %d", pHSL->myRainDropNumber);
		ImGui::Text("Water per drop [l]: %.2f", pHSL->myBambiBucketWaterPerDrop);
		ImGui::Text("RD Thread Overvlow: %d", pHSL->myRainDropOverflow);


		ImGui::Text("CurrentRopeLength:  %.3f", pHSL->myCargoDataShared.myCurrentRopeLength);
		ImGui::Text("RopeLength:         %.3f", pHSL->myCargoDataShared.myNewRopeLength);
		ImGui::Text("StretchRelative:    %.3f", pHSL->myCargoDataShared.myRopeStretchRelative);
		ImGui::Text("ForceScalar:        %.3f", pHSL->myCargoDataShared.myRopeForceScalar);
		ImGui::Text("LengthDelta:        %.3f", pHSL->myCargoDataShared.myRopeLengthDelta);
		ImGui::Text("StretchSpeed:       %.3f", pHSL->myCargoDataShared.myRopeStretchSpeed);
		ImGui::Text("CorrectedD:         %.3f", pHSL->myCargoDataShared.myRopeCorrectedD);

		ImGui::Text("RopeRupture         %d", pHSL->myCargoDataShared.myRopeRuptured);

		ImGui::Text("Cargo:TerrainHit:         %d", pHSL->myCargo.myTerrainHit);
		ImGui::Text("Cargo:ObjectTerrainLevel: %.3f", pHSL->myCargo.myObjectTerrainLevel);
		ImGui::Text("Cargo:AirSpeed:           %.3f", pHSL->myCargo.myAirSpeed);
		ImGui::Text("Cargo:AirResistance:      %.3f", pHSL->myCargo.myAirResistance);
		ImGui::Text("Cargo:SpeedStaticFriction:%.3f", pHSL->myCargo.mySpeedStaticFriction);
		ImGui::Text("Cargo:WaterLevel:         %.3f", pHSL->myCargo.myWaterLevel);
		ImGui::Text("Cargo:Volume:             %.3f", pHSL->myCargo.myVolume);
		ImGui::Text("Cargo:BambiWaterLevel:    %.3f", pHSL->myCargo.myBambiBucketWaterLevel);
		ImGui::Text("Cargo:BambiWaterWeight:   %.3f", pHSL->myCargo.myBambiBucketWaterWeight);

		ImGui::Text("Hook:TerrainHit:         %d", pHSL->myHook.myTerrainHit);
		ImGui::Text("Hook:ObjectTerrainLevel: %.3f", pHSL->myHook.myObjectTerrainLevel);
		ImGui::Text("Hook:AirSpeed:           %.3f", pHSL->myHook.myAirSpeed);
		ImGui::Text("Hook:AirResistance:      %.3f", pHSL->myHook.myAirResistance);
		ImGui::Text("Hook:SpeedStaticFriction:%.3f", pHSL->myHook.mySpeedStaticFriction);
		ImGui::Text("Hook:WaterLevel:         %.3f", pHSL->myHook.myWaterLevel);
		ImGui::Text("Hook:Volume:             %.3f", pHSL->myHook.myVolume);

		ImGui::Text("Debug1              %f", pHSL->myCargoDataShared.myDebugValue1);
		ImGui::Text("Debug2              %f", pHSL->myCargoDataShared.myDebugValue2);
		ImGui::Text("Debug3              %f", pHSL->myCargoDataShared.myDebugValue3);
		ImGui::Text("Debug4              %f", pHSL->myCargoDataShared.myDebugValue4);

		if (pHSL->myCargoDataShared.myDebugStatement == true)
		{
			ImVec4 col = ImColor(0, 255, 0);
			ImGui::PushStyleColor(ImGuiCol_Text, col);
			ImGui::Text("Debug True");
			ImGui::PopStyleColor();
		}
		else
		{
			ImVec4 col = ImColor(255, 0, 0);
			ImGui::PushStyleColor(ImGuiCol_Text, col);
			ImGui::Text("Debug False");
			ImGui::PopStyleColor();
		}
	}

	ImGui::NextColumn();
	
	if (ImGui::Button("Enable")) pHSL->SlingEnable();
	ImGui::SameLine();
	if (ImGui::Button("Disable")) pHSL->SlingDisable();
	ImGui::SameLine();
	if (ImGui::Button("Reset")) pHSL->SlingReset();
	//ImGui::SameLine();

	if (ImGui::Button("Repair Rope")) pHSL->SlingRepairRope();

	if (ImGui::Button("Place Load Here")) pHSL->CargoPlaceOnGround();
	if (ImGui::Button("Place Load Coords")) pHSL->CargoPlaceCoordinates();

	if (ImGui::Button("Place Fire Here")) pHSL->FirePlaceOnGround();
	if (ImGui::Button("Place Fire Coords")) pHSL->FirePlaceCoordinates();

	if (ImGui::Button("Connect Load")) pHSL->SlingConnect();
	if (ImGui::Button("Release Load")) pHSL->SlingRelease();
	if (ImGui::Button("Cut Rope")) pHSL->myCargoDataShared.myRopeRuptured = true;

	if (ImGui::Button("Fill BambiBucket")) pHSL->myCargo.myBambiBucketWaterLevel = 1.0;
	if (ImGui::Button("Release BambiBucket")) pHSL->BambiBucketRelease();


	
	
	ImGui::InputText("WinchObject", &(pHSL->myWinchPath));
	ImGui::InputText("RopeObject", &(pHSL->myRopePath));
	ImGui::InputText("HookObject", &(pHSL->myHookPath));
	ImGui::InputText("CargoObject", &(pHSL->myCargoPath));
	
	
	if (ImGui::Button("Update Objects")) pHSL->UpdateObjects();

	if (pHSL->myObjectHasAnimation == true)
	{
		ImVec4 col = ImColor(255, 0, 0);
		ImGui::PushStyleColor(ImGuiCol_Text, col);
		ImGui::Text("Error: Incompatible Object");
		ImGui::PopStyleColor();
	}

	if (pHSL->myUpdateObjectError == true)
	{
		ImVec4 col = ImColor(255, 0, 0);
		ImGui::PushStyleColor(ImGuiCol_Text, col);
		ImGui::Text("Error: Failed to Load Objects."); 
		//ImGui::Text("Plugin Disabled");
		ImGui::PopStyleColor();
	}

	if (pHSL->myCargoDataShared.mySlingLineEnabled == true)
	{
		ImVec4 col = ImColor(0, 255, 0);
		ImGui::PushStyleColor(ImGuiCol_Text, col);
		ImGui::Text("Sling Line Enabled");
		ImGui::PopStyleColor();
	}
	else
	{
		ImVec4 col = ImColor(255, 0, 0);
		ImGui::PushStyleColor(ImGuiCol_Text, col);
		ImGui::Text("Sling Line Disabled");
		ImGui::PopStyleColor();
	}

/*
	ImGui::InputInt("Scenery N r", &(pHRM->m_scenery_number), 1, 1);

	if (ImGui::Button("Save All"))
	{
		pHRM->SaveMissions();

	}

	if (ImGui::Button("Read ALL"))
	{
		pHRM->ReadMissions();

	}

	ImGui::Separator();

	static std::vector<HRM_Mission *> *p_mission_vector = &(pHRM->m_street_missions);
	static HRM_Mission * p_mission = NULL;
	static HRM_Mission * p_mission_old = NULL;
	static int selected_mission_type = 0;
	static int mission_listbox_item_current = 0;

	//ImGui::PushFont(font10);
	pHRM->mp_current_mission = NULL;

	if (ImGui::RadioButton("Street Missions", selected_mission_type == 0))
	{
		p_mission_vector = &(pHRM->m_street_missions);
		selected_mission_type = 0;
	}

	if (ImGui::RadioButton("Urban Missions", selected_mission_type == 1))
	{
		p_mission_vector = &(pHRM->m_urban_missions);
		selected_mission_type = 1;
	}

	if (ImGui::RadioButton("SAR Missions", selected_mission_type == 2))
	{
		p_mission_vector = &(pHRM->m_sar_missions);
		selected_mission_type = 2;
	}

	if (ImGui::RadioButton("Sling Missions", selected_mission_type == 3))
	{
		p_mission_vector = &(pHRM->m_sling_missions);
		selected_mission_type = 3;
	}

	if (p_mission_vector->size() > 0)
	{
		const char* mission_listbox_items[1024];


		for (int index = 0; index < p_mission_vector->size(); index++)
		{
			mission_listbox_items[index] = p_mission_vector->at(index)->m_name.c_str();
		}

		ImGui::ListBox("Missions", &mission_listbox_item_current, mission_listbox_items, p_mission_vector->size(), 10);

		if (ImGui::Button("Delete Mission"))
		{
			HRM_Mission *p_del = p_mission_vector->at(mission_listbox_item_current);
			p_mission_vector->erase(p_mission_vector->begin() + mission_listbox_item_current);

			p_del->RemoveMission();

			delete p_del;
			mission_listbox_item_current = 0;
		}

	}

	if (ImGui::Button("New Mission"))
	{
		HRM_Mission *p_new = new HRM_Mission();

		p_mission_vector->push_back(p_new);

		p_new->m_mission_type = selected_mission_type;

	}



	ImGui::NextColumn();

	if (p_mission_vector->size() > 0)
	{
		if (mission_listbox_item_current < p_mission_vector->size())
		{
			p_mission = p_mission_vector->at(mission_listbox_item_current);
			pHRM->mp_current_mission = p_mission;

			p_mission->SetPosition(pHRM->m_ld_latitude, pHRM->m_ld_longitude, pHRM->m_lf_heading);

			if ((p_mission != p_mission_old) && (p_mission_old != NULL)) p_mission_old->RemoveMission();
			p_mission_old = p_mission;

			ImGui::InputText("M Name", &(p_mission->m_name));
			ImGui::InputText("M Start", &(p_mission->m_start_text));
			ImGui::InputText("M Pickup", &(p_mission->m_pickup_text));
			ImGui::InputText("M End", &(p_mission->m_end_text));
			ImGui::InputText("M Arr Failed", &(p_mission->m_failed_arr_text));
			ImGui::InputText("M Hosp Failed", &(p_mission->m_failed_hosp_text));

			ImGui::InputInt("S Start", &(p_mission->m_sound_start), 1, 1);
			ImGui::InputInt("S Arr", &(p_mission->m_sound_arr), 1, 1);
			ImGui::InputInt("S Pickup", &(p_mission->m_sound_pickup), 1, 1);
			ImGui::InputInt("S End", &(p_mission->m_sound_end), 1, 1);
			ImGui::InputInt("SF Arr", &(p_mission->m_sound_failed_arr), 1, 1);
			ImGui::InputInt("SF Hosp", &(p_mission->m_sound_failed_hops), 1, 1);

			ImGui::NextColumn();

			const char* object_listbox_items[1024];

			static int object_listbox_item_current = 0;


			for (int index = 0; index < p_mission->m_object_vector.size(); index++)
			{
				object_listbox_items[index] = p_mission->m_object_vector.at(index)->m_obj_path.c_str();
			}

			ImGui::ListBox("Objects", &object_listbox_item_current, object_listbox_items, p_mission->m_object_vector.size(), 10);

			if (ImGui::Button("Delete Object"))
			{
				HRM_Object *p_del = p_mission->m_object_vector.at(object_listbox_item_current);
				p_mission->m_object_vector.erase(p_mission->m_object_vector.begin() + object_listbox_item_current);

				p_del->DestroyInstance();

				delete p_del;
				object_listbox_item_current = 0;
			}

			if (ImGui::Button("New Object"))
			{
				HRM_Object *p_new = new HRM_Object();

				p_mission->m_object_vector.push_back(p_new);

				p_mission->RemoveMission();
				p_mission->DrawMission();

			}

			if (ImGui::Button("Reset Position to ACF"))
			{
				pHRM->UpdatePosition();

				p_mission->RemoveMission();
				p_mission->DrawMission();
			}

			if (ImGui::Button("Redraw Mission"))
			{
				p_mission->RemoveMission();
				p_mission->DrawMission();
			}



			ImGui::NextColumn();

			if (object_listbox_item_current < p_mission->m_object_vector.size())
			{

				HRM_Object *p_HRM_obj = p_mission->m_object_vector.at(object_listbox_item_current);

				if (p_HRM_obj)
				{

					ImGui::InputText("O Path", &(p_HRM_obj->m_obj_path));

					//ImGui::InputDouble("Angle", &(p_HRM_obj->m_zero_angle), 1, 1, 0, 0);
					//ImGui::InputDouble("Dist", &(p_HRM_obj->m_zero_distance), 1, 1, 0, 0);

					ImGui::InputDouble("X [m]:", &(p_HRM_obj->m_dist_x), 1, 1, 0, 0);
					ImGui::InputDouble("Y [m]:", &(p_HRM_obj->m_dist_y), 1, 1, 0, 0);

					ImGui::InputDouble("Elev", &(p_HRM_obj->m_elevation), 0.1, 0.1, 0, 0);

					ImGui::InputDouble("Heading", &(p_HRM_obj->m_heading), 1, 1, 0, 0);
					ImGui::InputDouble("Pitch", &(p_HRM_obj->m_pitch), 1, 1, 0, 0);
					ImGui::InputDouble("Roll", &(p_HRM_obj->m_roll), 1, 1, 0, 0);

					ImGui::Checkbox("IsPatient", &(p_HRM_obj->m_is_patient));

					if (ImGui::Button("Set Pos", ImVec2(180, 20)))
					{
						p_HRM_obj->SetPositionCart(pHRM->m_ld_latitude, pHRM->m_ld_longitude, pHRM->m_lf_heading);
					}

				}
			}


		}
	}
	

	/*ImGui::Columns(4, 0, false);

	static std::vector<HRM_Mission *> *p_mission_vector = &(pHRM->m_street_missions);
	static HRM_Mission * p_mission = NULL;
	static HRM_Mission * p_mission_old = NULL;
	static int selected_mission_type = 0;
	static int mission_listbox_item_current = 0;


	if (ImGui::RadioButton("Street Missions", selected_mission_type == 0))
	{
		p_mission_vector = &(pHRM->m_street_missions);
		selected_mission_type = 0;
	}

	if (ImGui::RadioButton("Urban Missions", selected_mission_type == 1))
	{
		p_mission_vector = &(pHRM->m_urban_missions);
		selected_mission_type = 1;
	}

	if (ImGui::RadioButton("SAR Missions", selected_mission_type == 2))
	{
		p_mission_vector = &(pHRM->m_sar_missions);
		selected_mission_type = 2;
	}

	if (ImGui::RadioButton("Sling Missions", selected_mission_type == 3))
	{
		p_mission_vector = &(pHRM->m_sling_missions);
		selected_mission_type = 3;
	}

	if (p_mission_vector->size() > 0)
	{
		const char* mission_listbox_items[1024];
		

		for (int index = 0; index < p_mission_vector->size(); index++)
		{
			mission_listbox_items[index] = p_mission_vector->at(index)->m_name.c_str();
		}

		ImGui::ListBox("Missions", &mission_listbox_item_current, mission_listbox_items, p_mission_vector->size(), 10);

		if (ImGui::Button("Delete Mission"))
		{
			HRM_Mission *p_del = p_mission_vector->at(mission_listbox_item_current);
			p_mission_vector->erase(p_mission_vector->begin() + mission_listbox_item_current);

			p_del->RemoveMission();

			delete p_del;
			mission_listbox_item_current = 0;
		}

	}

	if (ImGui::Button("New Mission"))
	{
		HRM_Mission *p_new = new HRM_Mission();

		p_mission_vector->push_back(p_new);

		p_new->m_mission_type = selected_mission_type;

	}


	ImGui::Spacing();
	ImGui::Separator();
	ImGui::Spacing();

	if (ImGui::Button("Save ALL"))
	{
		pHRM->SaveMissions();

	}

	ImGui::NextColumn();

	if (p_mission_vector->size() > 0)
	{
		if (mission_listbox_item_current < p_mission_vector->size())
		{
			p_mission = p_mission_vector->at(mission_listbox_item_current);

			if ((p_mission != p_mission_old) && (p_mission_old != NULL)) p_mission_old->RemoveMission();
			p_mission_old = p_mission;

			ImGui::InputText("M Name", &(p_mission->m_name));
			ImGui::InputText("M Start", &(p_mission->m_start_text));
			ImGui::InputText("M Pickup", &(p_mission->m_pickup_text));
			ImGui::InputText("M Flight2", &(p_mission->m_flight2_text));
			ImGui::InputText("M End", &(p_mission->m_end_text));
			ImGui::InputText("M Arr Failed", &(p_mission->m_failed_arr_text));
			ImGui::InputText("M Hosp Failed", &(p_mission->m_failed_hosp_text));

			ImGui::InputInt("S Start", &(p_mission->m_sound_start), 1, 1);
			ImGui::InputInt("S Arr", &(p_mission->m_sound_arr), 1, 1);
			ImGui::InputInt("S Pickup", &(p_mission->m_sound_pickup), 1, 1);
			ImGui::InputInt("S End", &(p_mission->m_sound_end), 1, 1);
			ImGui::InputInt("SF Arr", &(p_mission->m_sound_failed_arr), 1, 1);
			ImGui::InputInt("SF Hosp", &(p_mission->m_sound_failed_hops), 1, 1);

			ImGui::NextColumn();

			const char* object_listbox_items[1024];

			static int object_listbox_item_current = 0;


			for (int index = 0; index < p_mission->m_object_vector.size(); index++)
			{
				object_listbox_items[index] = p_mission->m_object_vector.at(index)->m_obj_path.c_str();
			}

			ImGui::ListBox("Objects", &object_listbox_item_current, object_listbox_items, p_mission->m_object_vector.size(), 10);

			if (ImGui::Button("Delete Object"))
			{
				HRM_Object *p_del = p_mission->m_object_vector.at(object_listbox_item_current);
				p_mission->m_object_vector.erase(p_mission->m_object_vector.begin() + object_listbox_item_current);

				p_del->DestroyInstance();

				delete p_del;
				object_listbox_item_current = 0;
			}

			if (ImGui::Button("New Object"))
			{
				HRM_Object *p_new = new HRM_Object();

				p_mission->m_object_vector.push_back(p_new);

				p_mission->RemoveMission();
				p_mission->DrawMission();

			}

			if (ImGui::Button("Reset Position to ACF"))
			{
				pHRM->UpdatePosition();

				p_mission->RemoveMission();
				p_mission->DrawMission();
			}

			if (ImGui::Button("Redraw Mission"))
			{
				p_mission->RemoveMission();
				p_mission->DrawMission();
			}



			ImGui::NextColumn();

			if (object_listbox_item_current < p_mission->m_object_vector.size())
			{

				HRM_Object *p_HRM_obj = p_mission->m_object_vector.at(object_listbox_item_current);

				if (p_HRM_obj)
				{
					
					ImGui::InputText("O Path", &(p_HRM_obj->m_obj_path));

					

					ImGui::InputDouble("X [m]:", &(p_HRM_obj->m_dist_x), 1, 1, 0, 0);
					ImGui::InputDouble("Y [m]:", &(p_HRM_obj->m_dist_y), 1, 1, 0, 0);

					ImGui::InputDouble("Elev", &(p_HRM_obj->m_elevation), 0.1, 0.1, 0, 0);

					ImGui::InputDouble("Heading", &(p_HRM_obj->m_heading), 1, 1, 0, 0);
					ImGui::InputDouble("Pitch", &(p_HRM_obj->m_pitch), 1, 1, 0, 0);
					ImGui::InputDouble("Roll", &(p_HRM_obj->m_roll), 1, 1, 0, 0);

					if (ImGui::Button("Set Pos", ImVec2(180, 20)))
					{
						p_HRM_obj->SetPositionCart(pHRM->m_ld_latitude, pHRM->m_ld_longitude, pHRM->m_lf_heading);
					}

				}
			}


		}
	}
	




	/*
	const char* object_listbox_items[1024];

	static int object_listbox_item_current = 0;
	

	for (int index = 0; index < pHRM->m_object_vector.size(); index++)
	{
		object_listbox_items[index] = pHRM->m_object_vector.at(index)->m_obj_path.c_str();
	}

	ImGui::ListBox("listbox\n(single select)", &object_listbox_item_current, object_listbox_items, pHRM->m_object_vector.size(), 10);
	
	if (object_listbox_item_current >= pHRM->m_object_vector.size()) return;

	HRM_Object *p_HRM_obj = pHRM->m_object_vector.at(object_listbox_item_current);
	
	win_width = ImGui::GetWindowWidth();
	win_height = ImGui::GetWindowHeight();

	ImGui::Text("Hello World NR 2");

	if (p_HRM_obj)
	{
		ImGui::Text(p_HRM_obj->m_obj_path.c_str());
		ImGui::InputDouble("Angle", &(p_HRM_obj->m_zero_angle), 1, 1, 0, 0);
		ImGui::InputDouble("Dist", &(p_HRM_obj->m_zero_distance), 1, 1, 0, 0);
		ImGui::InputDouble("Elev", &(p_HRM_obj->m_elevation), 0.1, 0.1, 0, 0);

		ImGui::InputDouble("Heading", &(p_HRM_obj->m_heading), 1, 1, 0, 0);
		ImGui::InputDouble("Pitch", &(p_HRM_obj->m_pitch), 1, 1, 0, 0);
		ImGui::InputDouble("Roll", &(p_HRM_obj->m_roll), 1, 1, 0, 0);

		if (ImGui::Button("Set Pos", ImVec2(180, 20)))
		{
			p_HRM_obj->SetPosition(pHRM->m_ld_latitude, pHRM->m_ld_longitude, pHRM->m_lf_heading);
		}

	}*/

	

	/*ImGui::Spacing();
	ImGui::Columns(6, 0, false);
	ImGui::PushStyleVar(ImGuiStyleVar_FrameRounding, 5.f);
	if (ImGui::Button("MyFlight", ImVec2(150, 20))) selected_tab = 1;
	ImGui::NextColumn();
	if (ImGui::Button("Graphs", ImVec2(150, 20))) selected_tab = 2;
	ImGui::NextColumn();
	if (ImGui::Button("Areas of Concern", ImVec2(150, 20))) selected_tab = 3;
	ImGui::NextColumn();
	if (ImGui::Button("Logbook", ImVec2(150, 20))) selected_tab = 4;
	ImGui::NextColumn();
	if (ImGui::Button("Aircraft Configuration", ImVec2(180, 20))) selected_tab = 6;
	ImGui::NextColumn();
	if (ImGui::Button("Settings", ImVec2(150, 20))) selected_tab = 5;
	ImGui::Columns(1);
	ImGui::PopStyleVar();
	ImGui::Separator();
	if (selected_tab == 1) {
		ImGui::Columns(2, 0, true);
		ImGui::SetColumnWidth(-1, 150);
		ImGui::Text("Call-Outs:");
		//for (int i = 0; i < (cols - 1); i++) ImGui::NextColumn();
		ImGui::Checkbox(" 60 kt", &(pHRM->m_ivyConfig->m_kt60_enabled));
		ImGui::Checkbox(" 80 kt", &(pHRM->m_ivyConfig->m_kt80_enabled));
		ImGui::Checkbox("100 kt", &(pHRM->m_ivyConfig->m_kt100_enabled));

		//ImGui::NextColumn();
		ImGui::Text(" ");
		ImGui::InputInt("V1", &(pHRM->m_ivyAircraft->m_li_v1), 1, 1);
		
		
		ImGui::InputInt("VR", &(pHRM->m_ivyAircraft->m_li_vr), 1, 1);
		
		
		
		ImGui::InputInt("V2", &(pHRM->m_ivyAircraft->m_li_v2), 1, 1);

		

		output_dh = XPLMGetDataf(pHRM->m_f_decision_height);
		
		//output_dh = XPLMGetDataf(pHRM->m_f_decision_height);
		ImGui::InputDouble("DH", &output_dh,1,1,0,0);
		XPLMSetDataf(pHRM->m_f_decision_height, output_dh);
		
		ImGui::NextColumn();
		//const char* listbox_items[1000];

		ImGui::Text("Errors :");

		for (auto line : pHRM->m_error_list)
		{
			ImGui::Text(line.c_str());
		}
		
		//static int listbox_item_current = 0;
		//listbox_item_current = 0;
		
		//ImGui::ListBox("", &listbox_item_current, listbox_items, IM_ARRAYSIZE(listbox_items), 20);
	}

	// Show Graphs
	if (selected_tab == 2)
	{
		ImGui::Columns(2, 0, true);
		ImGui::SetColumnWidth(-1, 200);
		ImGui::Spacing();

		if (ImGui::Button("Altitude", ImVec2(180, 20))) selected_graph = 1;
		if (ImGui::Button("Climb Rate", ImVec2(180, 20))) selected_graph = 2;
		if (ImGui::Button("G-Forces Vertical", ImVec2(180, 20))) selected_graph = 3;
		if (ImGui::Button("G-Forces Forward", ImVec2(180, 20))) selected_graph = 4;
		if (ImGui::Button("G-Forces Side", ImVec2(180, 20))) selected_graph = 5;

		ImGui::NextColumn();

		// Altitude
		if (selected_graph == 1)
		{
			double *p_values = pHRM->m_graph_altitude;

			double max_value = 0;
			double min_value = p_values[0];
			for (int index = 0; index < MAX_GRAPH_DATA; index++)
			{
				if (max_value < p_values[index])
					max_value = p_values[index];

				if (min_value > p_values[index])
					min_value = p_values[index];
			}

			ImGui::PlotLines("", p_values, MAX_GRAPH_DATA, 0, "Altitude", min_value - 1, max_value + 1, ImVec2(win_width - 300, win_height - 100));

			std::string text = std::to_string((int)max_value) + " ft";
			ImGui::SetCursorScreenPos(ImVec2(win_width - 90, 40));
			ImGui::Text(text.c_str());

			text = std::to_string((int)min_value) + " ft";
			ImGui::SetCursorScreenPos(ImVec2(win_width - 90, win_height - 75));
			ImGui::Text(text.c_str());
		}
		// Climb Rate
		else if (selected_graph == 2)
		{
			double *p_values = pHRM->m_graph_climb;

			double max_value = 0;
			double min_value = p_values[0];

			double abs_max = 0;

			for (int index = 0; index < MAX_GRAPH_DATA; index++)
			{
				if (max_value < p_values[index])
					max_value = p_values[index];

				if (min_value > p_values[index])
					min_value = p_values[index];
			}

			abs_max = abs(max_value);
			if (abs_max < abs(min_value))
				abs_max = abs(min_value);

			max_value = abs_max;
			min_value = -1 * abs_max;

			ImGui::PlotLines("", p_values, MAX_GRAPH_DATA, 0, "Climb Rate", min_value - 1, max_value + 1, ImVec2(win_width - 300, win_height - 100));

			std::string text = std::to_string((int)max_value) + " ft/min";
			ImGui::SetCursorScreenPos(ImVec2(win_width - 90, 40));
			ImGui::Text(text.c_str());

			text = std::to_string((int)min_value) + " ft/min";
			ImGui::SetCursorScreenPos(ImVec2(win_width - 90, win_height - 75));
			ImGui::Text(text.c_str());
		}
		// G Vertical
		else if (selected_graph == 3)
		{
			double *p_values = pHRM->m_graph_g_vert;

			double max_value = 0;
			double min_value = p_values[0];

			double abs_max = 0;

			for (int index = 0; index < MAX_GRAPH_DATA; index++)
			{
				if (max_value < p_values[index])
					max_value = p_values[index];

				if (min_value > p_values[index])
					min_value = p_values[index];
			}

			abs_max = abs(max_value);
			if (abs_max < abs(min_value))
				abs_max = abs(min_value);

			max_value = abs_max + 0.1;
			min_value = -1 * abs_max - 0.1;

			ImGui::PlotLines("", p_values, MAX_GRAPH_DATA, 0, "G Forces Vertical", min_value, max_value, ImVec2(win_width - 300, win_height - 100));

			std::stringstream text;
			text << std::fixed << std::setprecision(2) << max_value << " g";
			ImGui::SetCursorScreenPos(ImVec2(win_width - 90, 40));
			ImGui::Text(text.str().c_str());

			text.str("");
			text << std::fixed << std::setprecision(2) << min_value << " g";
			ImGui::SetCursorScreenPos(ImVec2(win_width - 90, win_height - 75));
			ImGui::Text(text.str().c_str());
		}
		// Forward G
		else if (selected_graph == 4)
		{
			double *p_values = pHRM->m_graph_g_horiz;

			double max_value = 0;
			double min_value = p_values[0];

			double abs_max = 0;

			for (int index = 0; index < MAX_GRAPH_DATA; index++)
			{
				if (max_value < p_values[index])
					max_value = p_values[index];

				if (min_value > p_values[index])
					min_value = p_values[index];
			}

			abs_max = abs(max_value);
			if (abs_max < abs(min_value))
				abs_max = abs(min_value);

			max_value = abs_max + 0.1;
			min_value = -1 * abs_max - 0.1;

			ImGui::PlotLines("", p_values, MAX_GRAPH_DATA, 0, "G Forces Forward", min_value, max_value, ImVec2(win_width - 300, win_height - 100));

			std::stringstream text;
			text << std::fixed << std::setprecision(2) << max_value << " g";
			ImGui::SetCursorScreenPos(ImVec2(win_width - 90, 40));
			ImGui::Text(text.str().c_str());

			text.str("");
			text << std::fixed << std::setprecision(2) << min_value << " g";
			ImGui::SetCursorScreenPos(ImVec2(win_width - 90, win_height - 75));
			ImGui::Text(text.str().c_str());
		}
		// Side G
		else if (selected_graph == 5)
		{
			double *p_values = pHRM->m_graph_g_side;

			double max_value = 0;
			double min_value = p_values[0];

			double abs_max = 0;

			for (int index = 0; index < MAX_GRAPH_DATA; index++)
			{
				if (max_value < p_values[index])
					max_value = p_values[index];

				if (min_value > p_values[index])
					min_value = p_values[index];
			}

			abs_max = abs(max_value);
			if (abs_max < abs(min_value))
				abs_max = abs(min_value);

			max_value = abs_max + 0.1;
			min_value = -1 * abs_max - 0.1;

			ImGui::PlotLines("", p_values, MAX_GRAPH_DATA, 0, "G Forces Sideways", min_value, max_value, ImVec2(win_width - 300, win_height - 100));

			std::stringstream text;
			text << std::fixed << std::setprecision(2) << max_value << " g left";
			ImGui::SetCursorScreenPos(ImVec2(win_width - 90, 40));
			ImGui::Text(text.str().c_str());

			text.str("");
			text << std::fixed << std::setprecision(2) << min_value << " g right";
			ImGui::SetCursorScreenPos(ImVec2(win_width - 90, win_height - 75));
			ImGui::Text(text.str().c_str());
		}



	}
	// Logbook
	if (selected_tab == 4)
	{
		for (auto line : boost::adaptors::reverse(pHRM->m_logbook_entries))
		{
			ImGui::Text(line.c_str());
		}
		
	}

	// Areas of Concern
	if (selected_tab == 3)
	{
		ImGui::Columns(2, 0, true);
		ImGui::SetColumnWidth(-1, 350);

		ImGui::Text("Areas of Concern:");
		ImGui::Separator();

		ImVec4 col = ImColor(255, 0, 0);
		ImGui::PushStyleColor(ImGuiCol_Text, col);
		for (auto ivyObj : pHRM->m_aoc_list_high)
			ImGui::Text(ivyObj->m_error_string.c_str());
		ImGui::PopStyleColor();

		col = ImColor(255, 255, 0);
		ImGui::PushStyleColor(ImGuiCol_Text, col);
		for (auto ivyObj : pHRM->m_aoc_list_med)
			ImGui::Text(ivyObj->m_error_string.c_str());
		ImGui::PopStyleColor();

		col = ImColor(0, 255, 0);
		ImGui::PushStyleColor(ImGuiCol_Text, col);
		for (auto ivyObj : pHRM->m_aoc_list_low)
			ImGui::Text(ivyObj->m_error_string.c_str());
		ImGui::PopStyleColor();

		ImGui::NextColumn();

		ImGui::PlotHistogram("", pHRM->m_ivy_error_count_history, MAX_ERROR_HISTORY, 0, "Number of Errors per Flight", 0, pHRM->m_ivy_error_count_history_max, ImVec2(win_width - 400, win_height - 100));
		ImGui::SetCursorPosX(362);
		ImGui::Text("0");
		ImGui::SameLine();
		ImGui::SetCursorPosX(win_width - 60);
		ImGui::Text(std::to_string(MAX_ERROR_HISTORY - 1).c_str());



	}

	// Settings
	if (selected_tab == 5)
	{
		ImGui::Columns(3, 0, true);
		ImGui::SetColumnWidth(-1, 350);
		ImGui::Text("Audio Settings:");

		ImGui::Spacing();

		for (int index = 0; index < pHRM->m_ivyConfig->m_audio_names.size(); index++)
		{
			if (ImGui::RadioButton(pHRM->m_ivyConfig->m_audio_names[index].c_str(), selected_radio == index)) {
				selected_radio = index;
			}
		}
		ImGui::Spacing();

		if (ImGui::Button("Apply Audio Settings", ImVec2(250, 20)))
		{
			pHRM->m_ivyConfig->m_mp3_dir = pHRM->m_ivyConfig->m_audio_dirs[selected_radio];
			pHRM->m_ivyConfig->SetAudioDirectory();
			pHRM->IvyLoadSoundFiles(true);
		}

		ImGui::NextColumn();

		ImGui::Checkbox("Enable Ivy", &(pHRM->m_ivyConfig->m_ivy_enable));
		ImGui::Checkbox("Enable Callouts", &(pHRM->m_ivyConfig->m_callouts_enable));
		ImGui::Checkbox("Enable Errors", &(pHRM->m_ivyConfig->m_errors_enable));
		ImGui::Checkbox("Enable Pre-Warnings", &(pHRM->m_ivyConfig->m_pre_warnings));
		ImGui::Checkbox("Enable Ouch", &(pHRM->m_ivyConfig->m_ouch_enabled));

		ImGui::Spacing();

		ImGui::Checkbox("Enable Screaming", &(pHRM->m_ivyConfig->m_passengers_screaming));
		ImGui::Checkbox("Enable Applause", &(pHRM->m_ivyConfig->m_passengers_applause));

		//ImGui::Checkbox("Enable Errors", &(pHRM->m_ivyConfig->m_p));


	}

	// Aircraft Configuration
	if (selected_tab == 6)
	{
		if (pHRM->m_ivyAircraft->m_aircraft_number == 0)
		{
			static bool create_failed = false;

			ImGui::Spacing();
			ImGui::Text("Unconfigured Aircraft");

			if (ImGui::Button("Create New Aircraft Configuration", ImVec2(250, 20)))
			{
				MyIvyConfigAircraft *p_ivyAircraft = new MyIvyConfigAircraft(pHRM->m_ivyConfig->m_config_path.c_str(),0, pHRM->m_ls_acf_descrip);

				if (p_ivyAircraft->m_aircraft_number == 0)
				{
					delete p_ivyAircraft;
					create_failed = true;
				}
				else
				{
					pHRM->m_ivy_aircraft_list->push_back(p_ivyAircraft);
					p_ivyAircraft->WriteConfigFile();
					pHRM->m_ivyAircraft = p_ivyAircraft;
				}
			}

			if (create_failed)
			{
				ImGui::Spacing();
				ImGui::Text("Aircraft Creation Failed");
			}
		}
		else
		{
			ImGui::Columns(3, 0, true);
			ImGui::SetColumnWidth(-1, 300);
			ImGui::Spacing();
			ImGui::Text("Aircraft Configuration #");
			ImGui::SameLine();
			ImGui::Text(std::to_string(pHRM->m_ivyAircraft->m_aircraft_number).c_str());
			ImGui::Text("Aircraft Name: ");
			
			ImGui::TextWrapped(pHRM->m_ivyAircraft->m_name);
			//ImGui::Separator();
			ImGui::Spacing();
			

			ImGui::InputInt("V1 Static         ", &(pHRM->m_ivyAircraft->m_li_v1),1,1);
			ImGui::InputInt("VR Static         ", &(pHRM->m_ivyAircraft->m_li_vr), 1, 1);
			ImGui::InputInt("V2 Static         ", &(pHRM->m_ivyAircraft->m_li_v2), 1, 1);

			ImGui::Checkbox("Dynamic V-Speeds", &(pHRM->m_ivyAircraft->m_vspeeds_enabled));
			ImGui::InputText("V1 Dataref", &(pHRM->m_ivyAircraft->m_lx_v1_data_ref));
			ImGui::InputText("VR Dataref", &(pHRM->m_ivyAircraft->m_lx_vr_data_ref));
			ImGui::InputText("V2 Dataref", &(pHRM->m_ivyAircraft->m_lx_v2_data_ref));

			ImGui::PushItemWidth(200);

			if (ImGui::Button("Update Datarefs",ImVec2(280,25)))
			{
				pHRM->m_ivyAircraft->InitDataRefs();
			}

			if (ImGui::Button("Write Configuration File", ImVec2(280, 25)))
			{
				pHRM->m_ivyAircraft->WriteConfigFile();
			}
			ImGui::PopItemWidth();

			ImGui::NextColumn();

			ImGui::SetColumnWidth(-1, 500);
			
			
			ImGui::PushItemWidth(100);
			ImGui::Checkbox("Enable Slats", &(pHRM->m_ivyAircraft->m_slats_enabled));
			//ImGui::SameLine();
			ImGui::InputDouble("Slats Tolerance", &(pHRM->m_ivyAircraft->m_slats_tolerance), 0.01, 0.01, 2, 0);
			ImGui::SameLine();
			ImGui::InputDouble("Slats Value", &(pHRM->m_ivyAircraft->m_lf_slats), 0.01, 0.01, 2, ImGuiInputTextFlags_ReadOnly);

			ImGui::PopItemWidth();
			ImGui::PushItemWidth(350);
			ImGui::InputText("SDataRef", &(pHRM->m_ivyAircraft->m_lf_slats_data_ref));
			ImGui::PopItemWidth();
			
			//ImGui::Spacing();
						
			for (int index = 0; index < IVY_FS_MAX; index++)
			{
				char buffer[1024];

				ImGui::PushItemWidth(100);
				sprintf(buffer, "Pos S%2i", index + 1);
				ImGui::InputInt(buffer, &(pHRM->m_ivyAircraft->m_slats_deploy_pos[index]), 1, 1, 0);

				ImGui::SameLine();

				sprintf(buffer, "Val S%2i", index + 1);
				ImGui::InputDouble(buffer, &(pHRM->m_ivyAircraft->m_slats_deploy_value[index]), 0.01, 0.01, "%.2f", ImGuiInputTextFlags_ReadOnly );
				ImGui::SameLine();

				ImGui::PopItemWidth();
				
				sprintf(buffer, "SET S%2i", index + 1);
				if (ImGui::Button(buffer)) pHRM->m_ivyAircraft->m_slats_deploy_value[index] = pHRM->m_ivyAircraft->m_lf_slats;
				ImGui::SameLine();
				sprintf(buffer, "DEL S%2i", index + 1);
				if (ImGui::Button(buffer)) pHRM->m_ivyAircraft->m_slats_deploy_value[index] = -100;

				
			}
			



			ImGui::NextColumn();

			ImGui::SetColumnWidth(-1, 500);

			ImGui::PushItemWidth(100);
			ImGui::Checkbox("Enable Flaps", &(pHRM->m_ivyAircraft->m_flaps_enabled));
			//ImGui::SameLine();
			ImGui::InputDouble("Flaps Tolerance", &(pHRM->m_ivyAircraft->m_flaps_tolerance), 0.01, 0.01, 2, 0);
			ImGui::SameLine();
			ImGui::InputDouble("Flaps Value", &(pHRM->m_ivyAircraft->m_lf_flaps), 0.01, 0.01, 2, ImGuiInputTextFlags_ReadOnly);

			ImGui::PopItemWidth();
			ImGui::PushItemWidth(350);
			ImGui::InputText("FDataRef", &(pHRM->m_ivyAircraft->m_lf_flaps_data_ref));
			ImGui::PopItemWidth();

			//ImGui::Spacing();

			for (int index = 0; index < IVY_FS_MAX; index++)
			{
				char buffer[1024];

				ImGui::PushItemWidth(100);
				sprintf(buffer, "Pos F%2i", index + 1);
				ImGui::InputInt(buffer, &(pHRM->m_ivyAircraft->m_flaps_deploy_pos[index]), 1, 1, 0);

				ImGui::SameLine();

				sprintf(buffer, "F Val #%2i", index + 1);
				ImGui::InputDouble(buffer, &(pHRM->m_ivyAircraft->m_flaps_deploy_value[index]), 0.01, 0.01, "%.2f", ImGuiInputTextFlags_ReadOnly);
				ImGui::SameLine();

				ImGui::PopItemWidth();

				sprintf(buffer, "SET F%2i", index + 1);
				if (ImGui::Button(buffer)) pHRM->m_ivyAircraft->m_flaps_deploy_value[index] = pHRM->m_ivyAircraft->m_lf_flaps;
				ImGui::SameLine();
				sprintf(buffer, "DEL F%2i", index + 1);
				if (ImGui::Button(buffer)) pHRM->m_ivyAircraft->m_flaps_deploy_value[index] = -100;


			}

			

			
			
		}
	}
	*/
	
	//ImGui::TextUnformatted("Hello, World!");

	//ImGui::Text("Window size: width = %f  height = %f", win_width, win_height);

	//ImGui::TextUnformatted("Two Widgets");
}




