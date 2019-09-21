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

void HSLImguiWidget::InputVector(vector<float>& vectorIn, std::string nameIn)
{
	for (int i = 0; i < vectorIn.size(); i++)
	{
		std::string name = nameIn + std::to_string(i + 1);
		//float* var = &(vectorIn(i));
		ImGui::InputFloat(name.c_str(), &(vectorIn(i)), 0.01, 0.01, 2, 0);
	}
}

void HSLImguiWidget::OutputVector(vector<float>& vectorOut, std::string nameOut)
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
	win_width = ImGui::GetWindowWidth();
	win_height = ImGui::GetWindowHeight();

	ImGui::Columns(4, 0, false);


	
	ImGui::PushItemWidth(100);

	ImGui::Checkbox("Physics Enabled", &(pHSL->myPhysicsEnabled));

	if (pHSL->myPhysicsEnabled == false)
	{
		ImGui::Text("Object:");
		InputVector(pHSL->myCargo.myVectorPosition, "Cargo Position");
		InputVector(pHSL->myHook.myVectorPosition, "Hook Position");
	}
	
	ImGui::Text("Winch:");
	InputVector(pHSL->myVectorWinchPosition, "Winch Position");
	if (ImGui::Button("Write Aircraft Ini File")) pHSL->AircraftConfigSave();

	ImGui::Text("Set Cargo Coordinates:");
	ImGui::InputDouble("Latitude", &(pHSL->myCargoSetLatitutde), 0.01, 0.01, "%.9f");
	ImGui::InputDouble("Longitude", &(pHSL->myCargoSetLongitude), 0.01, 0.01, "%.9f");

	ImGui::Text("Rope Parameters:");
	ImGui::InputFloat("Rope Length Start [m]", &(pHSL->myRopeLengthStart), 0.01, 0.01, 3, 0);
	ImGui::InputFloat("Rope Length [m]", &(pHSL->myRopeLengthNormal), 0.01, 0.01, 3, 0);
	ImGui::InputFloat("Rope Rupture Force [N]", &(pHSL->myRopeRuptureForce), 0.01, 0.01, 3, 0);
	ImGui::InputFloat("Rope Damping", &(pHSL->myRopeDamping), 0.01, 0.01, 3, 0);
	ImGui::InputFloat("Rope K", &(pHSL->myRopeK), 0.01, 0.01, 3, 0);
	ImGui::InputFloat("MaxRopeAcc", &(pHSL->myMaxAccRopeFactor), 0.01, 0.01, 3, 0);
	ImGui::InputFloat("Winch Speed [m/s]", &(pHSL->myWinchSpeed), 0.01, 0.01, 3, 0);

	ImGui::Text("Hook Parameters:");
	ImGui::InputFloat("Hook Mass [kg]", &(pHSL->myHook.myMass), 0.01, 0.01, 3, 0);
	ImGui::InputFloat("Hook Height [m]", &(pHSL->myHook.myHeight), 0.01, 0.01, 3, 0);
	ImGui::InputFloat("Hook Cross Section [m^2]", &(pHSL->myHook.myCrossSection), 0.01, 0.01, 3, 0);
	ImGui::InputFloat("Hook CW Front", &(pHSL->myHook.myCWFront), 0.01, 0.01, 3, 0);
	ImGui::InputFloat("Hook Friction Glide", &(pHSL->myHook.myFrictionGlide), 0.01, 0.01, 3, 0);
	ImGui::InputFloat("Hook Friction Static", &(pHSL->myHook.myFrictionStatic), 0.01, 0.01, 3, 0);

	ImGui::Text("Cargo Parameters:");
	ImGui::InputFloat("Cargo Mass [kg]", &(pHSL->myCargo.myMass), 0.01, 0.01, 3, 0);
	ImGui::InputFloat("Cargo Height [m]", &(pHSL->myCargo.myHeight), 0.01, 0.01, 3, 0);
	ImGui::InputFloat("Cargo Cross Section [m^2]", &(pHSL->myCargo.myCrossSection), 0.01, 0.01, 3, 0);
	ImGui::InputFloat("Cargo CW Front", &(pHSL->myCargo.myCWFront), 0.01, 0.01, 3, 0);
	ImGui::InputFloat("Cargo Friction Glide", &(pHSL->myCargo.myFrictionGlide), 0.01, 0.01, 3, 0);
	ImGui::InputFloat("Cargo Friction Static", &(pHSL->myCargo.myFrictionStatic), 0.01, 0.01, 3, 0);
	InputVector(pHSL->myCargo.myVectorCargoOffset, "Cargo Offset");	
	InputVector(pHSL->myCargo.myVectorCargoRotation, "Cargo Rotation");

	ImGui::NextColumn();
	OutputVector(pHSL->myVectorHookPosition, "Hook Pos");
	OutputVector(pHSL->myVectorRope, "Rope");
	OutputVector(pHSL->myVectorWinchPosition, "WinchPosition");
	OutputVector(pHSL->myCargo.myVectorVelocity, "Cargo:Velocity");
	
	OutputVector(pHSL->myCargo.myVectorPosition, "CargoPosition");
	OutputVector(pHSL->myCargo.myVectorDisplayOffset, "Cargo:Object Offset");
	OutputVector(pHSL->myCargo.myVectorDisplayAngle, "Cargo:Object Angle");
	
	OutputVector(pHSL->myCargo.myVectorForceGravity, "Cargo:ForceGravity");
	OutputVector(pHSL->myCargo.myVectorWindVelocity, "Cargo:WindVelocity");
	OutputVector(pHSL->myCargo.myVectorForceRope, "Cargo:ForceRope");
	OutputVector(pHSL->myCargo.myVectorAirVelocity, "Cargo:AirVelocity");
	OutputVector(pHSL->myCargo.myVectorForceAir, "Cargo:ForceAir");
	OutputVector(pHSL->myCargo.myVectorForceTotal, "Cargo:ForceTotal");
	OutputVector(pHSL->myCargo.myVectorHorizontalVelocity, "Cargo:HorizontalVelocity");
	OutputVector(pHSL->myCargo.myVectorForceFriction, "Cargo:ForceFriction");
	OutputVector(pHSL->myCargo.myVectorAccTotal, "Cargo:AccTotal");
	OutputVector(pHSL->myCargo.myVectorVelocityDelta, "Cargo:VelocityDelta");
	OutputVector(pHSL->myCargo.myVectorForceChopper, "Cargo:ForceChopper");
	OutputVector(pHSL->myCargo.myVectorMomentumChopper, "Cargo:MomentumChopper");

	OutputVector(pHSL->myHook.myVectorPosition, "HookPosition");
	OutputVector(pHSL->myHook.myVectorVelocity, "Hook:Velocity");
	OutputVector(pHSL->myHook.myVectorDisplayOffset, "Hook:Object Offset");
	OutputVector(pHSL->myHook.myVectorDisplayAngle, "Hook:Object Angle");
	
	OutputVector(pHSL->myHook.myVectorForceGravity, "Hook:ForceGravity");
	OutputVector(pHSL->myHook.myVectorWindVelocity, "Hook:WindVelocity");
	OutputVector(pHSL->myHook.myVectorForceRope, "Hook:ForceRope");
	OutputVector(pHSL->myHook.myVectorAirVelocity, "Hook:AirVelocity");
	OutputVector(pHSL->myHook.myVectorForceAir, "Hook:ForceAir");
	OutputVector(pHSL->myHook.myVectorForceTotal, "Hook:ForceTotal");
	OutputVector(pHSL->myHook.myVectorHorizontalVelocity, "Hook:HorizontalVelocity");
	OutputVector(pHSL->myHook.myVectorForceFriction, "Hook:ForceFriction");
	OutputVector(pHSL->myHook.myVectorAccTotal, "Hook:AccTotal");
	OutputVector(pHSL->myHook.myVectorVelocityDelta, "Hook:VelocityDelta");
	OutputVector(pHSL->myHook.myVectorForceChopper, "Hook:ForceChopper");
	OutputVector(pHSL->myHook.myVectorMomentumChopper, "Hook:MomentumChopper");


	ImGui::NextColumn();


	
	ImGui::Text("FrameTime [s]:      %.3f", pHSL->myFrameTime);
	ImGui::Text("FlightLoopTime [us]:%d", pHSL->myProcessingTimeFlightLoop);
	ImGui::Text("DrawTime [us]:      %d", pHSL->myProcessingTimeDrawRoutine);


	ImGui::Text("CurrentRopeLength:  %.3f", pHSL->myCurrentRopeLength);
	ImGui::Text("RopeLength:         %.3f", pHSL->myNewRopeLength);
	ImGui::Text("StretchRelative:    %.3f", pHSL->myRopeStretchRelative);
	ImGui::Text("ForceScalar:        %.3f", pHSL->myRopeForceScalar);
	ImGui::Text("LengthDelta:        %.3f", pHSL->myRopeLengthDelta);
	ImGui::Text("StretchSpeed:       %.3f", pHSL->myRopeStretchSpeed);
	ImGui::Text("CorrectedD:         %.3f", pHSL->myRopeCorrectedD);

	ImGui::Text("RopeRupture         %d",   pHSL->myRopeRuptured);

	ImGui::Text("Cargo:TerrainHit:         %d", pHSL->myCargo.myTerrainHit);
	ImGui::Text("Cargo:ObjectTerrainLevel: %.3f", pHSL->myCargo.myObjectTerrainLevel);
	ImGui::Text("Cargo:AirSpeed:           %.3f", pHSL->myCargo.myAirSpeed);
	ImGui::Text("Cargo:AirResistance:      %.3f", pHSL->myCargo.myAirResistance);
	ImGui::Text("Cargo:SpeedStaticFriction:%.3f", pHSL->myCargo.mySpeedStaticFriction);

	ImGui::Text("Hook:TerrainHit:         %d", pHSL->myHook.myTerrainHit);
	ImGui::Text("Hook:ObjectTerrainLevel: %.3f", pHSL->myHook.myObjectTerrainLevel);
	ImGui::Text("Hook:AirSpeed:           %.3f", pHSL->myHook.myAirSpeed);
	ImGui::Text("Hook:AirResistance:      %.3f", pHSL->myHook.myAirResistance);
	ImGui::Text("Hook:SpeedStaticFriction:%.3f", pHSL->myHook.mySpeedStaticFriction);

	ImGui::Text("Debug1              %f", pHSL->myDebugValue1);
	ImGui::Text("Debug2              %f", pHSL->myDebugValue2);
	ImGui::Text("Debug3              %f", pHSL->myDebugValue3);
	ImGui::Text("Debug4              %f", pHSL->myDebugValue4);

	if (pHSL->myDebugStatement == true)
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

	ImGui::NextColumn();
	
	if (ImGui::Button("Enable")) pHSL->SlingEnable();
	if (ImGui::Button("Disable")) pHSL->SlingDisable();
	if (ImGui::Button("Reset")) pHSL->SlingReset();

	if (ImGui::Button("Place Load Here")) pHSL->CargoPlaceOnGround();
	if (ImGui::Button("Place Load Coords")) pHSL->CargoPlaceCoordinates();

	if (ImGui::Button("Connect Load")) pHSL->SlingConnect();
	if (ImGui::Button("Release Load")) pHSL->SlingRelease();
	if (ImGui::Button("Cut Rope")) pHSL->myRopeRuptured = true;

	
	
	ImGui::InputText("WinchObject", &(pHSL->myWinchPath));
	ImGui::InputText("RopeObject", &(pHSL->myRopePath));
	ImGui::InputText("HookObject", &(pHSL->myHookPath));
	ImGui::InputText("CargoObject", &(pHSL->myCargoPath));
	
	
	if (ImGui::Button("Update Objects")) pHSL->UpdateObjects();

	if (pHSL->mySlingLineEnabled == true)
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
		ImGui::InputFloat("DH", &output_dh,1,1,0,0);
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
			float *p_values = pHRM->m_graph_altitude;

			float max_value = 0;
			float min_value = p_values[0];
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
			float *p_values = pHRM->m_graph_climb;

			float max_value = 0;
			float min_value = p_values[0];

			float abs_max = 0;

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
			float *p_values = pHRM->m_graph_g_vert;

			float max_value = 0;
			float min_value = p_values[0];

			float abs_max = 0;

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
			float *p_values = pHRM->m_graph_g_horiz;

			float max_value = 0;
			float min_value = p_values[0];

			float abs_max = 0;

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
			float *p_values = pHRM->m_graph_g_side;

			float max_value = 0;
			float min_value = p_values[0];

			float abs_max = 0;

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
			ImGui::InputFloat("Slats Tolerance", &(pHRM->m_ivyAircraft->m_slats_tolerance), 0.01, 0.01, 2, 0);
			ImGui::SameLine();
			ImGui::InputFloat("Slats Value", &(pHRM->m_ivyAircraft->m_lf_slats), 0.01, 0.01, 2, ImGuiInputTextFlags_ReadOnly);

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
				ImGui::InputFloat(buffer, &(pHRM->m_ivyAircraft->m_slats_deploy_value[index]), 0.01, 0.01, "%.2f", ImGuiInputTextFlags_ReadOnly );
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
			ImGui::InputFloat("Flaps Tolerance", &(pHRM->m_ivyAircraft->m_flaps_tolerance), 0.01, 0.01, 2, 0);
			ImGui::SameLine();
			ImGui::InputFloat("Flaps Value", &(pHRM->m_ivyAircraft->m_lf_flaps), 0.01, 0.01, 2, ImGuiInputTextFlags_ReadOnly);

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
				ImGui::InputFloat(buffer, &(pHRM->m_ivyAircraft->m_flaps_deploy_value[index]), 0.01, 0.01, "%.2f", ImGuiInputTextFlags_ReadOnly);
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




