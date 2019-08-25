#pragma once
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <list>
#include <sys/stat.h>
#include <iostream>
#include <sstream>
#include <stdlib.h>
#include <time.h>
#include <vector>
#include <algorithm>
#include <queue>
#include <fstream>
#include <chrono>
#include <ctime>
#include <iomanip>
#include <set>

#include "XPLMDefs.h"
#include "XPLMProcessing.h"
#include "XPLMDisplay.h"
#include "XPLMGraphics.h"
#include "XPLMDataAccess.h"
#include "XPLMUtilities.h"
#include "XPLMNavigation.h"
#include "XPLMPlugin.h"
#include "XPLMMenus.h"
#include "XPWidgetDefs.h"
#include "XPWidgets.h"
#include "XPStandardWidgets.h"
#include "XPWidgetUtils.h"
#include "XPLMInstance.h"

#include "ImgWindow.h"
#include "imgui.h"

#define _USE_MATH_DEFINES
#include <math.h>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/property_tree/ini_parser.hpp>

// OS X: we use this to convert our file path.
#if APL
#include <Carbon/Carbon.h>
#endif

// Your include paths for OpenAL may vary by platform.
//#include "al.h"
//#include "alc.h"



#define IVY_MAX_AIRCRAFT_CONFIG 100

#define MAX_ERROR_HISTORY 100

#define MAX_GRAPH_DATA 1000
#define MAX_GRAPH_TIME 100

#define VERT_AXIS 1

#define HSL_ROPE_POINTS_MAX 2000



class HSL_PlugIn;
extern HSL_PlugIn* pHSL;
extern std::ofstream hsl_output_file;

inline bool file_exists(const std::string& name) 
{
	struct stat buffer;
	return (stat(name.c_str(), &buffer) == 0);
}

inline bool is_int(float f)
{
	if (f == ((int)f)) return true;
	return false;
}

inline void HSLDebugString(std::string output)
{
	hsl_output_file << output << std::endl;
	hsl_output_file.flush();
}

static void load_cb(const char * real_path, void * ref)
{
	XPLMObjectRef * dest = (XPLMObjectRef *)ref;
	if (*dest == NULL)
	{
		*dest = XPLMLoadObject(real_path);
	}
}

namespace HRM
{
	const int wp_code = 28;
	const int max_scenery = 100;

	const float patient_weight = 75;
	const float crew_weight = 225;
	const float ems_equippment_weight = 350;

	const int pickup_max_distance = 100;
	const int hospital_max_distance = 500;

	const float fse_min_park_brake = 0.5;

	const float coord_invalid = -10000;

	const int type_sling = 3;

	enum Mission_State
	{
		State_Create_Mission,
		State_Plan_Flight,
		State_Pre_Flight,
		State_Flight_1,
		State_At_Patient,
		State_Flight_2,
		State_At_Hospital,
		State_Mission_Finished,
		State_Mission_Cancelled
	};

	enum Difficulty
	{
		Easy,
		Normal,
		Hard
	};

	enum Scenario_Position
	{
		Scenairo_Aircraft,
		Scenario_ICAO
	};

	const float preflight_time_easy = 300;
	const float preflight_time_normal = 150;
	const float preflight_time_hard = 90;

	const float flight_time_up_down_easy = 60;
	const float flight_time_up_down_normal = 45;
	const float flight_time_up_down_hard = 30;

	const float flight_time_per_nm_easy = 60;
	const float flight_time_per_nm_normal = 45;
	const float flight_time_per_nm_hard = 30;

	const float flight_time_search_easy = 300;
	const float flight_time_search_normal = 180;
	const float flight_time_search_hard = 150;

	const float flight_time_sling_easy = 600;
	const float flight_time_sling_normal = 300;
	const float flight_time_sling_hard = 150;

	//const float threshold_g_mult_easy = 1;
	//const float threshold_g_mult_normal = 1;
	//const float threshold_g_mult_hard = 1;

	//const float threshold_g_mult_flight1 = 1.2;

	const float threshold_g_mult_flight2 = 1;

	const float threshold_gf_low = 0.2;
	const float threshold_gf_med = 0.35;
	const float threshold_gf_high = 0.5;

	const float threshold_gs_low = 0.15;
	const float threshold_gs_med = 0.35;
	const float threshold_gs_high = 0.5;

	const float threshold_gv_pos_low = 1.2;
	const float threshold_gv_pos_med = 1.5;
	const float threshold_gv_pos_high = 1.8;

	const float threshold_gv_neg_low = 0;
	const float threshold_gv_neg_med = 0;
	const float threshold_gv_neg_high = 0;

	const int points_speed_flight1 = 25;
	const int points_speed_flight2 = 25;
	const int points_g_flight2 = 50;

	const float eval_g_total_factor = 1;

	const float eval_g_low_factor = 1;
	const float eval_g_med_factor = 2;
	const float eval_g_high_factor = 3;

	const float eval_flight1_nominal_speed = 120;
	const float eval_flight2_nominal_speed = 120;
	const float eval_flight2_sling_nominal_speed = 50;




}

namespace HSL {
	enum WinchDirection
	{
		Up,
		Down,
		Stop
	};
}
double calc_distance_m(double lat1, double long1, double lat2, double long2);
double calc_distance_nm(double lat1, double long1, double lat2, double long2);


int WrapVSpeedHandler(XPWidgetMessage  inMessage, XPWidgetID  inWidget, intptr_t inParam1, intptr_t inParam2);
int WrapLogbookHandler(XPWidgetMessage  inMessage, XPWidgetID  inWidget, intptr_t  inParam1, intptr_t  inParam2);
int WrapLogbookScrollHandler(XPWidgetMessage  inMessage, XPWidgetID  inWidget, intptr_t inParam1, intptr_t inParam2);


void WrapDrawOutputWindow(XPLMWindowID in_window_id, void * in_refcon);
void WrapMenuHandler(void * in_menu_ref, void * in_item_ref);
int WrapSayBaroCallback(XPLMCommandRef cmd, XPLMCommandPhase phase, void *refcon);
int WrapSayWindCallback(XPLMCommandRef cmd, XPLMCommandPhase phase, void *refcon);
int WrapAnnouncementCallback(XPLMCommandRef cmd, XPLMCommandPhase phase, void *refcon);
int WrapResetHRMCallback(XPLMCommandRef cmd, XPLMCommandPhase phase, void *refcon);
int WrapToogleWindowCallback(XPLMCommandRef cmd, XPLMCommandPhase phase, void *refcon);



int WrapWinchUpCallback(XPLMCommandRef cmd, XPLMCommandPhase phase, void* refcon);
int WrapWinchDownCallback(XPLMCommandRef cmd, XPLMCommandPhase phase, void* refcon);
int WrapWinchStopCallback(XPLMCommandRef cmd, XPLMCommandPhase phase, void* refcon);
int WrapEnableCallback(XPLMCommandRef cmd, XPLMCommandPhase phase, void* refcon);
int WrapDisableCallback(XPLMCommandRef cmd, XPLMCommandPhase phase, void* refcon);
int WrapResetCallback(XPLMCommandRef cmd, XPLMCommandPhase phase, void* refcon);
int WrapConnectLoadCallback(XPLMCommandRef cmd, XPLMCommandPhase phase, void* refcon);
int WrapReleaseLoadCallback(XPLMCommandRef cmd, XPLMCommandPhase phase, void* refcon);
int WrapUpdateParametersCallback(XPLMCommandRef cmd, XPLMCommandPhase phase, void* refcon);

int WrapDrawCallback(XPLMDrawingPhase inPhase, int inIsBefore, void* inRefcon);

void WrapKeyCallback(XPLMWindowID inWindowID, char inKey, XPLMKeyFlags inFlags, char inVirtualKey, void * inRefcon, int losingFocus);
int WrapMouseClickCallback(XPLMWindowID inWindowID, int x, int y, XPLMMouseStatus inMouse, void * inRefcon);

PLUGIN_API float WrapFlightLoopCallback(float elapsedMe, float elapsedSim, int counter, void * refcon);