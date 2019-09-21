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
#include "XPLMPlanes.h"

#include "ImgWindow.h"
#include "imgui.h"

#define _USE_MATH_DEFINES
#include <math.h>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/io.hpp> 
using namespace boost::numeric::ublas;


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
#define MAX_OBJ_SPEED 300.0f // ~sonic speed

#define MSG_ADD_DATAREF 0x01000000
//#define USE_INSTANCED_DRAWING 0



class HSL_PlugIn;
extern HSL_PlugIn* pHSL;
extern std::ofstream hsl_output_file;

inline vector<float> get_unit_vector(vector<float>& v_in)
{
	vector<float> ret = v_in;
	float length = norm_2(v_in);
	if (length != 0) ret = v_in / length;
	return ret;
}

inline vector<float> cross_product(vector<float>& a, vector<float>& b)
{
	vector<float> ret(3);

	ret(0) = a(1) * b(2) - a(2) * b(1);
	ret(1) = a(2) * b(0) - a(0) * b(2);
	ret(2) = a(0) * b(1) - a(1) * b(0);
	return ret;
}

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

inline void check_nan(float& f)
{
	if (isnan(f)) f = 0;
}

inline void check_nan(vector<float>& vec)
{
	for (int i = 0; i < vec.size(); i++) check_nan(vec(i));
}

inline void limit_max(vector<float>& vec, float max_value)
{
	for (int i = 0; i < vec.size(); i++)
	{
		if (vec(i) > max_value) vec(1) = max_value;
	}
}

inline vector<float> XPlaneCartToSphere(vector<float>& cart)
{
	// Opengl to sphere coordinates																																																																		
	//r = sqrt(x * x + y * y + z * z);
	//theta = atan2(y, x);
	//phi = atan2(sqrt(x * x + y * y), z);

	vector<float> sphere(3);

	/*
	// Using Heading
	sphere(0) = sqrt((cart(0) * cart(0)) + (cart(1) * cart(1)) + (cart(2) * cart(2)));
	sphere(1) = atan2(cart(2), cart(0));
	sphere(2) = acos(cart(1) / sphere(0));

	*/

	// Using Pitch/Roll
	sphere(0) = sqrt((cart(0) * cart(0)) + (cart(1) * cart(1)) + (cart(2) * cart(2)));

	if (sphere(0) == 0.0f)
	{
		sphere(1) = 0;
		sphere(2) = 0;
	}
	else
	{
		sphere(1) = atan2(cart(2), cart(1));// - M_PI / 2.0f); // pitch
		sphere(2) = asin(cart(0) / sphere(0));
	}
	return sphere;
}

inline vector<float> XPlaneSphereToCart(vector<float>& sphere)
{
	//_pos[0] = _r* sin_theta * cos_phi;
	//_pos[1] = _r* sin_theta * sin_phi;
	//_pos[2] = _r* cos_theta;

	vector<float> cart(3);

	cart(0) = sphere(0) * sin(sphere(2));
	cart(1) = sphere(0) * cos(sphere(1)) * cos(sphere(2));
	cart(2) = sphere(0) * sin(sphere(1)) * cos(sphere(2));
	

	return cart;
}

inline void DrawInstanceCreate(XPLMInstanceRef &instanceIn, XPLMObjectRef &objectIn)
{
	if (objectIn == NULL)
	{
		instanceIn = NULL;
	}
	else if (instanceIn == NULL)
	{
		const char* drefs[] = { NULL, NULL };
		instanceIn = XPLMCreateInstance(objectIn, drefs);
	}
}

inline void DrawInstanceDestroy(XPLMInstanceRef& instanceIn)
{
	if (instanceIn != NULL)
	{
		XPLMDestroyInstance(instanceIn);
		instanceIn = NULL;
	}
}

inline void DrawInstanceSetPosition(XPLMInstanceRef& instanceIn, XPLMObjectRef &objectIn, vector<float> &positionInVec, bool instanced_drawing)
{
	if ((instanceIn != NULL) || (instanced_drawing == 0))
	{
		check_nan(positionInVec);

		XPLMDrawInfo_t		drawInfo;
		drawInfo.structSize = sizeof(drawInfo);
		drawInfo.x = positionInVec(0);
		drawInfo.y = positionInVec(1);
		drawInfo.z = positionInVec(2);
		drawInfo.pitch = 0;
		drawInfo.heading = 0;
		drawInfo.roll = 0;
		try
		{
			if (instanced_drawing == true)	XPLMInstanceSetPosition(instanceIn, &drawInfo, NULL);
			else XPLMDrawObjects(objectIn, 1, &drawInfo, 0, 0);
		}
		catch (...)
		{

		}
	}
}

inline void DrawInstanceSetPosition(XPLMInstanceRef& instanceIn, XPLMObjectRef& objectIn, vector<float>& positionInVec, vector<float>& angleInVec, bool instanced_drawing)
{
	if ((instanceIn != NULL) || (instanced_drawing == 0))
	{
		XPLMDrawInfo_t		drawInfo;
		drawInfo.structSize = sizeof(drawInfo);
		drawInfo.x = positionInVec(0);
		drawInfo.y = positionInVec(1);
		drawInfo.z = positionInVec(2);
		drawInfo.pitch = angleInVec(0);
		drawInfo.roll = angleInVec(1);
		drawInfo.heading = angleInVec(2);
		

		try
		{
			if (instanced_drawing == true)	XPLMInstanceSetPosition(instanceIn, &drawInfo, NULL);
			else XPLMDrawObjects(objectIn, 1, &drawInfo, 0, 0);
		}
		catch (...)
		{

		}
	}
}
/*
inline vector<float> AdjustFrameMovement(vector<float> coordsAircraft)
{
	vector<float> world_coords = coordsAircraft;

	//world_coords(0) += myLdLocalX - myLastLocalX;
	//world_coords(1) += myLdLocalY - myLastLocalY;
	//world_coords(2) += myLdLocalZ - myLastLocalZ;

	return world_coords;
}*/

static void load_cb(const char * real_path, void * ref)
{
	XPLMObjectRef * dest = (XPLMObjectRef *)ref;
	if (*dest == NULL)
	{
		*dest = XPLMLoadObject(real_path);
	}
}

namespace HSL_Data
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

	const float density_water = 997;




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
int WrapUpdateObjectCallback(XPLMCommandRef cmd, XPLMCommandPhase phase, void* refcon);

float WrapReadFloatCallback(void* inRefcon);
void WrapWriteFloatCallback(void* inRefcon, float inValue);
double WrapReadDoubleCallback(void* inRefcon);
void WrapWriteDoubleCallback(void* inRefcon, double inValue);

int WrapReadIntCallback(void* inRefcon);
void WrapWriteIntCallback(void* inRefcon, int inValue);
int WrapReadStringCallback(
	void* inRefcon,
	void* outValue,    /* Can be NULL */
	int                  inOffset,
	int                  inMaxLength);

void WrapWriteStringCallback(
	void* inRefcon,
	void* inValue,
	int                  inOffset,
	int                  inLength);

int WrapReadVectorFloatCallback(
	void* inRefcon,
	float* outValues,    /* Can be NULL */
	int                  inOffset,
	int                  inMax);
void WrapWriteVectorFloatCallback(
	void* inRefcon,
	float* inValues,
	int                  inOffset,
	int                  inCount);


int WrapWinchUpCallback(XPLMCommandRef cmd, XPLMCommandPhase phase, void* refcon);
int WrapWinchDownCallback(XPLMCommandRef cmd, XPLMCommandPhase phase, void* refcon);
int WrapWinchStopCallback(XPLMCommandRef cmd, XPLMCommandPhase phase, void* refcon);
int WrapEnableCallback(XPLMCommandRef cmd, XPLMCommandPhase phase, void* refcon);
int WrapDisableCallback(XPLMCommandRef cmd, XPLMCommandPhase phase, void* refcon);
int WrapResetCallback(XPLMCommandRef cmd, XPLMCommandPhase phase, void* refcon);
int WrapConnectLoadCallback(XPLMCommandRef cmd, XPLMCommandPhase phase, void* refcon);
int WrapReleaseLoadCallback(XPLMCommandRef cmd, XPLMCommandPhase phase, void* refcon);
int WrapToggleControlWindowCallback(XPLMCommandRef cmd, XPLMCommandPhase phase, void* refcon);

int WrapDrawCallback(XPLMDrawingPhase inPhase, int inIsBefore, void* inRefcon);

void WrapKeyCallback(XPLMWindowID inWindowID, char inKey, XPLMKeyFlags inFlags, char inVirtualKey, void * inRefcon, int losingFocus);
int WrapMouseClickCallback(XPLMWindowID inWindowID, int x, int y, XPLMMouseStatus inMouse, void * inRefcon);

PLUGIN_API float WrapFlightLoopCallback(float elapsedMe, float elapsedSim, int counter, void * refcon);