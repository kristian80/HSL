#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "XPLMUtilities.h"
#include "XPLMProcessing.h"
#include "XPLMPlugin.h"
#include "HSL_PlugIn.h"
#include "HSL.h"

// OS X: we use this to convert our file path.
#if APL
#include <Carbon/Carbon.h>
#endif






/**************************************************************************************************************
 * Global Variables 
 **************************************************************************************************************/
HSL_PlugIn * pHSL;
std::ofstream hsl_output_file;



// Mac specific: this converts file paths from HFS (which we get from the SDK) to Unix (which the OS wants).
// See this for more info:
//
// http://www.xsquawkbox.net/xpsdk/mediawiki/FilePathsAndMacho

#if APL
int ConvertPath(const char * inPath, char * outPath, int outPathMaxLen) {

	CFStringRef inStr = CFStringCreateWithCString(kCFAllocatorDefault, inPath, kCFStringEncodingMacRoman);
	if (inStr == NULL)
		return -1;
	CFURLRef url = CFURLCreateWithFileSystemPath(kCFAllocatorDefault, inStr, kCFURLHFSPathStyle, 0);
	CFStringRef outStr = CFURLCopyFileSystemPath(url, kCFURLPOSIXPathStyle);
	if (!CFStringGetCString(outStr, outPath, outPathMaxLen, kCFURLPOSIXPathStyle))
		return -1;
	CFRelease(outStr);
	CFRelease(url);
	CFRelease(inStr);
	return 0;
}
#endif

// Initialization code.

static float InitPlugin(float elapsed, float elapsed_sim, int counter, void * ref)
{
	HSLDebugString("HSL: Initializing.\n");
	pHSL->PluginStart();
	return 0.0f;
}

PLUGIN_API int XPluginStart(char * name, char * sig, char * desc)
{
	HSLDebugString("HSL: Startup.\n");
	strcpy(name, "HSL");
	strcpy(sig, "k80.HSL");
	strcpy(desc, "Helicopter Sling Line");

	hsl_output_file.open("HSLLog.txt");

	pHSL = new HSL_PlugIn();

	if (sizeof(unsigned int) != 4 ||
		sizeof(unsigned short) != 2)
	{
		HSLDebugString("HRM: This plugin was compiled with a compiler with weird type sizes.\n");
		return 0;
	}

	// Do deferred sound initialization. See http://www.xsquawkbox.net/xpsdk/mediawiki/DeferredInitialization
	// for more info.
	XPLMRegisterFlightLoopCallback(InitPlugin, -1.0, NULL);

	

	return 1;
}

PLUGIN_API void XPluginStop(void)
{
	pHSL->PluginStop();
	delete pHSL;
	hsl_output_file.close();
}

PLUGIN_API int XPluginEnable(void)
{
	pHSL->PluginEnable();
	return 1;
}

PLUGIN_API void XPluginDisable(void)
{
	pHSL->PluginDisable();
}



PLUGIN_API void XPluginReceiveMessage(XPLMPluginID from, int msg, void * p)
{
	pHSL->PluginReceiveMessage(from, msg, p);
}

void WrapMenuHandler(void * in_menu_ref, void * in_item_ref)
{
	return pHSL->PluginMenuHandler(in_menu_ref, in_item_ref);
}

int WrapDrawCallback(XPLMDrawingPhase inPhase, int inIsBefore, void* inRefcon)
{
	return pHSL->DrawCallback(inPhase, inIsBefore, inRefcon);
}

/*
int WrapHRMVSpeedHandler(XPWidgetMessage  inMessage, XPWidgetID  inWidget, intptr_t inParam1, intptr_t inParam2)
{
	return pHSL->HRMVSpeedHandler(inMessage,inWidget,inParam1,inParam2);
}
int WrapHRMLogbookHandler(XPWidgetMessage  inMessage, XPWidgetID  inWidget, intptr_t inParam1, intptr_t inParam2)
{
	return pHSL->HRMLogbookHandler(inMessage,inWidget,inParam1,inParam2);
}
int WrapHRMLogbookScrollHandler(XPWidgetMessage  inMessage, XPWidgetID  inWidget, intptr_t inParam1, intptr_t inParam2)
{
	return pHSL->HRMLogbookScrollHandler(inMessage,inWidget,inParam1,inParam2);
}

void WrapHRMDrawOutputWindow(XPLMWindowID in_window_id, void * in_refcon)
{
	return pHSL->HRMDrawOutputWindow(in_window_id,in_refcon);
}

int WrapSayBaroCallback(XPLMCommandRef cmd, XPLMCommandPhase phase, void *refcon)
{
	return pHSL->SayBaroCallback(cmd,phase,refcon);
}
int WrapSayWindCallback(XPLMCommandRef cmd, XPLMCommandPhase phase, void *refcon)
{
	return pHSL->SayWindCallback(cmd,phase,refcon);
}
int WrapAnnouncementCallback(XPLMCommandRef cmd, XPLMCommandPhase phase, void *refcon)
{
	return pHSL->AnnouncementCallback(cmd,phase,refcon);
}
int WrapResetHRMCallback(XPLMCommandRef cmd, XPLMCommandPhase phase, void *refcon)
{
	return pHSL->ResetHRMCallback(cmd,phase,refcon);
}
int WrapToogleWindowCallback(XPLMCommandRef cmd, XPLMCommandPhase phase, void *refcon)
{
	return pHSL->ToogleWindowCallback(cmd,phase,refcon);
}

void WrapKeyCallback(XPLMWindowID inWindowID, char inKey, XPLMKeyFlags inFlags, char inVirtualKey, void * inRefcon, int losingFocus)
{
	return pHSL->KeyCallback(inWindowID,inKey,inFlags,inVirtualKey,inRefcon,losingFocus);
}
int WrapMouseClickCallback(XPLMWindowID inWindowID, int x, int y, XPLMMouseStatus inMouse, void * inRefcon)
{
	return pHSL->MouseClickCallback(inWindowID,x,y,inMouse,inRefcon);
}*/

PLUGIN_API float WrapFlightLoopCallback(float elapsedMe, float elapsedSim, int counter, void * refcon)
{
	return pHSL->PluginFlightLoopCallback(elapsedMe,elapsedSim,counter,refcon);
}

double calc_distance_m(double lat1, double long1, double lat2, double long2)
{
	lat1 = lat1 * M_PI / 180;
	long1 = long1 * M_PI / 180;
	lat2 = lat2 * M_PI / 180;
	long2 = long2 * M_PI / 180;

	double rEarth = 6372797;

	double dlat = lat2 - lat1;
	double dlong = long2 - long1;

	double x1 = sin(dlat / 2);
	double x2 = cos(lat1);
	double x3 = cos(lat2);
	double x4 = sin(dlong / 2);

	double x5 = x1 * x1;
	double x6 = x2 * x3 * x4 * x4;

	double temp1 = x5 + x6;

	double y1 = sqrt(temp1);
	double y2 = sqrt(1.0 - temp1);

	double temp2 = 2 * atan2(y1, y2);

	double range_m = temp2 * rEarth;

	return range_m;
}

double calc_distance_nm(double lat1, double long1, double lat2, double long2)
{
	lat1 = lat1 * M_PI / 180;
	long1 = long1 * M_PI / 180;
	lat2 = lat2 * M_PI / 180;
	long2 = long2 * M_PI / 180;

	double rEarth = 6372.797;

	double dlat = lat2 - lat1;
	double dlong = long2 - long1;

	double x1 = sin(dlat / 2);
	double x2 = cos(lat1);
	double x3 = cos(lat2);
	double x4 = sin(dlong / 2);

	double x5 = x1 * x1;
	double x6 = x2 * x3 * x4 * x4;

	double temp1 = x5 + x6;

	double y1 = sqrt(temp1);
	double y2 = sqrt(1.0 - temp1);

	double temp2 = 2 * atan2(y1, y2);

	double rangeKm = temp2 * rEarth;

	double CalcRange = rangeKm * 0.539957;

	return CalcRange;
}