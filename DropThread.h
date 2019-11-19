#pragma once

#include "HSL.h"
#include "DropObject.h"
#include "TSQueue.h"

namespace DropThreadData
{
	const int ThreadStop = 0;
	const int ThreadNewHSLData = 1;
	const int ThreadCompute = 2;
}

class DropThread
{
private:
	DropHSLData myHSLData;
	bool myRunFlag = true;
public:
	TSQueue<DropObject*> myDropObjectIn;
	TSQueue<DropObject*> myDropObjectOut;

	TSQueue<DropHSLData> myHSLDataIn;
	TSQueue<int>		 myHSLCommandsIn;

	void RunDropThread(int index);


};

