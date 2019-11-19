#include "DropThread.h"

void DropThread::RunDropThread(int index)
{
	while (myRunFlag)
	{
		int command = myHSLCommandsIn.pop();
		if (command == DropThreadData::ThreadStop)
		{
			myRunFlag = false;
		}
		else if (command == DropThreadData::ThreadNewHSLData)
		{
			while (myHSLDataIn.size() > 0)
			{
				myHSLData = myHSLDataIn.pop();
			}
		}
		else if (command == DropThreadData::ThreadCompute)
		{
			while (myDropObjectIn.size() > 0)
			{
				DropObject* pDrop = myDropObjectIn.pop();
				pDrop->CalculatePhysics(myHSLData);
				myDropObjectOut.push(pDrop);
			}
		}

	}
}
