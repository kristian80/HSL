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
