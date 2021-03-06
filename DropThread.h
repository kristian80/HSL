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

