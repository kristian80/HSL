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

#include "PhysicsThread.h"
#include "HSL_PlugIn.h"
#include <thread>

PhysicsThread::PhysicsThread(HSL_PlugIn& HSLNew) :
	HSL(HSLNew)
{
}

void PhysicsThread::RunPhysicsThread(int index)
{
	using namespace std::chrono_literals;

	std::unique_lock<std::recursive_mutex> physics_shm_lock(cargoDataSharedMutex, std::defer_lock);

	//SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_NORMAL);

	while (myRunFlag)
	{
		if (physics_shm_lock.try_lock()) // Non-Blocking Lock
		{
			if (HIGH_PERFORMANCE != HSL.myCargoDataShared.myHighPerformace)
			{
				if (HSL.myCargoDataShared.myHighPerformace == true)
					SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_HIGHEST);
				else
					SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_NORMAL);
			}

			HIGH_PERFORMANCE = HSL.myCargoDataShared.myHighPerformace;
			myRunFlag = HSL.myCargoDataShared.myThreadRunFlag;

			if (HSL.myCargoDataShared.myComputationRunFlag == true)
			{

				if (HSL.myCargoDataShared.myNewFrame == true)
				{
					HSL.myCargo.myUpdateHelicopterPosition = true;
					HSL.myHook.myUpdateHelicopterPosition = true;
					HSL.myCargoDataShared.myNewFrame = false;
				}

				HSL.myCargo.CalculatePhysics();
				HSL.myHook.CalculatePhysics();
			}

			physics_shm_lock.unlock(); // Unlock
		}
		if (HIGH_PERFORMANCE == true)
		{
			volatile int counter = 0;
			for (counter = 0; counter < 10000; counter++);
		}
		else
		{
			std::this_thread::sleep_for(1us);
		}

	}
}
