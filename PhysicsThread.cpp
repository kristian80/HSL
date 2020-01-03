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

	SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_HIGHEST);

	while (myRunFlag)
	{
		if (physics_shm_lock.try_lock()) // Non-Blocking Lock
		{
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
