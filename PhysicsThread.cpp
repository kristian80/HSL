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

	while (myRunFlag)
	{
		if (physics_shm_lock.try_lock()) // Non-Blocking Lock
		{
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
		std::this_thread::sleep_for(1us); // Low to enable fast recover after lock
	}
}
