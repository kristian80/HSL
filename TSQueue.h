#pragma once
#include <queue>
#include <thread>
#include <mutex>
#include <condition_variable>

template <typename T>
class TSQueue
{

private:
	std::queue<T> myQueue;
	std::mutex myMutex;
	std::condition_variable myCond;
public:

	T pop()
	{
		std::unique_lock<std::mutex> mlock(myMutex);
		while (myQueue.empty())
		{
			myCond.wait(mlock);
		}
		auto item = myQueue.front();
		myQueue.pop();
		return item;
	}

	void push(T item)
	{
		std::unique_lock<std::mutex> mlock(myMutex);
		myQueue.push(item);
		mlock.unlock();
		myCond.notify_one();
	}

	int size()
	{
		return myQueue.size();
	}
};