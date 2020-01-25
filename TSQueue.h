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
		return (int) myQueue.size();
	}
};