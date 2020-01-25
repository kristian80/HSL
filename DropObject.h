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
#include "LoadBaseObject.h"


class DropObject :
	public LoadBaseObject
{
public:


	vector<double> myVectorZeroVector = vector<double>(3);

	vector<double> myVectorVelocity = vector<double>(3);
	vector<double> myVectorForceGravity = vector<double>(3);
	vector<double> myVectorWindVelocity = vector<double>(3);

	vector<double> myVectorDisplayOffset = vector<double>(3);
	vector<double> myVectorDisplayAngle = vector<double>(3);

	vector<double> myVectorSize = vector<double>(3); //Length / Width / Height
	vector<double> myVectorCrossSection = vector<double>(3);
	vector<double> myVectorCW = vector<double>(3);

	bool myInstancedDrawing = false;

	double myMass = 5.0;
	double myVolume = 1.0;

	double myAirSpeed = 0;
	double myAirResistance = 0;

	vector<double> myVectorAirVelocity = vector<double>(3);
	vector<double> myVectorForceAir = vector<double>(3);
	vector<double> myVectorForceTotal = vector<double>(3);
	vector<double> myVectorAccTotal = vector<double>(3);
	vector<double> myVectorVelocityDelta = vector<double>(3);


	DropObject();
	DropObject(vector<double> pos, vector<double> vel);
	~DropObject();

	void CalculatePhysics(DropHSLData& HSL);
};

