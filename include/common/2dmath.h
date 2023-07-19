#pragma once
#include <cmath>
namespace physicalEngine
{
	static class twoDMath
	{
	public:
		static  double VerySmallAmount;

		static bool nearlyEqual(double v1, double v2)
		{
			return abs(v1 - v2) < VerySmallAmount;
		}
	};


}

