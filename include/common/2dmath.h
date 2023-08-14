#pragma once
#include <cmath>
namespace physicalEngine
{
	static class twoDMath
	{
	public:
		static  double VerySmallAmount;

		static bool nearlyEqual(double v1, double v2, double diff)
		{
			return fabs(v1 - v2) <= diff;
		}

		static bool realEqual(double v1, double v2)
		{
			return fabs(v1 - v2) <= 0.00001f;
		}
	};


}

