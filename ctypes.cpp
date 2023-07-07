#include "ctypes.h"
namespace physicalEngine {
	Vec2::Vec2() : x(0), y(0)
	{

	}

	Vec2::Vec2(Double _x, Double _y) : x(_x), y(_y)
	{

	}



	Vec2 Mat22::operator*(const Vec2& v) const
	{
		return{ matrix22[0][0] * v[0] + matrix22[0][1] * v[1],  //todo 
			    matrix22[1][0] * v[0] + matrix22[1][1] * v[1]
		};
	}

	const Mat22 Mat22::I{ 1, 0, 0, 1 };

}