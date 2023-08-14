#include "../../include/common/ctypes.h"
#include <iostream>
namespace physicalEngine {
	Vec2::Vec2() : x(0.f), y(0.f)
	{

	}

	Vec2::Vec2(Double _x, Double _y) : x(_x), y(_y)
	{

	}



	void Mat22::rotate(Double theta)
	{
		const auto s = std::sin(theta);
		const auto c = std::cos(theta);

		matrix22[0][0] = c;
		matrix22[0][1] = -s;

		matrix22[1][0] = s;
		matrix22[1][1] = c;

	}

	Vec2 Mat22::operator*(const Vec2& v) const
	{
		return{ matrix22[0][0] * v[0] + matrix22[0][1] * v[1],  //todo 
			    matrix22[1][0] * v[0] + matrix22[1][1] * v[1]
		};
	}

	physicalEngine::Mat22 Mat22::operator*(const Double& v) const
	{
		return Mat22{ matrix22[0][0] * v ,matrix22[0][1] * v,  //todo 
				matrix22[1][0] * v , matrix22[1][1] * v
		};
	}

	physicalEngine::Mat22 Mat22::invert() const
	{
		auto det_ = det();
		if (det_ == 0.f)
		{
			const auto inf = std::numeric_limits<Double>::infinity();

			return Mat22{ inf, inf,inf,inf};
		}
		
		Mat22 res{ matrix22[1][1], -matrix22[1][0], -matrix22[0][1], matrix22[0][0] };
		res = res * (1.0f / det_);
		return res;
		
		
	}

	physicalEngine::Vec2 Mat22::multiply(const Vec2& v) const
	{
		return Vec2{ matrix22[0][0] * v.x + matrix22[0][1] * v.y, matrix22[1][0] * v.x + matrix22[1][1] * v.y };
	}

	physicalEngine::Double Mat22::det() const
	{
		return matrix22[0][0] * matrix22[1][1] - matrix22[1][0] * matrix22[0][1];
	}

	const Mat22 Mat22::I{ 1, 0, 0, 1 };

}

int maintest()
{
	using namespace physicalEngine;
	Vec2 edge = Vec2(389.48416757653843, 253.92712781139048) - Vec2(119.19350720581272, 384.09224954665791);
	auto N = edge.normal();


	Vec2 ap = Vec2(274.14990691854609, 313.00000000000000) - Vec2(119.19350720581272, 384.09224954665791);

	auto res = N.dot(ap);
	std::cout << ap.length() << std::endl;
	std::cout << N.length() << std::endl;

	std::cout << res << std::endl;

	return 0;

}