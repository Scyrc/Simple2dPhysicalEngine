#pragma once
#include <cassert>
#include <array>
namespace physicalEngine
{
	using Double = double;
	struct Mat22;
	struct Vec2
	{
		Double x, y;

		Vec2();

		Vec2(Double, Double);
		Double& operator[](size_t index)
		{
			assert(index < 2);
			return index == 0 ? x : y;
		}

		const Double& operator[](size_t index) const 
		{
			//return this[index];
			return (*const_cast<Vec2*>(this))[index];
		}

		 Vec2 operator-(const Vec2& v) const
		 {
			 return { x - v.x, y - v.y };
		 }

		 Vec2 operator+(const Vec2& v) const
		 {
			 return { x + v.x, y + v.y };
		 }

		 Vec2 operator*(Double a) const
		 {
			 return { a*x, a*y};
		 }

		 Vec2 operator/(Double a) const
		 {
			 assert(a != 0);
			 return { x / a, y / a};
		 }

		
	};

	struct Mat22
	{
	public:
		static const Mat22 I;
		std::array<Vec2, 2> matrix22;
		Mat22() :Mat22(0, 0, 0, 0) {}
		Mat22(double x1, double y1, double x2, double y2): 
			matrix22{{{x1, y1},{x2, y2}}}{}
		Mat22(const std::array<Vec2, 2>& mat) :matrix22(mat){}



		Vec2 operator*(const Vec2& v) const;
	};

	
}