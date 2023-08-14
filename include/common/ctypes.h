#pragma once
#include <cassert>
#include <array>
#include <float.h>
#include <cmath>
#include "2dmath.h"
#include "constant.h"
namespace physicalEngine
{
	
	constexpr Double DoubleMax = DBL_MAX;;
	constexpr Double DoubleMin = -DBL_MAX;

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

		Double cross(const Vec2& v) const
		{
			return x * v.y - y * v.x;
		}

		Double dot(const Vec2& v) const
		{
			return x * v.x + y * v.y;
		}

		 Vec2 operator-(const Vec2& v) const
		 {
			 return { x - v.x, y - v.y };
		 }

		 Vec2 operator+(const Vec2& v) const
		 {
			 return { x + v.x, y + v.y };
		 }

		 void operator+=(const Vec2& v) 
		 {
			 x += v.x;
			 y += v.y;
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

		 Vec2 normal() const
		 {
			 Vec2 N{ -y, x };
			 N.normalize();
			 return N;
		 }

		 Vec2 defaultNormal() const
		 {
			 Vec2 N{ -y, x };
			 return N;
		 }

		 Double length () const
		 {
			 return std::sqrt(x * x + y * y);
		 }
		 void normalize()
		 {
			 (*this) = (*this) / std::sqrt(x * x + y * y);

			 
			 /*
			 * 
			 debug  ¥ÌŒÛ–¥∑® ................

			 x /= std::sqrt(x * x + y * y);
			 y /= std::sqrt(x * x + y * y);

			 */

		 }

		 Double lengthSquared()
		 {
			 return x * x + y * y;
		 }


		 static Double DistanceSquared(Vec2 p1, Vec2 p2)
		 {
			 return (p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y);
		 }

		 static Double twoPointsDistance(Vec2 p1, Vec2 p2)
		 {
			 return std::sqrt(Vec2::DistanceSquared(p1, p2));
		 }

		 static bool NearlyEqual(Vec2 v1, Vec2 v2)
		 {
			 return DistanceSquared(v1, v2) <= twoDMath::VerySmallAmount * twoDMath::VerySmallAmount;
		 }

		 bool fuzzyEqual(const Vec2& rhs, const Double& diff) const
		 {
			 return std::abs(x - rhs.x) <= diff && std::abs(y - rhs.y) <= diff;
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


		void rotate(Double theta);
		Vec2 operator*(const Vec2& v) const;
		Mat22 operator*(const Double& v) const;

		Mat22 invert() const;

		Vec2 multiply(const Vec2& v) const;

		Double det() const;
	};

	
}