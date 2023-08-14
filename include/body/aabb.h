#pragma once

#include "body.h"
namespace physicalEngine
{
	class AABB
	{
	public:
		AABB() =default;
		AABB(Body* body);
		AABB(const AABB& aabb1, const AABB& aabb2);
		void draw();
		void zoomSize(Double factor);
		Double getSurfaceArea() const { return (height + width) * 2.0; }
		void Combine(const AABB& aabb1, const AABB& aabb2);

		bool Contain(const AABB& aabb1);

		bool overlap(const AABB& aabb);
		//static AABB& CombineAABB(const AABB& aabb1, const AABB& aabb2);

	protected:
		void FromPolygon(Body* body);
		void FromCicle(Body* body);

	private:
		Vec2 position; // 矩形几何中心作为位置
		Double height = 0;
		Double width = 0;
		Double topL = 0;
		Double topR = 0;
		Double bottomL = 0;
		Double bottomR = 0;
		Vec2 lowerBound;
		Vec2 upperBound;

	};

}
