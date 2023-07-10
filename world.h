#pragma once


#include "body.h"
#include "bvhtree.h"

namespace physicalEngine
{
	class World
	{
	public:
		World();
		Polygon* addPolygon(float mass, const VerticesListType& verticesList, const Vec2& position = { 0, 0 });
		void GenerateTree();
		void step();
	
	//protected:
		std::vector<Body*> bodyList;
	private:
		Vec2 Gravity{ 0, -9.8};

		void StepVelocity(Double dt);
		void StepPosition(Double dt);

		BVHTree* bvhTree;

	};


}

