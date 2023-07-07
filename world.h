#pragma once


#include "body.h"

namespace physicalEngine
{
	class World
	{
	public:
		Polygon* addPolygon(float mass, const VerticesListType& verticesList, const Vec2& position = { 0, 0 });

		void step();
		
	//protected:
		std::vector<Polygon*> bodyList;
	private:
		Vec2 Gravity{ 0, -9.8};

		void StepVelocity(Double dt);
		void StepPosition(Double dt);

	};


}

