#pragma once

#include "body.h"
namespace physicalEngine
{
	class AABB
	{
	public:
		AABB() = delete;
		AABB(Body* body);
		void draw();
		void zoomSize(Double factor);
	protected:
		void FromPolygon(Body* body);

	private:
		Vec2 position; // ���μ���������Ϊλ��
		Double height = 0;
		Double width = 0;
		Double topL = 0;
		Double topR = 0;
		Double bottomL = 0;
		Double bottomR = 0;
	};

}
