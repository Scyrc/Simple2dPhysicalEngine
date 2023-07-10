#include "aabb.h"
#include <gl/glut.h>

namespace physicalEngine
{
	AABB::AABB(Body* body)
	{
		auto type = body->getType();
		switch (type)
		{
		case PolygonType:
				FromPolygon(body);
			break;
		default:
			break;
		}
	}

	

	AABB::AABB(const AABB& aabb1, const AABB& aabb2)
	{
		Combine(aabb1, aabb2);
	}

	void AABB::FromPolygon(Body* body)
	{
		Polygon* polygon = dynamic_cast<Polygon*>(body);
		assert(polygon != nullptr);
		VerticesListType verticesPos;
		polygon->GetVerticesWorldPosition(verticesPos);
		assert(verticesPos.size() > 0);
		Double minX = verticesPos[0].x;
		Double maxX = verticesPos[0].x;

		Double minY = verticesPos[0].y;
		Double maxY = verticesPos[0].y;

		for (auto& vertice : verticesPos)
		{
			minX = minX > vertice.x ? vertice.x : minX;
			maxX = maxX < vertice.x ? vertice.x : maxX;
			minY = minY > vertice.y ? vertice.y : minY;
			maxY = maxY < vertice.y ? vertice.y : maxY;
		}

		height = maxY - minY;
		width = maxX - minX;

		position = Vec2(minX + 0.5 * width, minY + 0.5 * height);

		lowerBound = { position.x - 0.5 * width, position.y - 0.5 * height };
		upperBound = { position.x + 0.5 * width, position.y + 0.5 * height };


		//zoomSize(1.2);
	}

	void AABB::draw()
	{
		glBegin(GL_LINE_LOOP);
		glBegin(GL_LINE_STRIP);
		glColor3f(0.0f, 1.0f, 0.0f); 

		
	    glVertex2d(position.x - 0.5 *width, position.y - 0.5 * height);
		glVertex2d(position.x + 0.5 * width, position.y - 0.5 * height);
		glVertex2d(position.x + 0.5 * width, position.y + 0.5 * height);
		glVertex2d(position.x - 0.5 * width, position.y + 0.5 * height);


		glEnd();
	}

	void AABB::zoomSize(Double factor)
	{
		height *= factor;
		width *= factor;

		lowerBound = { position.x - 0.5 * width, position.y - 0.5 * height };
		upperBound = { position.x + 0.5 * width, position.y + 0.5 * height };
	}

	void AABB::Combine(const AABB& aabb1, const AABB& aabb2)
	{
		Double minX = std::min(aabb1.lowerBound.x, aabb2.lowerBound.x);
		Double maxX = std::max(aabb1.upperBound.x, aabb2.upperBound.x);
		Double minY = std::min(aabb1.lowerBound.y, aabb2.lowerBound.y);
		Double maxY = std::max(aabb1.upperBound.y, aabb2.upperBound.y);


		height = maxY - minY;
		width = maxX - minX;

		position = Vec2(minX + 0.5 * width, minY + 0.5 * height);

		lowerBound = { position.x - 0.5 * width, position.y - 0.5 * height };
		upperBound = { position.x + 0.5 * width, position.y + 0.5 * height };
	}

	bool AABB::Contain(const AABB& aabb1)
	{
		return lowerBound.x <= aabb1.lowerBound.x&&
			lowerBound.y <= aabb1.lowerBound.y&&
			upperBound.x >= aabb1.upperBound.x&&
			upperBound.y >= aabb1.upperBound.y;
	}

}