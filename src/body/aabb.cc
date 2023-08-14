#include "../../include/body/aabb.h"
#include "../../include/body/body.h"
#include "../../include/body/circle.h"
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
		case CircleType:
			FromCicle(body);
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

	void AABB::FromCicle(Body* body)
	{
		Circle* circle = dynamic_cast<Circle*>(body);
		assert(circle != nullptr);

		height = circle->getRadius() * 2.0f;
		width = circle->getRadius() * 2.0f;

		position = circle->getCentroidWorldPos();

		lowerBound = { position.x - circle->getRadius(), position.y - circle->getRadius() };
		upperBound = { position.x + circle->getRadius(), position.y + circle->getRadius() };
	}

	void AABB::draw()
	{
		glBegin(GL_LINE_LOOP);
		glBegin(GL_LINE_STRIP);
		glColor3f(0.0f, 1.0f, 0.0f); 

		auto positionShow = position * 10;
		auto widthShow = width * 10;
		auto heightShow = height * 10;

		
	    glVertex2d(positionShow.x  - 0.5 * widthShow, positionShow.y  - 0.5 * heightShow);
		glVertex2d(positionShow.x  + 0.5 * widthShow, positionShow.y - 0.5 * heightShow);
		glVertex2d(positionShow.x  + 0.5 * widthShow, positionShow.y  + 0.5 * heightShow);
		glVertex2d(positionShow.x  - 0.5 * widthShow, positionShow.y  + 0.5 * heightShow);


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

	bool AABB::overlap(const AABB& aabb)
	{
		Vec2 d1, d2;

		d1 = lowerBound - aabb.upperBound;
		d2 = aabb.lowerBound - upperBound;

		if (d1.x > 0.0f || d1.y > 0.0f)
			return false;

		if (d2.x > 0.0f || d2.y > 0.0f)
			return false;

		return true;
	}

}