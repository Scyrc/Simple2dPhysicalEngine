#pragma once
#include "../body/body.h"
#include "../body/circle.h"
#include "../collision/contact.h"

namespace physicalEngine
{
	

	static uint32_t makeId(uint32_t id1, uint32_t id2)
	{
		return std::min(id1, id2) << 16 | std::max(id1, id2);
	}
	static void findMaxSeparation(Polygon* polygon1, Polygon* polygon2, Sat& sat)
	{
		sat.MaxSeparation = DoubleMin;
		VerticesListType verticesWorldPos1;
		polygon1->GetVerticesWorldPosition(verticesWorldPos1);

		VerticesListType verticesWorldPos2;
		polygon2->GetVerticesWorldPosition(verticesWorldPos2);

		for (size_t i = 0; i < verticesWorldPos1.size(); ++i)
		{
			auto va = verticesWorldPos1[i];

			auto N = polygon1->edge(i).normal();

			auto min_sep = DoubleMax;


			for (size_t j = 0; j < verticesWorldPos2.size(); ++j)
			{
				auto vb = verticesWorldPos2[j];

				min_sep = std::min(min_sep, (vb - va).dot(N));
			}

			if (min_sep > sat.MaxSeparation)
			{
				sat.MaxSeparation = min_sep;
				sat.Edgeindex = i;
			}
		}
	}
	static size_t findCloestPointOnPolygonToCircle(const Vec2& circleCenter, const VerticesListType& verticesList)
	{
		size_t res = 0;
		Double minDistance = DoubleMax;
		Double currDistance;
		for (int i =0;i<verticesList.size();++i)
		{
			Vec2 vertice = verticesList[i];
			currDistance = Vec2::twoPointsDistance(vertice, circleCenter);
			if (currDistance < minDistance)
			{
				minDistance = currDistance;
				res = i;
			}
		}
		return res;
	}

	void projectPolygonAtAxis(const VerticesListType& verticesList, const Vec2& axis, Double& minV, Double& maxV)
	{
		minV = DoubleMax;
		maxV = DoubleMin;

		for (auto& v : verticesList)
		{
			float proj = v.dot(axis);

			if (proj < minV) minV = proj;
			if (proj > maxV) maxV = proj;
		}
	}

	void projectPointAtAxis(const Vec2& point, const Vec2& axis, Double& value)
	{
		value = point.dot(axis);
	}

	void projectCircleAtAxis(const Vec2& center, const Double radius, const Vec2& axis, Double& minV, Double& maxV)
	{

		Vec2 directionAndRadius = axis * radius;

		Vec2 p1 = center + directionAndRadius;
		Vec2 p2 = center - directionAndRadius;

		minV = p1.dot(axis);
		maxV = p2.dot(axis);

		if (minV > maxV)
		{
			std::swap(minV, maxV);
		}
	}

	static bool findMaxSeparation(Polygon* polygon1, Circle* circle2, Sat& sat)
	{
		Vec2 directon = circle2->getCentroidWorldPos() - polygon1->getCentroidWorldPos();
		Vec2 normal;
		sat.MaxSeparation = DoubleMax;
		VerticesListType verticesWorldPos1;
		polygon1->GetVerticesWorldPosition(verticesWorldPos1);
		Vec2 axis;
		Double minA, maxA, minB, maxB;
		Double axisDepth;
		for (size_t i = 0; i < polygon1->getEdgeNums(); ++i)
		{

			axis = polygon1->edge(i).normal();

			if (directon.dot(axis) < 0.f)
			{
				continue;
			}

			projectPolygonAtAxis(verticesWorldPos1, axis, minA, maxA);
			projectCircleAtAxis(circle2->getCentroidWorldPos(), circle2->getRadius(), axis, minB, maxB);

			if (minA >= maxB || minB >= maxA)
				return false;

		

			axisDepth = std::min(maxA - minB, maxB - minA);

			if (axisDepth < sat.MaxSeparation)
			{
				

				sat.MaxSeparation = axisDepth;
				sat.Edgeindex = i;
				sat.Bodytag = 0;
				normal = axis;
			}
		}

		size_t pointIndex = findCloestPointOnPolygonToCircle(circle2->getCentroidWorldPos(), verticesWorldPos1);

		Vec2 closetPoint = verticesWorldPos1[pointIndex];

		axis = (closetPoint - circle2->getCentroidWorldPos()).normal();

		//std::cout << "closetPoint: " << pointIndex  << std::endl;

		projectPolygonAtAxis(verticesWorldPos1, axis, minA, maxA);
		projectCircleAtAxis(circle2->getCentroidWorldPos(), circle2->getRadius(), axis, minB, maxB);

		if (minA >= maxB || minB >= maxA)
			return false;


		axisDepth = std::min(maxA - minB, maxB - minA);

		if (axisDepth < sat.MaxSeparation)
		{
			sat.MaxSeparation = axisDepth;
			sat.Edgeindex = pointIndex;
			sat.Bodytag = 0;
			normal = axis;
		}

		

		if (directon.dot(normal) < 0.f)
		{
			sat.reverse = -1;
			//std::cout << "sat.reverse = -1" << std::endl;
		}

		return true;

	}


	bool findMaxSeparation1(Polygon* polygon1, Polygon* polygon2, Sat& sat)
	{
		//Double depth = DoubleMax;
		sat.MaxSeparation = DoubleMax;
		VerticesListType verticesWorldPos1;
		polygon1->GetVerticesWorldPosition(verticesWorldPos1);

		VerticesListType verticesWorldPos2;
		polygon2->GetVerticesWorldPosition(verticesWorldPos2);

		for (size_t i = 0; i < polygon1->getEdgeNums(); ++i)
		{
			auto axis = polygon1->edge(i).normal(); // 已经标准化

			double minA, maxA, minB, maxB;
			projectPolygonAtAxis(verticesWorldPos1, axis, minA, maxA);
			projectPolygonAtAxis(verticesWorldPos2, axis, minB, maxB);

			if (minA >= maxB || minB >= maxA)
				return false;

			float axisDepth = std::min(maxB - minA, maxA - minB);

			if (axisDepth < sat.MaxSeparation)
			{
				sat.MaxSeparation = axisDepth;
				sat.Edgeindex = i;
				sat.Bodytag = 0;
			}
		}

		for (size_t i = 0; i < polygon2->getEdgeNums(); ++i)
		{
			auto axis = polygon2->edge(i).normal(); // 已经标准化

			double minA, maxA, minB, maxB;
			projectPolygonAtAxis(verticesWorldPos1, axis, minA, maxA);
			projectPolygonAtAxis(verticesWorldPos2, axis, minB, maxB);

			if (minA >= maxB || minB >= maxA)
				return false;

			float axisDepth = std::min(maxB - minA, maxA - minB);

			if (axisDepth < sat.MaxSeparation)
			{
				sat.MaxSeparation = axisDepth;
				sat.Edgeindex = i;
				sat.Bodytag = 1;
			}
		}
		/*
		Vec2 direction =

		if (FlatMath.Dot(direction, normal) < 0f)
		{
			normal = -normal;
		}
		*/
		

		return true;
	}




	static size_t closestEdge(Vec2 Normal, Polygon* polygon)
	{
		size_t idx = SIZE_MAX;

		auto minDotV = DoubleMax;

		for (size_t i = 0; i < polygon->getEdgeNums(); ++i)
		{
			Vec2 edgeNormal = polygon->edge(i).normal();

			Double dotValue = edgeNormal.dot(Normal);

			if (dotValue < minDotV)
			{
				minDotV = dotValue;
				idx = i;
			}
		}

		return idx;
	}
	static Double PointSegmentDistance(const Vec2& point, Vec2 lineStart, Vec2 lineEnd, Vec2& contactPoint)
	{
		Vec2 ab = lineEnd - lineStart;
		Vec2 ap = point - lineStart;


		auto projV = ab.dot(ap);

		Double diff = projV / ab.lengthSquared();

		if (diff <= 0.f)
		{
			contactPoint = lineStart;
		}
		else if (diff >= 1.f)
		{
			contactPoint = lineEnd;

		}
		else
		{
			contactPoint = lineStart + ab * diff;
		}


		return Vec2::DistanceSquared(point, contactPoint);

	}
	static void calcContactPointsPolygonVsPolygon(Collision& colliison)
	{
		Polygon* polygonA = dynamic_cast<Polygon*>(colliison.bodyA);
		Polygon* polygonB = dynamic_cast<Polygon*>(colliison.bodyB);
		assert(polygonA != nullptr);
		assert(polygonB != nullptr);

		/*colliison.Normal = polygonA->edge(colliison.sat.Edgeindex).normal();

		size_t idx = closestEdge(colliison.Normal, polygonB);*/

		Vec2 contact1, contact2;
		size_t contactCount = 0;
		VerticesListType verticeWorldPosA;
		polygonA->GetVerticesWorldPosition(verticeWorldPosA);

		VerticesListType verticeWorldPosB;
		polygonB->GetVerticesWorldPosition(verticeWorldPosB);

		Double minDisSq = DoubleMax;
		/*
		for (size_t i = 0; i < verticeWorldPosA.size(); ++i)
		{
			Vec2 P = verticeWorldPosA[i];
			for (size_t j = 0; j < verticeWorldPosB.size(); ++j)
			{
				Vec2 cp;
				auto disSq = PointSegmentDistance(P, verticeWorldPosB[j], verticeWorldPosB[(j + 1) % verticeWorldPosB.size()], cp);

				if (twoDMath::nearlyEqual(disSq, minDisSq))
				{
				if (!contact1.fuzzyEqual(cp, 0.05f))
				{
					contact2 = cp;
					contactCount = 2;
				}
				}
				else if (disSq < minDisSq)
				{
					minDisSq = disSq;
					contactCount = 1;
					contact1 = cp;
				}
				
			}
		}
		*/

		

		for (size_t i = 0; i < verticeWorldPosB.size(); ++i)
		{
			Vec2 P = verticeWorldPosB[i];
			for (size_t j = 0; j < verticeWorldPosA.size(); ++j)
			{
				Vec2 cp;
				auto disSq = PointSegmentDistance(P, verticeWorldPosA[j], verticeWorldPosA[(j + 1) % verticeWorldPosA.size()], cp);

				if (twoDMath::nearlyEqual(disSq, minDisSq, 0.00000001f))
				{
					if (!contact1.fuzzyEqual(cp, 0.05f))
					{
						contact2 = cp;
						contactCount = 2;
					}
				}
				else if (disSq < minDisSq)
				{
					minDisSq = disSq;
					contactCount = 1;
					contact1 = cp;
				}
				
			}
		}

		if (contactCount == 2)
		{
			colliison.contactList.emplace_back(new Contact(contact1));
			colliison.contactList.emplace_back(new Contact(contact2));
		}
		else if (contactCount == 1)
		{
			colliison.contactList.emplace_back(new Contact(contact1));
		}
		else
		{
			//std::cout << "检测空的接触列表" << std::endl;
			assert(false);
		}


	}
	size_t findIncidentEdge(Polygon* polygon, const Vec2& normal)
	{
		// find closet vertice
		
		VerticesListType verticeWorldPos;
		polygon->GetVerticesWorldPosition(verticeWorldPos);

		Double maxValue = Constant::MinValue;
		Double projV;
		size_t clostIndex = 0;
		
		for (int i=0; i< verticeWorldPos.size(); ++i)
		{
			projectPointAtAxis(verticeWorldPos[i], normal, projV);
			if (maxValue < projV)
			{
				maxValue = projV;
				clostIndex = i;
			}
		}
	

		Vec2 edge1, edge2;

		polygon->adjacentEdge(clostIndex, edge1, edge2);

		edge1.normalize();

		edge2.normalize();
	
		if (std::abs(edge1.dot(normal)) < std::abs(edge2.dot(normal)))
		{
			return clostIndex == 0 ? verticeWorldPos.size() - 1 : clostIndex - 1;
		}
		else
		{
			return clostIndex;
		}
	}

	static void clip(std::vector<Contact>& in, const Vec2& p1, const Vec2& p2, bool isSave)
	{
		Vec2 normal = (p2 - p1).normal();

		Double projV1 = normal.dot(in[0].point - p1);
		Double projV2 = normal.dot(in[1].point - p1);

		if (projV1 <= 0.f && projV2 <= 0.f)
		{
			return;
		}

		if (isSave == false) // 第2次裁剪
		{
			if (projV1 > 0.f)
			{
				in.erase(in.begin());
			}
			else
			{
				in.erase(in.begin() + 1);
			}

			return;
		}

		// 第1次裁剪
		Double interp = projV1 / (projV1 - projV2);

		Vec2 pos = in[0].point + (in[1].point - in[0].point) * interp;

		if (projV1 > 0.f)
		{
			in[0] = pos;
		}
		else
		{
			in[1] = pos;
		}
	}
	static void calculatePerpendicularPoint(const Vec2& A, const Vec2& B, const Vec2& P, Vec2& cp) {

		// Check if the line is vertical
		if (B.x - A.x == 0) {
			cp = Vec2{ A.x, P.y };
		}

		// Check if the line is horizontal
		if (B.y - A.y == 0) {
			cp = Vec2{ P.x, A.y };
		}

		double slope = (B.y - A.y) / (B.x - A.x);


		double perpendicularSlope = -1 / slope;
		double perpendicularX = (P.y - A.y + (1 / perpendicularSlope) * A.x + perpendicularSlope * P.x) / (1 / perpendicularSlope + perpendicularSlope);
		double perpendicularY = A.y + perpendicularSlope * (perpendicularX - A.x);

		cp = Vec2{ perpendicularX, perpendicularY };
	}

	static void projectPointToLine(const Vec2& A, const Vec2& B, const Vec2& P, Vec2& cp)
	{
		Vec2 AB = B - A;
		Vec2 AP = P - A;
		Vec2 AC = AB * (AB.dot(AP) / AB.lengthSquared());
		cp = A + AC;
	}

	static void calcContactPointsPolygonVsPolygonVClip(Collision& colliison)
	{
		Polygon* polygonA = dynamic_cast<Polygon*>(colliison.bodyA);
		Polygon* polygonB = dynamic_cast<Polygon*>(colliison.bodyB);
		assert(polygonA != nullptr);
		assert(polygonB != nullptr);

		

		//size_t refenceEdgeIdx = colliison.sat.Edgeindex;

		Vec2 normal = colliison.Normal;

		size_t refenceEdgeIdx = findIncidentEdge(polygonA, normal);

		size_t incidentEdge = findIncidentEdge(polygonB, normal * -1);


		VerticesListType verticeWorldPosB;
		polygonB->GetVerticesWorldPosition(verticeWorldPosB);

		Contact c1(verticeWorldPosB[incidentEdge]);
		Contact c2(verticeWorldPosB[(incidentEdge + 1) % verticeWorldPosB.size()]);

		std::vector<Contact> contacts;
		contacts.emplace_back(c1);
		contacts.emplace_back(c2);

		//第一次裁剪
		VerticesListType verticeWorldPosA;

		polygonA->GetVerticesWorldPosition(verticeWorldPosA);

		if (refenceEdgeIdx == 0)
		{
			clip (contacts, verticeWorldPosA.back(), verticeWorldPosA[refenceEdgeIdx], true);
		
		}
		else
		{
			clip( contacts, verticeWorldPosA[refenceEdgeIdx - 1], verticeWorldPosA[refenceEdgeIdx], true);
		
		}

		clip(contacts, verticeWorldPosA[(refenceEdgeIdx + 1) % verticeWorldPosA.size()], verticeWorldPosA[(refenceEdgeIdx + 2) % verticeWorldPosA.size()], true);
		
	
		//第二次裁剪
	

		clip(contacts, verticeWorldPosA[refenceEdgeIdx], verticeWorldPosA[(refenceEdgeIdx + 1) % verticeWorldPosA.size()], false);

		for (auto& contact : contacts)
		{
			contact.pointB = contact.point;
			contact.localB = colliison.bodyB->toLocalPoint(contact.pointB);
			projectPointToLine(verticeWorldPosA[refenceEdgeIdx], verticeWorldPosA[(refenceEdgeIdx + 1) % verticeWorldPosA.size()],contact.point, contact.pointA);
			contact.localA = colliison.bodyA->toLocalPoint(contact.pointA);


			colliison.contactListNew.emplace_back(new Contact(contact));

			//colliison.contactList.emplace_back(new Contact(contact));
		}

	}
	static void calcContactCircleVsCircle(Collision& colliison)
	{
		Circle* circleA = dynamic_cast<Circle*>(colliison.bodyA);
		Circle* circleB = dynamic_cast<Circle*>(colliison.bodyB);

		assert(circleA != nullptr);
		assert(circleB != nullptr);

		/*colliison.Normal = polygonA->edge(colliison.sat.Edgeindex).normal();

		size_t idx = closestEdge(colliison.Normal, polygonB);*/

		Vec2 abDirection = circleB->getCentroidWorldPos() - circleA->getCentroidWorldPos();
		abDirection.normalize();

		Vec2 contact = circleA->getCentroidWorldPos() + abDirection * circleA->getRadius();

		Contact* c = new Contact(contact);
		c->pointA = contact;
		c->localA = colliison.bodyA->toLocalPoint(c->pointA);

		c->pointB = circleB->getCentroidWorldPos() - abDirection * circleB->getRadius();
		c->pointB = c->pointA;
		//c->pointB = circleB->getCentroidWorldPos() - abDirection * circleB->getRadius();

		c->localB = colliison.bodyB->toLocalPoint(c->pointB);

		//colliison.contactList.emplace_back(new Contact(contact));

		colliison.contactListNew.emplace_back(c);

	}

	static void calcContactPolygonVsCircle(Collision& colliison)
	{
	

		Polygon *polygonA = dynamic_cast<Polygon*>(colliison.bodyA);
		Circle* circleB = dynamic_cast<Circle*>(colliison.bodyB);
		assert(polygonA != nullptr);
		assert(circleB != nullptr);

		Vec2 directon = circleB->getCentroidWorldPos() - polygonA->getCentroidWorldPos();

		bool flag = false;
		Double minDisSq = DoubleMax;
		Vec2 contact;

		Vec2 contactTemp;
		size_t contactCount = 0;
		VerticesListType verticeWorldPosA;
		polygonA->GetVerticesWorldPosition(verticeWorldPosA);

		for (size_t i = 0; i < verticeWorldPosA.size(); ++i)
		{
			Vec2 edgeNormal = polygonA->edge(i).normal();

			if (edgeNormal.dot(directon) < 0.f)
			{
				continue;
			}
			
			Double dis = PointSegmentDistance(circleB->getCentroidWorldPos(), verticeWorldPosA[i], verticeWorldPosA[(i + 1) % verticeWorldPosA.size()], contactTemp);

			if (dis < minDisSq)
			{
				minDisSq = dis;
				contact = contactTemp;
				flag = true;
			}
		}

		if (!flag)
		{
			assert(false);
		}
		//std::cout << contact.x << " " << contact.y << std::endl;
		//colliison.contactList.emplace_back(new Contact(contact));
		Contact* c = new Contact(contact);
		c->pointA = c->point;
		c->localA = colliison.bodyA->toLocalPoint(c->pointA);

		c->pointB = c->point;
		c->localB = colliison.bodyB->toLocalPoint(c->pointB);

		colliison.contactListNew.emplace_back(c);

	}

	static void calcContactPoints(Collision& colliison)
	{
		if (colliison.bodyA->getType() == BodyType::PolygonType && (colliison.bodyB->getType() == BodyType::PolygonType))
		{

			//calcContactPointsPolygonVsPolygon(colliison);
			calcContactPointsPolygonVsPolygonVClip(colliison);
			return;
		}

		if (colliison.bodyA->getType() == BodyType::CircleType && (colliison.bodyB->getType() == BodyType::CircleType))
		{

			calcContactCircleVsCircle(colliison);
			return;
		}
		if (colliison.bodyA->getType() == BodyType::PolygonType && (colliison.bodyB->getType() == BodyType::CircleType))
		{

			calcContactPolygonVsCircle(colliison);
			return;
		}
	}
}
