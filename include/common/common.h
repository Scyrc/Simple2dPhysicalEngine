#pragma once
#include "../body/body.h"
namespace physicalEngine
{
	struct Sat
	{
		int Edgeindex;
		Double MaxSeparation;
		int Bodytag;
		Sat() :Edgeindex(-1), MaxSeparation(DoubleMin) {}
	};
	struct Contact
	{
		Contact(Vec2& point_)
		{
			point = point_;
		}
		Vec2 point;

		Vec2 ra;
		Vec2 rb;

	};
	struct Collision
	{
		Body* bodyA;
		Body* bodyB;
		Vec2 Normal;
		Sat sat;
		Double depth;
		std::vector<Contact*> contactList;
	};

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
		for (size_t i = 0; i < verticeWorldPosA.size(); ++i)
		{
			Vec2 P = verticeWorldPosA[i];
			for (size_t j = 0; j < verticeWorldPosB.size(); ++j)
			{
				Vec2 cp;
				auto disSq = PointSegmentDistance(P, verticeWorldPosB[j], verticeWorldPosB[(j + 1) % verticeWorldPosB.size()], cp);

				if (twoDMath::nearlyEqual(disSq, minDisSq))
				{
				if (!Vec2::NearlyEqual(contact1, cp))
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

		for (size_t i = 0; i < verticeWorldPosB.size(); ++i)
		{
			Vec2 P = verticeWorldPosB[i];
			for (size_t j = 0; j < verticeWorldPosA.size(); ++j)
			{
				Vec2 cp;
				auto disSq = PointSegmentDistance(P, verticeWorldPosA[j], verticeWorldPosA[(j + 1) % verticeWorldPosA.size()], cp);

				if (twoDMath::nearlyEqual(disSq, minDisSq))
				{
					if (!Vec2::NearlyEqual(contact1, cp))
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
			std::cout << "检测空的接触列表" << std::endl;
			assert(false);
		}


	}

	static void calcContactPoints(Collision& colliison)
	{
		if (colliison.bodyA->getType() == BodyType::PolygonType && (colliison.bodyB->getType() == BodyType::PolygonType))
		{

			calcContactPointsPolygonVsPolygon(colliison);
		}
	}
}
