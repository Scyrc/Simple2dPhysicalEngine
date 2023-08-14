#pragma once
#include "../common/common.h"
namespace physicalEngine
{

	class NarrowPhase
	{
	public:
		static void BodyCollisionDetect(Body* BodyA, Body* BodyB, std::unordered_map<uint32_t, Collision>& collisionMap, Collision& collison)
		{
			if (BodyA->getType() == BodyType::PolygonType && BodyB->getType() == BodyType::PolygonType)  //PolygonVsPolygon
			{
				Polygon* PolygonA = dynamic_cast<Polygon*>(BodyA);
				Polygon* PolygonB = dynamic_cast<Polygon*>(BodyB);
				assert(PolygonA != nullptr);
				assert(PolygonB != nullptr);
				PolygonVsPolygon(PolygonA, PolygonB, collisionMap, collison);
				return;
			}

			if ((BodyA->getType() == BodyType::PolygonType && BodyB->getType() == BodyType::CircleType) ) //PolygonVsCircle	
			{
				Polygon* PolygonA = dynamic_cast<Polygon*>(BodyA);
				Circle* CircleB = dynamic_cast<Circle*>(BodyB);
				assert(PolygonA != nullptr);
				assert(CircleB != nullptr);
				PolygonVsCircle(PolygonA, CircleB, collisionMap, collison);
				return;
			}

			if ((BodyA->getType() == BodyType::CircleType && BodyB->getType() == BodyType::PolygonType)) //PolygonVsCircle	
			{
				Polygon* PolygonA = dynamic_cast<Polygon*>(BodyB);
				Circle* CircleB = dynamic_cast<Circle*>(BodyA);
				assert(PolygonA != nullptr);
				assert(CircleB != nullptr);
				PolygonVsCircle(PolygonA, CircleB, collisionMap, collison);
				return;
			}


			if (BodyA->getType() == BodyType::CircleType && BodyB->getType() == BodyType::CircleType) //CricleVsCircle
			{
				Circle* CircleA = dynamic_cast<Circle*>(BodyA);
				Circle* CircleB = dynamic_cast<Circle*>(BodyB);
				assert(CircleA != nullptr);
				assert(CircleB != nullptr);
				CircleVsCircle(CircleA, CircleB, collisionMap, collison);
				return;
			}

		}
	private:
		static void PolygonVsPolygon(Polygon* PolygonA, Polygon* PolygonB, std::unordered_map<uint32_t, Collision>& collisionMap, Collision& collisonReturn)
		{
			Sat satA;
			findMaxSeparation(PolygonA, PolygonB, satA);
			
			if (satA.MaxSeparation > 0)
			{
				return;
			}
			Sat satB;
			findMaxSeparation(PolygonB, PolygonA, satB);
			
			if (satB.MaxSeparation > 0)
			{
				return;
			}

			Collision collison;

			VerticesListType verticesWorldPos1;
			PolygonA->GetVerticesWorldPosition(verticesWorldPos1);

			VerticesListType verticesWorldPos2;
			PolygonB->GetVerticesWorldPosition(verticesWorldPos2);


			if (satA.MaxSeparation > satB.MaxSeparation)
			{
				collison.bodyA = PolygonA;
				collison.bodyB = PolygonB;
				collison.sat = satA;
				collison.Normal = collison.bodyA->edge(collison.sat.Edgeindex).normal();
				collison.Normal.normalize();
				collison.depth = std::abs(collison.sat.MaxSeparation);
			}
			else if (satA.MaxSeparation < satB.MaxSeparation)
			{
				collison.bodyA = PolygonB;
				collison.bodyB = PolygonA;
				collison.sat = satB;
				collison.Normal = collison.bodyA->edge(collison.sat.Edgeindex).normal();
				collison.Normal.normalize();
				collison.depth = std::abs(collison.sat.MaxSeparation);

			}
			else if (satA.MaxSeparation == satB.MaxSeparation)
			{
				auto AedgeLength = PolygonA->edge(satA.Edgeindex).lengthSquared();
				auto BedgeLength = PolygonB->edge(satB.Edgeindex).lengthSquared();

				if (AedgeLength < BedgeLength)
				{
					std::swap(PolygonA, PolygonB);
					std::swap(satA, satB);
					//std::cout << "AedgeLength < BedgeLength" << std::endl;
				}
				collison.bodyA = PolygonA;
				collison.bodyB = PolygonB;
				collison.sat = satA;
				collison.Normal = collison.bodyA->edge(collison.sat.Edgeindex).normal();
				collison.Normal.normalize();
				collison.depth = std::abs(collison.sat.MaxSeparation);
			}

			calcContactPoints(collison);

			/*
			auto id = makeId(collison.bodyA->id, collison.bodyB->id);
			if (collisionMap.find(id) != collisionMap.end())
			{
				assert(false);
			}
			collisionMap.emplace(makeId(collison.bodyA->id, collison.bodyB->id), collison);
			*/
			collisonReturn = collison;
			collisonReturn.isCollision = true;
			
		}

		static void CircleVsCircle(Circle* CircleA, Circle* CircleB, std::unordered_map<uint32_t, Collision>& collisionMap, Collision& collisonReturn)
		{
			Double centerDistance = Vec2::twoPointsDistance(CircleA->getCentroidWorldPos(), CircleB->getCentroidWorldPos());
		
			if (centerDistance > CircleA->getRadius() + CircleB->getRadius())
				return;

			Double depth = (CircleA->getRadius() + CircleB->getRadius() ) - centerDistance;
	

			Vec2 Na = CircleB->getCentroidWorldPos() - CircleA->getCentroidWorldPos();
			Na.normalize();

			Collision collison;
			collison.bodyA = CircleA;
			collison.bodyB = CircleB;
			//collison.sat = satB;
			collison.Normal = Na;
			collison.depth = depth;

			calcContactPoints(collison);

			/*
			auto id = makeId(collison.bodyA->id, collison.bodyB->id);
			if (collisionMap.find(id) != collisionMap.end())
			{
				assert(false);
			}
			collisionMap.emplace(makeId(collison.bodyA->id, collison.bodyB->id), collison);
			*/
			collisonReturn = collison;
			collisonReturn.isCollision = true;
			
		}

		static void PolygonVsCircle(Polygon* PolygonA, Circle* CircleB, std::unordered_map<uint32_t, Collision>& collisionMap, Collision& collisonReturn)
		{
			static int i = 0;
			if (i == 1)
			{
				i = 100;
			}
			i++;
			Sat sat;
			if (!findMaxSeparation(PolygonA, CircleB, sat)) return;
			
			Collision collison;
			if (sat.Bodytag == 0)
			{
				//std::cout << "sat.Bodytag == 0" <<std::endl;
				collison.bodyA = PolygonA;
				collison.bodyB = CircleB;
				collison.sat = sat;
				collison.Normal = collison.bodyA->edge(collison.sat.Edgeindex).normal() * sat.reverse;
				collison.Normal.normalize();
				//std::cout << "edge index: " << collison.sat.Edgeindex << std::endl;

				//std::cout << "Normal: " <<collison.Normal.x << " " << collison.Normal.y << std::endl;

				collison.depth = (collison.sat.MaxSeparation);
			}
			else
			{
				std::cout << "\n sat.Bodytag != 0" << std::endl;
				//assert(false);
			}

			calcContactPoints(collison);

			/*
			auto id = makeId(collison.bodyA->id, collison.bodyB->id);
			if (collisionMap.find(id) != collisionMap.end())
			{
				assert(false);
			}
			collisionMap.emplace(makeId(collison.bodyA->id, collison.bodyB->id), collison);
			*/
			collisonReturn = collison;
			collisonReturn.isCollision = true;
		

		}

		static void PolygonVsPolygonNew(Polygon* PolygonA, Polygon* PolygonB, std::unordered_map<uint32_t, Collision>& collisionMap)
		{
			Sat sat;
			if (!findMaxSeparation1(PolygonA, PolygonB, sat)) return;
			
			Collision collison;

			
			if (sat.Bodytag == 0)
			{
				collison.bodyA = PolygonA;
				collison.bodyB = PolygonB;
				collison.sat = sat;
				collison.Normal = collison.bodyA->edge(collison.sat.Edgeindex).normal();
				collison.Normal.normalize();
				collison.depth = (collison.sat.MaxSeparation);
			}
			
			else
			{
				collison.bodyA = PolygonB;
				collison.bodyB = PolygonA;
				collison.sat = sat;
				collison.Normal = collison.bodyA->edge(collison.sat.Edgeindex).normal();
				collison.Normal.normalize();
				collison.depth = (collison.sat.MaxSeparation);
			}

			

			calcContactPoints(collison);
			/*if ((collison.bodyA->id == 1 || collison.bodyA->id == 4) && (collison.bodyB->id == 1 || collison.bodyB->id == 4))
			{
				static int count = 0;
				count += 1;
				std::cout << "narrow phase collision count:" << count <<std::endl;
			}*/

			auto id = makeId(collison.bodyA->id, collison.bodyB->id);
			if (collisionMap.find(id) != collisionMap.end())
			{
				assert(false);
			}
			collisionMap.emplace(makeId(collison.bodyA->id, collison.bodyB->id), collison);

		}
	};
}