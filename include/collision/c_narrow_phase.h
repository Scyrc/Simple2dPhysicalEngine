#pragma once
#include "../common/common.h"
#include <unordered_map>

namespace physicalEngine
{

	class NarrowPhase
	{
	public:
		static void BodyCollisionDetect(Body* BodyA, Body* BodyB, std::unordered_map<uint32_t, Collision>& collisionMap)
		{
			if (BodyA->getType() == BodyType::PolygonType && BodyB->getType() == BodyType::PolygonType)
			{
				Polygon* PolygonA = dynamic_cast<Polygon*>(BodyA);
				Polygon* PolygonB = dynamic_cast<Polygon*>(BodyB);
				assert(PolygonA != nullptr);
				assert(PolygonB != nullptr);
				PolygonVsPolygon(PolygonA, PolygonB, collisionMap);
			}
		}
	private:
		static void PolygonVsPolygon(Polygon* PolygonA, Polygon* PolygonB, std::unordered_map<uint32_t, Collision>& collisionMap)
		{
			/*if ((PolygonA->id == 1 || PolygonA->id == 4) && (PolygonB->id == 1 || PolygonB->id == 4))
			{
				static int count1 = 0;
				count1 += 1;
				std::cout << "broad phase collision count:" << count1 << std::endl;
				if (count1 == 1218)
				{
					std::cout << "broad phase collision count:" << 1218 << std::endl;

				}
			}*/
			
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
		/*	static int count = 0;
			count += 1;
			std::cout << "----------------------------" << count << std::endl;
			std::cout << "检测到碰撞 count = " << count << std::endl;
			std::cout << "碰撞的两个body :body id =" << PolygonA->id << " and body id =" << PolygonB->id << std::endl;*/
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
			else
			{
				collison.bodyA = PolygonB;
				collison.bodyB = PolygonA;
				collison.sat = satB;
				collison.Normal = collison.bodyA->edge(collison.sat.Edgeindex).normal();
				collison.Normal.normalize();
				collison.depth = std::abs(collison.sat.MaxSeparation);

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