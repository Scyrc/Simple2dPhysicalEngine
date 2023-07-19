#pragma once
#include "c_broad_phase.h"
#include <iostream>
#include "../common/common.h"
#include <unordered_map>
#include "c_narrow_phase.h"
#include <gl/glut.h>

namespace physicalEngine
{
	static class CollisionManager
	{
	public:
		static std::unordered_map<uint32_t, Collision> collisionMap;
		static int count;
		static void Detect(BVHTree* bvhTree)
		{
			collisionMap.clear();

			std::vector<std::pair<Body*, Body*>> overlapedAABBObjects;
			BroadPhase::detectOverlapAABB(bvhTree, overlapedAABBObjects);

			if (overlapedAABBObjects.size() > 0)
			{
				for (auto& pair : overlapedAABBObjects)
				{
					NarrowPhase::BodyCollisionDetect(pair.first, pair.second, collisionMap);
				}

				//std::cout << "----------------------------" << count << std::endl;
			}
		}
		static void SeparateBodies(Body* bodyA, Body* bodyB, Vec2 mtv)
		{
			if (bodyA->isStatic)
			{
				bodyB->move(mtv);
			}
			else if (bodyB->isStatic)
			{
				bodyA->move(mtv * -1);
			}
			else
			{
				bodyA->move(mtv / -2);
				bodyB->move(mtv / 2);
			}
		}
		static void solveCollision()
		{
			for (auto& k_v : collisionMap)
			{
				auto& collision = k_v.second;
				auto& bodyB = collision.bodyB;
				auto& bodyA = collision.bodyA;
				if (bodyA->isStatic && bodyB->isStatic) continue;;
				SeparateBodies(bodyA, bodyB, collision.Normal *  collision.depth);
				//std::cout << "collision.depth" << collision.depth << std::endl;
				for (int xxx = 0; xxx < 2; xxx++)
				{
					for (auto& contact : collision.contactList)
					{
						contact->ra = contact->point - bodyA->getCentroidWorldPos();
						contact->rb = contact->point - bodyB->getCentroidWorldPos();

						float staticFriction = (bodyA->getStaticFriction() + bodyB->getStaticFriction()) * 0.5f;
						float dynamicFriction = (bodyA->getDynamicFriction() + bodyB->getDynamicFriction()) * 0.5f;
						 
						Double jN;
						// 法向
						{
							auto dv = (bodyB->getVel() + contact->rb.defaultNormal() * bodyB->getAnguleVel()) -
								(bodyA->getVel() + contact->ra.defaultNormal() * bodyA->getAnguleVel());

							auto vn = dv.dot(collision.Normal);

							if (vn >= 0)
							{
								//std::cout << "vn > 0"  << std::endl;
								continue;
							}
							//std::cout << "vn < 0" << std::endl;

							Vec2 raPerp = contact->ra.defaultNormal();
							Vec2 rbPerp = contact->rb.defaultNormal();

							Double raPerpDotN = raPerp.dot(collision.Normal);
							Double rbPerpDotN = rbPerp.dot(collision.Normal);

							Double denom = bodyA->getMassInv() + bodyB->getMassInv() +
								(raPerpDotN * raPerpDotN) * bodyA->GetInvInertia() +
								(rbPerpDotN * rbPerpDotN) * bodyB->GetInvInertia();
							Double e = std::min(bodyA->getRestitution(), bodyB->getRestitution());
							Double j = -(1.f + e) * vn;
							j /= denom;

							j /= (Double)collision.contactList.size();

							jN = j;
							Vec2 impulse = collision.Normal * j;
							/*if (bodyA->isStatic)
							{
								bodyB->setVel(bodyB->getVel() + impulse * bodyB->getMassInv() * 2);
								bodyB->setAnguleVel(bodyB->getAnguleVel() + contact->rb.cross(impulse) * bodyB->GetInvInertia());
							}
							else if (bodyB->isStatic)
							{
								bodyA->setVel(bodyA->getVel() + impulse * -2 * bodyA->getMassInv());
								bodyA->setAnguleVel(bodyA->getAnguleVel() + -1 * contact->ra.cross(impulse) * bodyA->GetInvInertia());
							}*/
							bodyA->setVel(bodyA->getVel() + impulse * -1 * bodyA->getMassInv());
							bodyA->setAnguleVel(bodyA->getAnguleVel() + -1 * contact->ra.cross(impulse) * bodyA->GetInvInertia());

							bodyB->setVel(bodyB->getVel() + impulse * bodyB->getMassInv());
							bodyB->setAnguleVel(bodyB->getAnguleVel() + contact->rb.cross(impulse) * bodyB->GetInvInertia());

					}

						// 切向
						{
							
							auto dv = (bodyB->getVel() + contact->rb.defaultNormal() * bodyB->getAnguleVel()) -
								(bodyA->getVel() + contact->ra.defaultNormal() * bodyA->getAnguleVel());

							Vec2 vnT = dv - collision.Normal * dv.dot(collision.Normal);

							if (Vec2::NearlyEqual(vnT, Vec2(DBL_MIN, DBL_MIN)))
							{
								continue;
							}

							vnT.normalize();

							Vec2 raPerp = contact->ra.defaultNormal();
							Vec2 rbPerp = contact->rb.defaultNormal();

							Double raPerpDotT = raPerp.dot(vnT);
							Double rbPerpDotT = rbPerp.dot(vnT);

							Double denom = bodyA->getMassInv() + bodyB->getMassInv() +
								(raPerpDotT * raPerpDotT) * bodyA->GetInvInertia() +
								(rbPerpDotT * rbPerpDotT) * bodyB->GetInvInertia();

							
							Double jT = dv.dot(vnT) * -1;
							jT /= denom;

							jT /= (Double)collision.contactList.size();

							Vec2 firctionImpulse;

							if (std::abs(jT) <= std::abs(jN) * staticFriction)
							{
								firctionImpulse =  vnT * jT;
							}
							else
							{
								firctionImpulse = vnT * jN * -1 * dynamicFriction;
							}


							bodyA->setVel(bodyA->getVel() + firctionImpulse * -1 * bodyA->getMassInv());
							bodyA->setAnguleVel(bodyA->getAnguleVel() + -1 * contact->ra.cross(firctionImpulse) * bodyA->GetInvInertia());

							bodyB->setVel(bodyB->getVel() + firctionImpulse * bodyB->getMassInv());
							bodyB->setAnguleVel(bodyB->getAnguleVel() + contact->rb.cross(firctionImpulse) * bodyB->GetInvInertia());
						}
						
					}

				}
			}


		}

		static void drawContacts()
		{
			for (auto& k_v : collisionMap)
			{
				auto contacts = k_v.second.contactList;
				for (auto contact : contacts)
				{
					glColor3f(1.0f, 0.0f, 0.0f);    //设置绘图颜色
					glRectf(contact ->point.x - 3.f, contact->point.y - 3.f, contact->point.x + 3.f, contact->point.y + 3.f);  //绘制矩形

					/*Vec2 normalLine = k_v.second.Normal * k_v.second.depth;
					normalLine = normalLine * 10;
					glBegin(GL_LINE_LOOP);
					glLineWidth(10.0f);
					glColor3f(1.f, 1.f, 1.f);
					glVertex2d(contact->point.x, contact->point.y);
					glVertex2d(contact->point.x + normalLine.x, contact->point.y + normalLine.y);

					glEnd();*/

				}
			}
		}
	};

	int CollisionManager::count = 0;
	std::unordered_map<uint32_t, Collision> CollisionManager::collisionMap;
}