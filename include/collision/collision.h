#pragma once
#include "c_broad_phase.h"
#include "c_narrow_phase.h"
#include <gl/glut.h>
#include <unordered_map>
#include <iostream>
#include <algorithm>

namespace physicalEngine
{

	static class CollisionManager
	{
	public:
		static std::unordered_map<uint32_t, Collision> collisionMap;
		static void Detect(BVHTree* bvhTree)
		{
			//collisionMap.clear();

			std::vector<std::pair<Body*, Body*>> overlapedAABBObjects;
			BroadPhase::detectOverlapAABB(bvhTree, overlapedAABBObjects);
			
			if (overlapedAABBObjects.size() > 0)
			{
				for (auto& pair : overlapedAABBObjects)
				{
					Collision c;
					NarrowPhase::BodyCollisionDetect(pair.first, pair.second, collisionMap, c);
					if (c.isCollision)
					{
						addcollision(c);
					}
				}
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
				
				if (bodyA->isStatic  && bodyB->isStatic) continue;
				
				
				for (auto& contact : collision.contactListNew)
					{
						contact->ra = contact->pointA - bodyA->getCentroidWorldPos();
						contact->rb = contact->pointB - bodyB->getCentroidWorldPos();

						contact->staticFriction = std::sqrt(bodyA->getStaticFriction() * bodyB->getStaticFriction());
						contact->dynamicFriction = std::sqrt(bodyA->getDynamicFriction() * bodyB->getDynamicFriction());
						 
						Double jN;
						// 法向
						{
							auto dv = (bodyB->getVel() + contact->rb.defaultNormal() * bodyB->getAnguleVel()) -
								(bodyA->getVel() + contact->ra.defaultNormal() * bodyA->getAnguleVel());

							auto vn = dv.dot(collision.Normal);

							if (vn >= 0.0f)
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

							denom = denom <= 0.0000001f ? 0.f : 1.0 / denom;

							Double e = std::min(bodyA->getRestitution(), bodyB->getRestitution());
							Double j = -(1.f + e) * vn;
							j *= denom;

							j /= (Double)collision.contactList.size();

							jN = j;
							Vec2 impulse = collision.Normal * j;
							
							bodyA->setVel(bodyA->getVel() + impulse * -1 * bodyA->getMassInv());
							bodyA->setAnguleVel(bodyA->getAnguleVel() + -1 * contact->ra.cross(impulse) * bodyA->GetInvInertia());

							bodyB->setVel(bodyB->getVel() + impulse * bodyB->getMassInv());
							bodyB->setAnguleVel(bodyB->getAnguleVel() + contact->rb.cross(impulse) * bodyB->GetInvInertia());

						}

						// 切向
						{
							
							if (bodyA->getType() == BodyType::PolygonType && bodyB->getType() == BodyType::CircleType)
							{
								//std::cout << "debug";
							}
							auto dv = (bodyB->getVel() + contact->rb.defaultNormal() * bodyB->getAnguleVel()) -
								(bodyA->getVel() + contact->ra.defaultNormal() * bodyA->getAnguleVel());

							Vec2 vnT = dv - collision.Normal * dv.dot(collision.Normal);

						
							if (vnT.fuzzyEqual(Vec2(0.f, 0.f), 0.001f))
							{
								continue;
							}

							vnT.normalize();

							//auto tangent = collision.normal.defaultNormal();
							Vec2 raPerp = contact->ra.defaultNormal();
							Vec2 rbPerp = contact->rb.defaultNormal();

							Double raPerpDotT = raPerp.dot(vnT);
							Double rbPerpDotT = rbPerp.dot(vnT);

							Double denom = bodyA->getMassInv() + bodyB->getMassInv() +
								(raPerpDotT * raPerpDotT) * bodyA->GetInvInertia() +
								(rbPerpDotT * rbPerpDotT) * bodyB->GetInvInertia();
					

							denom = denom <= 0.0000001f ? 0.f : 1.0 / denom;

							Double jT = dv.dot(vnT) * -1;

							jT *= denom;

							jT /= (Double)collision.contactList.size();

							Vec2 firctionImpulse;

							if (std::abs(jT) <= std::abs(jN) * contact->staticFriction)
							{
								firctionImpulse =  vnT * jT;
								//firctionImpulse = vnT * jN * -1 * dynamicFriction;

							}
							else
							{
								//firctionImpulse = vnT * jT;
								firctionImpulse = vnT * jN * -1 * contact->dynamicFriction;
							}

							//firctionImpulse = vnT * jT;

							bodyA->setVel(bodyA->getVel() + firctionImpulse * -1 * bodyA->getMassInv());
							bodyA->setAnguleVel(bodyA->getAnguleVel() + -1 * contact->ra.cross(firctionImpulse) * bodyA->GetInvInertia());

							bodyB->setVel(bodyB->getVel() + firctionImpulse * bodyB->getMassInv());
							bodyB->setAnguleVel(bodyB->getAnguleVel() + contact->rb.cross(firctionImpulse) * bodyB->GetInvInertia());
						}
					}

			}


		}

		static void solveCollisionNew()
		{
			for (auto& k_v : collisionMap)
			{
				auto& collision = k_v.second;
				auto& bodyB = collision.bodyB;
				auto& bodyA = collision.bodyA;

				if (bodyA->isStatic && bodyB->isStatic) continue;

				for (auto& contact : collision.contactListNew)
				{
					Double jN;
					// 法向
					{
						auto dv = (bodyB->getVel() + contact->rb.defaultNormal() * bodyB->getAnguleVel()) -
							(bodyA->getVel() + contact->ra.defaultNormal() * bodyA->getAnguleVel());

						auto vn = dv.dot(collision.Normal);

						/*if (vn >= 0.0f)
						{
							continue;
						}*/
						/*
						if (std::fabs(vn) <= 0.001)
						{
							continue;
						}
						*/
						Double lambda_n =  vn * -1 + contact->velocityBias;
						//Double lambda_n = vn * (-1 - contact->restitution);

						lambda_n *= contact->effectiveMassNormal;
						//lambda_n /= collision.contactListNew.size();

						jN = lambda_n;

						Double oldImpulse = contact->accmulateImpulseNormal;

						contact->accmulateImpulseNormal = std::max(oldImpulse + lambda_n, Double(0.f));

						lambda_n = contact->accmulateImpulseNormal - oldImpulse;

						//std::cout << "lambda_n: " << lambda_n << std::endl;
						Vec2 impulse = collision.Normal * lambda_n;


						bodyA->applyImpulse(impulse * -1, contact->ra);
						bodyB->applyImpulse(impulse , contact->rb);

					}

					// 切向

					{
						auto dv = (bodyB->getVel() + contact->rb.defaultNormal() * bodyB->getAnguleVel()) -
							(bodyA->getVel() + contact->ra.defaultNormal() * bodyA->getAnguleVel());

						/*
						
						Vec2 vnT = dv - collision.Normal * dv.dot(collision.Normal);


						if (vnT.fuzzyEqual(Vec2(0.f, 0.f), 0.001f))
						{
							continue;
						}
						*/
						Double lambdaT = dv.dot(collision.Normal.normal()) * -1;

						lambdaT *= contact->effectiveMassTangent;

						Double maxFirctionImpulse = jN * contact->dynamicFriction;
						
						//lambdaT = std::clamp(lambdaT, -std::fabs(maxFirctionImpulse), std::fabs(maxFirctionImpulse));

						
						Double oldImpulse = (contact->accmulateImpulseTangent);

						contact->accmulateImpulseTangent = std::clamp(oldImpulse + lambdaT, -std::fabs(maxFirctionImpulse), std::fabs(maxFirctionImpulse));

						lambdaT = contact->accmulateImpulseTangent - oldImpulse;
						
						Vec2 firctionImpulse = collision.Normal.normal() * lambdaT;


						
						bodyA->applyImpulse(firctionImpulse * -1, contact->ra);
						bodyB->applyImpulse(firctionImpulse, contact->rb);
					}
				}
			}

		}

		static void solvePos()
		{
			for (auto& k_v : collisionMap)
			{
				auto& collision = k_v.second;
				auto& bodyA = collision.bodyA;
				auto& bodyB = collision.bodyB;
				
				if (bodyA->isStatic && bodyB->isStatic) continue;
				SeparateBodies(bodyA, bodyB, collision.Normal *  collision.depth);
				
			}
		}

		static void solvePosNew(Double& dt)
		{
			for (auto& k_v : collisionMap)
			{
				auto& collision = k_v.second;
				if (!collision.active)
				{
					std::cout << "collision not active" << std::endl;
				}
				auto& bodyA = collision.bodyA;
				auto& bodyB = collision.bodyB;

				if (bodyA->isStatic && bodyB->isStatic) continue;

				for (auto& contact : collision.contactListNew)
				{
					if (!contact->active)
					{
						std::cout << "contact not active" << std::endl;
					}
					Vec2 pa = bodyA->toWorldPoint(contact->localA);
					Vec2 pb = bodyB->toWorldPoint(contact->localB);
					Vec2 c = pb - pa;
					Double bias = 0.02* std::max(double(0.005f), c.length() - 0.005f);
					//Double bias = std::max(double(0.f), collision.depth - 0.005f);

					Double lambda = bias * contact->effectiveMassNormal;
					// lambda /= collision.contactListNew.size();
					Vec2 impulse = collision.Normal * lambda;

					if (!bodyA->isStatic)
					{
						bodyA->move(impulse * bodyA->getMassInv() * -1);
						//bodyA->setAngle(bodyA->getAngle() - impulse.cross(pa - bodyA->getCentroidWorldPos()) * bodyA->GetInvInertia());
					}
					if (!bodyB->isStatic )
					{
						bodyB->move(impulse * bodyB->getMassInv());
						//bodyB->setAngle(bodyB->getAngle() + impulse.cross(pb - bodyB->getCentroidWorldPos()) * bodyB->GetInvInertia());
					}

				}
			}
		}

		static void drawContacts()
		{
			for (auto& k_v : collisionMap)
			{
				for (auto& contact : k_v.second.contactListNew)
				{
					//std::cout << contact->ra.x << contact->ra.y << std::endl;
					
					glBegin(GL_LINE_LOOP);
					glLineWidth(4.0f);
					glColor3f(1.f, 1.f, 1.f);
					glVertex2d(k_v.second.bodyA->getCentroidWorldPos().x * 10, k_v.second.bodyA->getCentroidWorldPos().y * 10);
					glVertex2d(k_v.second.bodyA->getCentroidWorldPos().x * 10 + contact->ra.x * 10, k_v.second.bodyA->getCentroidWorldPos().y * 10 + contact->ra.y * 10);
					glEnd();

					glBegin(GL_LINE_LOOP);
					glLineWidth(4.0f);
					glColor3f(1.f, 1.f, 1.f);
					glVertex2d(k_v.second.bodyB->getCentroidWorldPos().x * 10, k_v.second.bodyB->getCentroidWorldPos().y * 10);
					glVertex2d(k_v.second.bodyB->getCentroidWorldPos().x * 10 + contact->rb.x * 10, k_v.second.bodyB->getCentroidWorldPos().y * 10 + contact->rb.y * 10);
					glEnd();
					
					glColor3f(1.0f, 0.0f, 0.0f);    //设置绘图颜色
					glRectf(contact->pointA.x * 10 - 2.f, contact->pointA.y * 10 - 2.f, contact->pointA.x * 10 + 2.f, contact->pointA.y * 10 + 2.f);  //绘制矩形

					glColor3f(0.0f, 0.0f, 1.0f);    //设置绘图颜色
					glRectf(contact->pointB.x * 10 - 2.f, contact->pointB.y * 10 - 2.f, contact->pointB.x * 10 + 2.f, contact->pointB.y * 10 + 2.f);  //绘制矩形

				}
			}
		}

		// 每一帧计算step 碰撞Detect后调用
		static void clearInactiveCollisionAndContactpointPoint()
		{
			// 清除 not active collision
			std::erase_if(collisionMap, [](const auto& item)
			{
					auto const& [key, value] = item;
					return !value.active;
			});

			// 清除 not active 接触点对
			for (auto&& iter : collisionMap)
			{
				auto& collision = iter.second;
				auto& contactList = collision.contactListNew;
				std::erase_if(contactList, [](const Contact* contact)
					{
						return !contact->active;
					});
			}
		}

		// 每一帧step最后调用
		static void deactivateCollisionAndContactpoint()
		{
			for (auto iter = collisionMap.begin(); iter != collisionMap.end(); ++iter)
			{
				iter->second.active = false; // 设置碰撞 active =false;

				for (auto& contact : iter->second.contactListNew)
				{
					contact->active = false; // 设置 接触点对 active =false;
				}
			}
		}

		static void addcollision(const Collision& collision)
		{
			

			auto id = makeId(collision.bodyA->id, collision.bodyB->id);

			if (collisionMap.find(id) == collisionMap.end())  // 上一帧两物体没有碰撞
			{
				collisionMap.emplace(id, collision);
				return;
			}
		    // 上一帧两物体碰撞

			auto& collisionOld = collisionMap[id];
			if (collisionOld.bodyA != collision.bodyA)  // 上一帧两物体碰撞,但现在
			{
				collisionMap.emplace(id, collision);
				return;
			}

			collisionOld.active = true; //激活碰撞, 更新碰撞信息
			collisionOld.depth = collision.depth;
			collisionOld.Normal = collision.Normal;
			
			/*
			if (collisionOld.bodyA != collision.bodyA) // 交换 A, B TODO
			{
				if (collisionOld.bodyA != collision.bodyB || collisionOld.bodyB != collision.bodyA)
				{
					std::cout << "替换错误 不是一个collision" << std::endl;

				}
				collisionOld.bodyA = collision.bodyA;
				collisionOld.bodyB = collision.bodyB;
			}
			*/

			auto& bodyA = collision.bodyA;
			auto& bodyB = collision.bodyB;


			for (const auto& contactNew : collision.contactListNew)
			{
				bool flag = false;

				//Vec2 localA = bodyA->toLocalPoint(contactNew->pointA); // todo point a
				//Vec2 localB = bodyB->toLocalPoint(contactNew->pointB); // todo point B

				for (auto& contactOld : collisionOld.contactListNew)
				{
					const bool isPointA = contactNew->pointA.fuzzyEqual(bodyA->toWorldPoint(contactOld->localA), 0.1f);
					const bool isPointB = contactNew->pointB.fuzzyEqual(bodyB->toWorldPoint(contactOld->localB), 0.1f);

					if (isPointA && isPointB)  // 激活碰撞点对,更新接触点信息
					{
						contactOld->pointA = contactNew->pointA;
						contactOld->pointB = contactNew->pointB;
						contactOld->active= true;
						flag = true;
						break;
					}
				}

				

				if (flag) continue;

				collisionOld.contactListNew.emplace_back(contactNew); //添加新的碰撞点对
			}
		}

		public:
		static void perpareContact()
		{
			for (auto& k_v : collisionMap)
			{
				auto& collision = k_v.second;
				auto& bodyA = collision.bodyA;
				auto& bodyB = collision.bodyB;

				for (auto& contact : collision.contactListNew)
				{
					contact->localA = bodyA->toLocalPoint(contact->pointA); 
					contact->localB = bodyB->toLocalPoint(contact->pointB);

					contact->ra = contact->pointA - bodyA->getCentroidWorldPos();
					contact->rb = contact->pointB - bodyB->getCentroidWorldPos();

					contact->staticFriction = std::sqrt(bodyA->getStaticFriction() * bodyB->getStaticFriction());
					contact->dynamicFriction = std::sqrt(bodyA->getDynamicFriction() * bodyB->getDynamicFriction());

					{
						Vec2 raPerp = contact->ra.defaultNormal();
						Vec2 rbPerp = contact->rb.defaultNormal();

						Double raPerpDotN = raPerp.dot(collision.Normal);
						Double rbPerpDotN = rbPerp.dot(collision.Normal);

						Double denom = bodyA->getMassInv() + bodyB->getMassInv() +
							(raPerpDotN * raPerpDotN) * bodyA->GetInvInertia() +
							(rbPerpDotN * rbPerpDotN) * bodyB->GetInvInertia();

						denom = denom <= 0.0000001f ? 0.f : 1.0 / denom;

						contact->effectiveMassNormal = denom;
					}

					{
						Vec2 vnT = collision.Normal.normal();
						Vec2 raPerp = contact->ra.defaultNormal();
						Vec2 rbPerp = contact->rb.defaultNormal();

						Double raPerpDotT = raPerp.dot(vnT);
						Double rbPerpDotT = rbPerp.dot(vnT);

						Double denom = bodyA->getMassInv() + bodyB->getMassInv() +
							(raPerpDotT * raPerpDotT) * bodyA->GetInvInertia() +
							(rbPerpDotT * rbPerpDotT) * bodyB->GetInvInertia();


						denom = denom <= 0.0000001f ? 0.f : 1.0 / denom;

						contact->effectiveMassTangent = denom;
					}

					{
						contact->restitution = std::min(bodyA->getRestitution(), bodyB->getRestitution());

						Vec2 wa = contact->ra.defaultNormal() * bodyA->getAnguleVel();
						Vec2 wb = contact->rb.defaultNormal() * bodyB->getAnguleVel();

						contact->va = bodyA->getVel() + wa;
						contact->vb = bodyB->getVel() + wb;

						auto dv = contact->vb - contact->va;
						auto vn = dv.dot(collision.Normal);
						contact->velocityBias = vn * contact->restitution * -1;
					}

					// storage impulse
					
					{
						Vec2 impulse = collision.Normal * contact->accmulateImpulseNormal + collision.Normal.normal()* contact->accmulateImpulseTangent;
						bodyA->applyImpulse(impulse * -1, contact->ra);
						bodyB->applyImpulse(impulse, contact->rb);
					}
					
				}
			}
		}
	};

	std::unordered_map<uint32_t, Collision> CollisionManager::collisionMap;

}