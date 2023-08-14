#pragma once
#include "../common/ctypes.h"
#include "../body/body.h"
#include <unordered_map>
#include <iostream>
namespace physicalEngine
{
	typedef struct VelocityConstrainyPoint
	{
		Vec2 ra;
		Vec2 rb;

		Vec2 va;
		Vec2 vb;

		Vec2 normal;
		Vec2 tangent;

		Vec2 velocityBias;

		Double bias = 0;
		Double penetration = 0.f;

		Double restitution = 0.8f;

		Double effectiveMassNormal = 0.f;
		Double effectiveMassTangent = 0.f;

		Double accmulateImpulseNormal = 0.f;
		Double accmulateImpulseTangent = 0.f;

	};

	typedef struct ContactConstraintPoint
	{

		ContactConstraintPoint()
		{

		}
		ContactConstraintPoint(Vec2& point_)
		{
			point = point_;
		}

		ContactConstraintPoint(Vec2& pointa, Vec2& pointb)
		{
			pointA = pointa;
			pointB = pointb;
		}
		uint32_t id;
		Vec2 normal;
		Vec2 point;
		Vec2 pointA;
		Vec2 pointB;

		Vec2 localA;
		Vec2 localB;
		Vec2 ra;
		Vec2 rb;
		bool active = false;
		Double firction;
		Body* bodyA = nullptr;
		Body* bodyB = nullptr;
		VelocityConstrainyPoint vcp;
	};


	typedef struct Contact
	{
		Contact(Vec2& point_)
		{
			point = point_;
		}

		Contact(Vec2& pointa, Vec2& pointb)
		{
			pointA = pointa;
			pointB = pointb;
		}
		Vec2 point;
		Vec2 pointA;
		Vec2 pointB;

		Vec2 localA;
		Vec2 localB;
		Vec2 ra;
		Vec2 rb;
		bool active = true;
		Double firction;
		Body* bodyA = nullptr;
		Body* bodyB = nullptr;
		VelocityConstrainyPoint vcp;
		Double staticFriction = 0.8;
		Double dynamicFriction = 0.5;
		Double effectiveMassNormal = 0.f;
		Double effectiveMassTangent = 0.f;

		Double accmulateImpulseNormal = 0.f;
		Double accmulateImpulseTangent = 0.f;
		Double velocityBias;
		Double restitution = 1.f;

		Vec2 va;
		Vec2 vb;
	};

	typedef struct Sat
	{
		int Edgeindex;
		Double MaxSeparation;
		int Bodytag = -1;
		int reverse = 1;
		Sat() :Edgeindex(-1), MaxSeparation(DoubleMin) {}
	};

	typedef struct Collision
	{
		bool isCollision = false;
		bool active = true;
		Body* bodyA;
		Body* bodyB;
		Vec2 Normal;
		Sat sat;
		Double depth;
		Double penetration;
		std::vector<Contact*> contactList;
		std::vector<Contact*> contactListNew;
	};
	/*
	
	class ContactManager
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
				SeparateBodies(bodyA, bodyB, collision.Normal * collision.depth);
				//std::cout << "collision.depth" << collision.depth << std::endl;
				for (int xxx = 0; xxx < 10; xxx++)
				{
					for (auto& contact : collision.contactList)
					{
						contact->ra = contact->point - bodyA->getCentroidWorldPos();
						contact->rb = contact->point - bodyB->getCentroidWorldPos();
						//std::cout << contact->ra.x << contact->ra.y << std::endl;

						float staticFriction = (bodyA->getStaticFriction() * bodyB->getStaticFriction());
						float dynamicFriction = (bodyA->getDynamicFriction() * bodyB->getDynamicFriction());

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
	/*
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
								firctionImpulse = vnT * jT;
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

		static void solveCollisionNew()
		{
			for (auto& k_v : collisionMap)
			{
				auto& collision = k_v.second;
				auto& bodyB = collision.bodyB;
				auto& bodyA = collision.bodyA;

				if (bodyA->isStatic && bodyB->isStatic) continue;

				for (auto& contact : collision.contactList)
				{
					contact->ra = contact->point - bodyA->getCentroidWorldPos();
					contact->rb = contact->point - bodyB->getCentroidWorldPos();


					auto& vcp = contact->vcp;

					Vec2 wa = contact->ra.defaultNormal() * bodyA->getAnguleVel();
					Vec2 wb = contact->rb.defaultNormal() * bodyB->getAnguleVel();

					vcp.va = bodyA->getVel() + wa;
					vcp.vb = bodyB->getVel() + wb;

					auto dv = vcp.va - vcp.vb;

					Double jv = vcp.normal.dot(dv) * -1.0f + vcp.bias;

					float staticFriction = (bodyA->getStaticFriction() * bodyB->getStaticFriction());
					float dynamicFriction = (bodyA->getDynamicFriction() * bodyB->getDynamicFriction());

					Double jN;
					// 法向
					{


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
					}


				}

			}

		}

		static void drawContacts()
		{
			for (auto& k_v : collisionMap)
			{
				auto& contacts = k_v.second.contactList;
				for (auto contact : contacts)
				{
					//std::cout << contact->ra.x << contact->ra.y << std::endl;

					glBegin(GL_LINE_LOOP);
					glLineWidth(4.0f);
					glColor3f(1.f, 1.f, 1.f);
					glVertex2d(k_v.second.bodyA->getCentroidWorldPos().x, k_v.second.bodyA->getCentroidWorldPos().y);
					glVertex2d(k_v.second.bodyA->getCentroidWorldPos().x + contact->ra.x, k_v.second.bodyA->getCentroidWorldPos().y + contact->ra.y);
					glEnd();

					glBegin(GL_LINE_LOOP);
					glLineWidth(4.0f);
					glColor3f(1.f, 1.f, 1.f);
					glVertex2d(k_v.second.bodyB->getCentroidWorldPos().x, k_v.second.bodyB->getCentroidWorldPos().y);
					glVertex2d(k_v.second.bodyB->getCentroidWorldPos().x + contact->rb.x, k_v.second.bodyB->getCentroidWorldPos().y + contact->rb.y);
					glEnd();

					glColor3f(1.0f, 0.0f, 0.0f);    //设置绘图颜色
					glRectf(contact->point.x - 2.f, contact->point.y - 2.f, contact->point.x + 2.f, contact->point.y + 2.f);  //绘制矩形
				}
			}
		}
	};
	*/
	
}