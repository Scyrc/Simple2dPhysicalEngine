#pragma once

#include "joint.h"
#include "body.h"
#include <gl/glut.h>

namespace physicalEngine
{
	struct DistancePointPrimitive
	{
		Body* bodyA = nullptr;
		Vec2 localPointA;
		Vec2 targetPoint;
		Vec2 normal;
		Double biasFactor = 1.f;
		Double bias = 0.0f;
		Double minDistance = 0.0f;
		Double maxDistance = 0.0f;
		Double effectiveMass = 0.0f;
		Double accumulatedImpulse = 0.0f;

	};


	struct DistanceConstraintPrimitive
	{
		Body* bodyA = nullptr;
		Body* bodyB = nullptr;
		Vec2 localPointA;
		Vec2 localPointB;
		Vec2 ra;
		Vec2 rb;

		Vec2 bias;
	
		Mat22 effectiveMass;
		Vec2 Impulse;
		Double maxForce = 200.0f;
	};

	class DistanceJoint : public Joint
	{
	public:
		DistanceJoint() { jointType = JointType::Distance; }
		DistanceJoint(const DistancePointPrimitive& primitive_) :primitive(primitive_) { jointType = JointType::Distance; }


		virtual void prepare(const Double& dt) override
		{
			assert(primitive.minDistance <= primitive.maxDistance);

			auto& bodyA = primitive.bodyA;
			Vec2 pa = bodyA->toWorldPoint(primitive.localPointA);
			Vec2 pb = primitive.targetPoint;

			Double im_a = primitive.bodyA->getMassInv();
			Double ii_a = primitive.bodyA->GetInvInertia();

			Vec2 currentDirectio = pb - pa;
			Double length = currentDirectio.length();

			Double c = 0.f;

			primitive.normal = currentDirectio;
			primitive.normal.normalize();

			if (length < primitive.minDistance)
			{
				c = primitive.minDistance - length;
				primitive.normal = primitive.normal * -1;
			}
			else if (length > primitive.maxDistance)
			{
				c = length - primitive.maxDistance;
			}
			else
			{
				primitive.accumulatedImpulse = 0.f;
				primitive.normal = Vec2(0.f, 0.f);
				primitive.bias = 0.f;
				return;
			}

			if (primitive.bodyA->getVel().dot(primitive.normal) > 0)
			{
				primitive.accumulatedImpulse = 0.f;
				primitive.normal = Vec2(0.f, 0.f);
				primitive.bias = 0.f;
				return;
			}
			Vec2 ra = pa - bodyA->getCentroidWorldPos();
			Double ra_n = primitive.normal.dot(ra);

			primitive.effectiveMass = 1.0f / (im_a + ii_a * ra_n * ra_n);
			primitive.bias = primitive.biasFactor * c / dt;
		}


		virtual void solveVelocity(const Double& dt) override
		{
			if (primitive.bias == 0.f) return;
			auto& bodyA = primitive.bodyA;
			Vec2 ra =bodyA->toWorldPoint(primitive.localPointA) - bodyA->getCentroidWorldPos();

			Vec2 va = bodyA->getVel() + ra.defaultNormal() * bodyA->getAnguleVel();

			Vec2 dv = va;

			Double jv = primitive.normal.dot(dv);

			Double jvb = jv * -1 + primitive.bias;

			Double lambda_n = jvb * primitive.effectiveMass;
			Double oldImpulse = primitive.accumulatedImpulse;
			primitive.accumulatedImpulse = std::max(oldImpulse + lambda_n, Double(0.f));
			lambda_n = primitive.accumulatedImpulse - oldImpulse;

			Vec2 impulse = primitive.normal * lambda_n;

			primitive.bodyA->applyImpulse(impulse, ra);

		}


		virtual void solvePosition(const Double& dt) override
		{
			
		}
		virtual void Draw() override
		{
			glBegin(GL_LINE_LOOP);
			glLineWidth(3.0f);
			glColor3f(1.f, 1.f, 1.f);
			
			
			glVertex2d(primitive.targetPoint.x * 10, primitive.targetPoint.y * 10);

			glVertex2d(primitive.bodyA->toWorldPoint(primitive.localPointA).x * 10, primitive.bodyA->toWorldPoint(primitive.localPointA).y * 10);

			glEnd();


			glColor3f(252 / 255.f, 161 / 255.f, 4 / 255.f);   
			glRectf(primitive.targetPoint.x * 10 - 1.5f, primitive.targetPoint.y * 10 - 1.5f, primitive.targetPoint.x * 10 + 1.5f, primitive.targetPoint.y * 10 + 1.5f);  //绘制矩形


			glColor3f(252 / 255.f, 161 / 255.f, 4 / 255.f);
			glRectf(primitive.bodyA->toWorldPoint(primitive.localPointA).x * 10 - 1.5f, primitive.bodyA->toWorldPoint(primitive.localPointA).y * 10 - 1.5f, primitive.bodyA->toWorldPoint(primitive.localPointA).x * 10 + 1.5f, primitive.bodyA->toWorldPoint(primitive.localPointA).y * 10 + 1.5f);  //绘制矩形

		}


	private:
		Double factor = 0.4f;
		DistancePointPrimitive primitive;
	};


	class DistanceConstraintJoint : public Joint
	{
	public:
		DistanceConstraintJoint() = default;

		DistanceConstraintJoint(DistanceConstraintPrimitive primitive_) :primitive(primitive_) { jointType = JointType::Distance; }
	

		void Draw() override
		{
			
		}


		void prepare(const Double& dt) override
		{
			if (primitive.bodyA == nullptr || primitive.bodyB == nullptr) return;

			auto& bodyA = primitive.bodyA;
			auto& bodyB = primitive.bodyB;

			Double im_a = bodyA->getMassInv();
			Double ii_a = bodyA->GetInvInertia();

			Double im_b = bodyB->getMassInv();
			Double ii_b = bodyB->GetInvInertia();

			primitive.ra = bodyA->toWorldPoint(primitive.localPointA) - bodyA->getCentroidWorldPos();
			primitive.rb = bodyB->toWorldPoint(primitive.localPointB) - bodyB->getCentroidWorldPos();
			auto& ra = primitive.ra;
			auto& rb = primitive.rb;

			Vec2 currentDirection = bodyA->toWorldPoint(primitive.localPointA) - bodyB->toWorldPoint(primitive.localPointB);

			

			Double m11 = im_a + ra.y * ra.y * ii_a + im_b + rb.y * rb.y * ii_b;
			Double m12 = -ra.x * ra.y * ii_a - rb.x * rb.y * rb.y * ii_b;
			Double m21 = m12;
			Double m22 = im_a + ra.x * ra.x * ii_a + im_b + rb.x * rb.x * ii_b;

			Mat22 K{ m11, m12, m21, m22 };

			primitive.bias = currentDirection * factor;
			primitive.effectiveMass = K.invert();
		}


		void solveVelocity(const Double& dt) override
		{
			if (primitive.bodyA == nullptr || primitive.bodyB == nullptr) return;
			auto& bodyA = primitive.bodyA;
			auto& bodyB = primitive.bodyB;

			Vec2 va = bodyA->getVel() + primitive.ra.defaultNormal() * bodyA->getAnguleVel();
			Vec2 vb = bodyB->getVel() + primitive.rb.defaultNormal() * bodyB->getAnguleVel();

			Vec2 jvb = va - vb;
			jvb += primitive.bias;

			jvb = jvb * -1;
			Vec2 J = primitive.effectiveMass * jvb;
			Vec2 oldImpulse = primitive.Impulse;
			primitive.Impulse += J;

			Double maxImpulse = dt * primitive.maxForce;

			if (primitive.Impulse.lengthSquared() > maxImpulse * maxImpulse)
			{
				primitive.Impulse.normalize();
				primitive.Impulse = primitive.Impulse * maxImpulse;
			}

			J = primitive.Impulse - oldImpulse;

			primitive.bodyA->applyImpulse(J, primitive.ra);
			primitive.bodyB->applyImpulse(J * -1, primitive.rb);

		}


		void solvePosition(const Double& dt) override
		{
			
		}

	protected:
		DistanceConstraintPrimitive primitive;
		Double factor = 0.1f;

	};
}