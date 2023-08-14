#pragma once

#include "joint.h"
#include "body.h"
#include <gl/glut.h>

namespace physicalEngine
{

	struct RevoluteConstraintPrimitive
	{
		Body* bodyA = nullptr;
		Body* bodyB = nullptr;
		Vec2 localPointA;
		Vec2 localPointB;
		
		Double damping = 0.f;
		Double stiffness = 0.f;
		Double frequency = 8.f;
		Double maxForce = 5000.0f;
		Double dampingRatio = 0.2f;
		Double gamma = 0.0f;

		Vec2 bias;
		Mat22 effectiveMass;
		Vec2 accumulatedImpulse;
	};


	class RevoluteConstraintJoint : public Joint
	{
	public:
		RevoluteConstraintJoint() { jointType = JointType::Revolute; };

		RevoluteConstraintJoint(RevoluteConstraintPrimitive primitive_) :primitive(primitive_) { jointType = JointType::Revolute; }


		void Draw() override
		{

		}


		void prepare(const Double& dt) override
		{
			if (primitive.bodyA == nullptr || primitive.bodyB == nullptr) return;

			auto& bodyA = primitive.bodyA;
			auto& bodyB = primitive.bodyB;
			Double ma = bodyA->getMass();
			Double im_a = bodyA->getMassInv();
			Double ii_a = bodyA->GetInvInertia();

			Double mb = bodyB->getMass();
			Double im_b = bodyB->getMassInv();
			Double ii_b = bodyB->GetInvInertia();

			if (primitive.frequency > 0.0f)
			{
				Double nf = naturalFrequency(primitive.frequency);
				primitive.stiffness = springStiffness(ma + mb, nf);
				primitive.damping = springDampingCoefficient(ma + mb, nf, primitive.dampingRatio);
			}
			else
			{
				primitive.stiffness = 0.f;
				primitive.damping = 0.f;
			}

			primitive.gamma = constraintImpulseMixing(dt, primitive.stiffness, primitive.damping);
			Double erp = errorReductionParameter(dt, primitive.stiffness, primitive.damping);

			Vec2 pa = bodyA->toWorldPoint(primitive.localPointA);
			Vec2 ra = pa - bodyA->getCentroidWorldPos();

			Vec2 pb = bodyB->toWorldPoint(primitive.localPointB);
			Vec2 rb = pb - bodyB->getCentroidWorldPos();

			primitive.bias = (pa - pb) * erp;


			Double m11 = im_a + ra.y * ra.y * ii_a + im_b + rb.y * rb.y * ii_b;
			Double m12 = -ra.x * ra.y * ii_a - rb.x * rb.y * rb.y * ii_b;
			Double m21 = m12;
			Double m22 = im_a + ra.x * ra.x * ii_a + im_b + rb.x * rb.x * ii_b;

			m11 += primitive.gamma;
			m22 += primitive.gamma;

			Mat22 K{ m11, m12, m21, m22 };
			primitive.effectiveMass = K.invert();

			primitive.bodyA->applyImpulse(primitive.accumulatedImpulse, ra);
			primitive.bodyB->applyImpulse(primitive.accumulatedImpulse * -1, rb);

		}


		void solveVelocity(const Double& dt) override
		{
			if (primitive.bodyA == nullptr || primitive.bodyB == nullptr) return;
			auto& bodyA = primitive.bodyA;
			auto& bodyB = primitive.bodyB;

			Vec2 pa = bodyA->toWorldPoint(primitive.localPointA);
			Vec2 ra = pa - bodyA->getCentroidWorldPos();

			Vec2 pb = bodyB->toWorldPoint(primitive.localPointB);
			Vec2 rb = pb - bodyB->getCentroidWorldPos();

			Vec2 va = bodyA->getVel() + ra.defaultNormal() * bodyA->getAnguleVel();
			Vec2 vb = bodyB->getVel() + rb.defaultNormal() * bodyB->getAnguleVel();

			Vec2 jvb = va - vb;
			jvb += primitive.bias;
			jvb += primitive.accumulatedImpulse * primitive.gamma;
			jvb = jvb * -1;
			Vec2 J = primitive.effectiveMass * jvb;
			Vec2 oldImpulse = primitive.accumulatedImpulse;
			primitive.accumulatedImpulse += J;

			Double maxImpulse = dt * primitive.maxForce;

			if (primitive.accumulatedImpulse.lengthSquared() > maxImpulse * maxImpulse)
			{
				primitive.accumulatedImpulse.normalize();
				primitive.accumulatedImpulse = primitive.accumulatedImpulse * maxImpulse;
			}

			J = primitive.accumulatedImpulse - oldImpulse;

			primitive.bodyA->applyImpulse(J, ra);
			primitive.bodyB->applyImpulse(J * -1, rb);
		}


		void solvePosition(const Double& dt) override
		{

		}

	protected:
		RevoluteConstraintPrimitive primitive;

	};
}