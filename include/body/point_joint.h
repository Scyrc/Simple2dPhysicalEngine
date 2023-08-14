#pragma once
#include "joint.h"
#include "body.h"
#include <gl/glut.h>
namespace physicalEngine
{
	struct PointJointPrimitive
	{
		Body* bodyA;
		Vec2 localPointA;
		Vec2 targetPoint;
		Vec2 normal;

		Double damping = 0.0;
		Double stiffness = 0.0;
		Double frequency = 10;
		Double maxForce = 1000;
		Double dampingRatio = 1;
		Double gamma = 0.0;
		Vec2 bias;
		Mat22 effectiveMass;
		Vec2 accumulatedImpulse;

	};
	class PointJoint : public Joint
	{
	public:
		PointJoint()
		{
			jointType = JointType::Point;
		}
		PointJoint(const PointJointPrimitive& prim) : primitive(prim)
		{
			jointType = JointType::Point;
		}
		void set(const PointJointPrimitive& prim)
		{
			primitive = prim;
		}
		void prepare(const Double& dt) override
		{
			if (primitive.bodyA == nullptr)
				return;
			Body* bodyA = primitive.bodyA;

			Double m_a = bodyA->getMass();
			Double im_a = bodyA->getMassInv();
			Double ii_a = bodyA->GetInvInertia();
			if (primitive.frequency > 0.0)
			{
				Double nf = naturalFrequency(primitive.frequency);
				primitive.stiffness = springStiffness(m_a, nf);
				primitive.damping = springDampingCoefficient(m_a, nf, primitive.dampingRatio);
			}
			else
			{
				primitive.stiffness = 0.0;
				primitive.damping = 0.0;
			}
			primitive.gamma = constraintImpulseMixing(dt, primitive.stiffness, primitive.damping);
			Double erp = errorReductionParameter(dt, primitive.stiffness, primitive.damping);


			Vec2 pa = bodyA->toWorldPoint(primitive.localPointA);
			Vec2 ra = pa - bodyA->getPos();
			Vec2 pb = primitive.targetPoint;

			primitive.bias = (pa - pb) * erp;
			Mat22 k;
			Double m11 = im_a + ra.y * ra.y * ii_a;
			Double m12 = -ra.x * ra.y * ii_a;
			Double m21 = m12;
			Double m22 = im_a + ra.x * ra.x * ii_a;

			m11 += primitive.gamma;
			m22 += primitive.gamma;

			primitive.effectiveMass = k.invert();
			//warm start
			//m_primitive.impulse *= dt / dt;
			bodyA->applyImpulse(primitive.accumulatedImpulse, ra);
		}
		void solveVelocity(const Double& dt) override
		{
			if (primitive.bodyA == nullptr)
				return;
			Vec2 ra = primitive.bodyA->toWorldPoint(primitive.localPointA) - primitive.bodyA->getCentroidWorldPos();
			Vec2 va = primitive.bodyA->getVel() + ra.defaultNormal() * primitive.bodyA->getAnguleVel();
			Vec2 jvb = va;
			jvb += primitive.bias;
			jvb += primitive.accumulatedImpulse * primitive.gamma;
			jvb = jvb * -1;
			Vec2 J = primitive.effectiveMass.multiply(jvb);
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
		}
		void solvePosition(const Double& dt) override
		{
			
		}
		PointJointPrimitive getprimitive() const
		{
			return primitive;
		}

		void Draw() override
		{
			//throw std::logic_error("The method or operation is not implemented.");
		}

	private:
		PointJointPrimitive primitive;
		Double m_factor = 0.22f;
	};
}