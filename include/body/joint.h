#pragma once

#include "../common/ctypes.h"
namespace physicalEngine
{
	enum class JointType
	{
		Distance = 0,
		Revolute = 1,
		Point = 2,
	};
	class Joint {
	public:
		Joint() = default;
		virtual void Draw() = 0;

		virtual void prepare(const Double& dt) = 0;
		virtual void solveVelocity(const  Double& dt) = 0;
		virtual void solvePosition(const Double& dt) = 0;

		static Double naturalFrequency(Double frequency)
		{
			return 2.0f * 3.1415926f * frequency;
		}

		static Double springDampingCoefficient(Double mass, Double naturalFrequency, Double dampingRatio)
		{
			return dampingRatio * 2.0f * mass * naturalFrequency;
		}

		static Double springStiffness(Double mass, Double naturalFrequency)
		{
			return mass * naturalFrequency * naturalFrequency;
		}

		static Double constraintImpulseMixing(Double dt, Double stiffness, Double damping)
		{
			Double cim = dt * (dt * stiffness + damping);
			return twoDMath::realEqual(cim, 0.0f) ? 0.0f : 1.0f / cim;
		}

		static Double errorReductionParameter(Double dt, Double stiffness, Double damping)
		{
			Double erp = dt * stiffness + damping;
			return twoDMath::realEqual(erp, 0.0f) ? 0.0f : stiffness / erp;
		}
	protected:
		uint32_t id;
		JointType jointType;
	};
}