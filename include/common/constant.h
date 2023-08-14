#pragma once

namespace physicalEngine
{
	using Double = double;

	namespace Constant 
	{
		constexpr Double Pi = 3.141526539f;
		constexpr Double HalfPi = Pi / 2.0f;

		constexpr Double sceneTop = 90.0f;
		constexpr Double sceneL = 0.f;
		constexpr Double sceneR = 100.f;


		constexpr Double MaxValue = DBL_MAX;
		constexpr Double MinValue = -DBL_MAX;

	};
}