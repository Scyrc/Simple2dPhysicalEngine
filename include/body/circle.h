#pragma once

#include "body.h"

namespace physicalEngine
{
	class Circle : public Body
	{

	public:
		Circle(Double density, const Vec2& position, const Double radius);
		virtual void Draw() override;

		virtual BodyType getType() const override;

		virtual Vec2 edge(size_t idx) const override;

		virtual size_t getEdgeNums() const override;

		virtual Vec2 getCentroidWorldPos() const override;

		virtual void stepPos(Double dt) override;

		//virtual void stepVel(Vec2 Gravity, Double dt) override;*/
	
		virtual void move(Vec2 mtv) override;
		
		virtual void moveToPos(const Vec2& pos) override;
		

		virtual void RotateTo(Double angle) override;
		
		virtual bool IsContianPoint(const Vec2& point) override;

		Double getRadius() const { return radius; }
	private:
		Double radius;
		Double radiusShow;

		Vec2 topPoint;
		Vec2 topPointForDraw;
		Vec2 positionShow;

		const  Double pi = 3.141592653f;
	};
}