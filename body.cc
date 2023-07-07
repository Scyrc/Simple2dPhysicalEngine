#include "body.h"
#include <gl/glut.h>
namespace physicalEngine
	
{

	Polygon::Polygon(Double mass_, const VerticesListType& VerticesList_) : Body(mass_), VerticesList(VerticesList_)
	{
		
	}

	VerticesListType* Polygon::GetVerticesWorldPosition()
	{
		VerticesListType resList(VerticesList.size());

		for (int i = 0; i < VerticesList.size(); ++i)
		{
			resList[i] = position + this->operator[](i);
		}

		return &resList;
	}

	void Polygon::Draw()
	{
		glBegin(GL_LINE_LOOP);
		glBegin(GL_LINE_STRIP);
		glColor3f(1.0f, 0.0f, 0.0f); // Red

		//VerticesListType* list = GetVerticesWorldPosition();
		VerticesListType resList(VerticesList.size());

		for (int i = 0; i < VerticesList.size(); ++i)
		{
			resList[i] = position + this->operator[](i);
		}

		for (auto& v : resList)
		{
			glVertex2d(v.x, v.y);
		}

		glEnd();
	}

}
	