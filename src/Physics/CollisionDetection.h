#ifndef COLLISIONDETECTION_H
#define COLLISIONDETECTION_H

#include "Body.h"
#include "Contact.h"

struct CollisionDetection {
	static bool IsColliding(Body* body1, Body* body2, std::vector<Contact>& contacts);
	static bool IsCollidingCircleCircle(Body* body1, Body* body2, std::vector<Contact>& contacts);
	static bool IsCollidingCirclePolygon(Body* circBody, Body* polyBody, std::vector<Contact>& contacts);
	static bool IsCollidingPolygonPolygon(Body* body1, Body* body2, std::vector<Contact>& contacts);//Uses the Separating Axis Theorem (doesn't work with concave polygons)
};

#endif