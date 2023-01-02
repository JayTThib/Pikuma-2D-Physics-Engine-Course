#ifndef COLLISIONDETECTION_H
#define COLLISIONDETECTION_H

#include "Body.h"
#include "Contact.h"

struct CollisionDetection {
	static bool IsColliding(Body* body1, Body* body2, Contact& contact);
	static bool IsCollidingCircleCircle(Body* body1, Body* body2, Contact& contact);
	static bool IsCollidingCirclePolygon(Body* body1, Body* body2, Contact& contact);
	static bool IsCollidingPolygonPolygon(Body* body1, Body* body2, Contact& contact);//Uses the Separating Axis Theorem (doesn't work with concave polygons)
};

#endif