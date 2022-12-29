#ifndef COLLISIONDETECTION_H
#define COLLISIONDETECTION_H

#include "Body.h"
#include "Contact.h"

struct CollisionDetection {
	static bool IsColliding(Body* bodyA, Body* bodyB, Contact& contact);
	static bool IsCollidingCircleCircle(Body* bodyA, Body* bodyB, Contact& contact);
	//static bool IsCollidingPolygonPolygon(Body* bodyA, Body* bodyB);
	//static bool IsCollidingPolygonCircle(Body* bodyA, Body* bodyB);
};

#endif