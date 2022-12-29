#ifndef COLLISIONDETECTION_H
#define COLLISIONDETECTION_H

#include "Body.h"

struct CollisionDetection {
	static bool IsColliding(Body* bodyA, Body* bodyB);
	static bool IsCollidingCircleCircle(Body* bodyA, Body* bodyB);
	//static bool IsCollidingPolygonPolygon(Body* bodyA, Body* bodyB);
	//static bool IsCollidingPolygonCircle(Body* bodyA, Body* bodyB);
};

#endif