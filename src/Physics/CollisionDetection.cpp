#include "CollisionDetection.h"

bool CollisionDetection::IsColliding(Body* bodyA, Body* bodyB, Contact& contact) {
	if (bodyA->shape->GetType() == CIRCLE && bodyB->shape->GetType() == CIRCLE) {
		return IsCollidingCircleCircle(bodyA, bodyB, contact);
	}
	else if ((bodyA->shape->GetType() == POLYGON || bodyA->shape->GetType() == BOX) 
		&& (bodyB->shape->GetType() == POLYGON || bodyB->shape->GetType() == BOX)) {
		return IsCollidingPolygonPolygon(bodyA, bodyB, contact);
	}
	else{//temporary - remove later
		return false;
	}
}

bool CollisionDetection::IsCollidingCircleCircle(Body* bodyA, Body* bodyB, Contact& contact) {
	CircleShape* circleA = (CircleShape*)bodyA->shape;
	CircleShape* circleB = (CircleShape*)bodyB->shape;

	const Vec2 ab = bodyB->position - bodyA->position;
	const float radiusSum = circleA->radius + circleB->radius;

	bool isColliding = ab.MagnitudeSquared() <= (radiusSum * radiusSum);
	if (!isColliding) {
		return false;
	}
	else {
		contact.bodyA = bodyA;
		contact.bodyB = bodyB;

		contact.normal = ab;
		contact.normal.Normalize();

		contact.start = bodyB->position - contact.normal * circleB->radius;
		contact.end = bodyA->position + contact.normal * circleA->radius;
		contact.depth = (contact.end - contact.start).Magnitude();
		return true;
	}
}

bool CollisionDetection::IsCollidingPolygonPolygon(Body* bodyA, Body* bodyB, Contact& contact) {
	//Uses the Separating Axis Theorem (doesn't work with concave polygons)
	const PolygonShape* polyA = (PolygonShape*)bodyA->shape;
	const PolygonShape* polyB = (PolygonShape*)bodyB->shape;
	
	if (polyA->FindMinSeparation(polyB) >= 0) {
		return false;
	}
	else if (polyB->FindMinSeparation(polyA) >= 0) {
		return false;
	}
	else{
		return true;
	}
}