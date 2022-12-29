#include "CollisionDetection.h"

bool CollisionDetection::IsColliding(Body* bodyA, Body* bodyB, Contact& contact) {
	bool aIsCircle = bodyA->shape->GetType() == CIRCLE;
	bool bIsCircle = bodyB->shape->GetType() == CIRCLE;

	if (aIsCircle && bIsCircle) {
		return IsCollidingCircleCircle(bodyA, bodyB, contact);
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