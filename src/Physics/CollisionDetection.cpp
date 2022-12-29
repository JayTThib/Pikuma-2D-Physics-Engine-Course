#include "CollisionDetection.h"

bool CollisionDetection::IsColliding(Body* bodyA, Body* bodyB) {
	bool aIsCircle = bodyA->shape->GetType() == CIRCLE;
	bool bIsCircle = bodyB->shape->GetType() == CIRCLE;

	if (aIsCircle && bIsCircle) {
		return IsCollidingCircleCircle(bodyA, bodyB);
	}
}

bool CollisionDetection::IsCollidingCircleCircle(Body* bodyA, Body* bodyB) {
	CircleShape* circleA = (CircleShape*)bodyA->shape;
	CircleShape* circleB = (CircleShape*)bodyB->shape;

	const Vec2 ab = bodyA->position - bodyB->position;
	const float radiusSum = circleA->radius + circleB->radius;
	return ab.MagnitudeSquared() <= (radiusSum * radiusSum);
}