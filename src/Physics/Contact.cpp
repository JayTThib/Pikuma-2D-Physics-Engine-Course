#include "Contact.h"

//Apply positional correction using the projection method
void Contact::ResolvePenetration() {
	if (body1->IsStatic() && body2->IsStatic()) {
		return;
	}

	float d1 = depth / (body1->inverseMass + body2->inverseMass) * body1->inverseMass;
	float d2 = depth / (body1->inverseMass + body2->inverseMass) * body2->inverseMass;

	body1->position -= normal * d1;
	body2->position += normal * d2;
}

void Contact::ResolveCollision() {
	ResolvePenetration();
	
	float elasticity = std::min(body1->elasticity, body2->elasticity);
	//The distance between the center of mass and the contact point.
	Vec2 body1Dist = end - body1->position;
	Vec2 body2Dist = start - body2->position;

	Vec2 body1Vel = body1->velocity + Vec2(-body1->angularVelocity * body1Dist.y, body1->angularVelocity * body1Dist.x);
	Vec2 body2Vel = body2->velocity + Vec2(-body2->angularVelocity * body2Dist.y, body2->angularVelocity * body2Dist.x);
	const Vec2 relativeVelocity = body1Vel - body2Vel;

	float relativeVelocityDotNormal = relativeVelocity.Dot(normal);

	const Vec2 impulseDirection = normal;
	const float impulseMagnitude = -(1 + elasticity) * relativeVelocityDotNormal / ((body1->inverseMass + body2->inverseMass) + body1Dist.Cross(normal) * body1Dist.Cross(normal) * body1->inverseRotationalInertia + body2Dist.Cross(normal) * body2Dist.Cross(normal) * body2->inverseRotationalInertia);

	Vec2 impulse = impulseDirection * impulseMagnitude;
	body1->ApplyImpulse(impulse, body1Dist);
	body2->ApplyImpulse(-impulse, body2Dist);
}