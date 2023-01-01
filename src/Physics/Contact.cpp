#include "Contact.h"

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
	ResolvePenetration();//Apply positional correction using the projection method
	
	float elasticity = std::min(body1->elasticity, body2->elasticity);
	const Vec2 relativeVelocity = (body1->velocity - body2->velocity);
	float relativeVelocityDotNormal = relativeVelocity.Dot(normal);
	const Vec2 impulseDirection = normal;
	const float impulseMagnitude = -(1 + elasticity) * relativeVelocityDotNormal / (body1->inverseMass + body2->inverseMass);
	Vec2 impulse = impulseDirection * impulseMagnitude;

	body1->ApplyImpulse(impulse);
	body2->ApplyImpulse(-impulse);
}