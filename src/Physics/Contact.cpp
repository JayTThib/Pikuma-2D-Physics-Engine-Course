#include "Contact.h"

void Contact::ResolvePenetration() {
	if (bodyA->IsStatic() && bodyB->IsStatic()) {
		return;
	}

	float da = depth / (bodyA->inverseMass + bodyB->inverseMass) * bodyA->inverseMass;
	float db = depth / (bodyA->inverseMass + bodyB->inverseMass) * bodyB->inverseMass;

	bodyA->position -= normal * da;
	bodyB->position += normal * db;
}

void Contact::ResolveCollision() {
	ResolvePenetration();//Apply positional correction using the projection method
	
	float elasticity = std::min(bodyA->restitution, bodyB->restitution);
	const Vec2 relativeVelocity = (bodyA->velocity - bodyB->velocity);
	float relativeVelocityDotNormal = relativeVelocity.Dot(normal);
	const Vec2 impulseDirection = normal;
	const float impulseMagnitude = -(1 + elasticity) * relativeVelocityDotNormal / (bodyA->inverseMass + bodyB->inverseMass);
	Vec2 impulse = impulseDirection * impulseMagnitude;

	bodyA->ApplyImpulse(impulse);
	bodyB->ApplyImpulse(-impulse);
}