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