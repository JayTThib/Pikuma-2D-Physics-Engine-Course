#ifndef CONTACT_H
#define CONTACT_H

#include "Vec2.h"
#include "Body.h"

struct Contact {
	Body* bodyA;
	Body* bodyB;

	Vec2 start;
	Vec2 end;

	Vec2 normal;
	float depth;

	Contact() = default;
	~Contact() = default;
	
	void ResolvePenetration();
	void ResolveCollision();
};

#endif