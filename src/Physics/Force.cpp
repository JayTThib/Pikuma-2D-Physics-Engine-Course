#include "./Force.h"
#include <algorithm>

Vec2 Force::GenerateDragForce(const Body& body, float k) {
	if (body.velocity.MagnitudeSquared() > 0) {
		Vec2 dragDirection = body.velocity.UnitVector() * -1.0f;
		float dragMagnitude = k * body.velocity.MagnitudeSquared();
		return dragDirection * dragMagnitude;
	}
	else {
		return Vec2(0.0f, 0.0f);
	}
}

Vec2 Force::GenerateFrictionForce(const Body& body, float k) {
	Vec2 frictionDirection = body.velocity.UnitVector() * -1.0f;
	return frictionDirection * k;
}

Vec2 Force::GenerateGravitationalForce(const Body& bodyA, const Body& bodyB, float grav, float minDistance, float maxDistance) {
	Vec2 distance = (bodyB.position - bodyA.position);
	float distanceSquared = distance.MagnitudeSquared();
	distanceSquared = std::clamp(distanceSquared, minDistance, maxDistance);
	Vec2 attractionDirection = distance.UnitVector();
	float attractionMagnitude = grav * (bodyA.mass * bodyB.mass) / distanceSquared;
	return attractionDirection * attractionMagnitude;
}

Vec2 Force::GenerateSpringForce(const Body& body, Vec2 anchor, float restLength, float k) {
	Vec2 distance = body.position - anchor;
	float displacement = distance.Magnitude() - restLength;
	Vec2 springDirection = distance.UnitVector();
	float springMagnitude = -k * displacement;
	return springDirection * springMagnitude;
}

Vec2 Force::GenerateSpringForce(const Body& bodyA, const Body& bodyB, float restLength, float k) {
	Vec2 distance = bodyA.position - bodyB.position;
	float displacement = distance.Magnitude() - restLength;
	Vec2 springDirection = distance.UnitVector();
	float springMagnitude = -k * displacement;
	return springDirection * springMagnitude;
}