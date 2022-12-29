#include "./Force.h"
#include <algorithm>

Vec2 Force::GenerateDragForce(const Particle& particle, float k) {
	if (particle.velocity.MagnitudeSquared() > 0) {
		Vec2 dragDirection = particle.velocity.UnitVector() * -1.0f;
		float dragMagnitude = k * particle.velocity.MagnitudeSquared();
		return dragDirection * dragMagnitude;
	}
	else {
		return Vec2(0.0f, 0.0f);
	}
}

Vec2 Force::GenerateFrictionForce(const Particle& particle, float k) {
	Vec2 frictionDirection = particle.velocity.UnitVector() * -1.0f;
	return frictionDirection * k;
}

Vec2 Force::GenerateGravitationalForce(const Particle& particleA, const Particle& particleB, float grav, float minDistance, float maxDistance) {
	Vec2 distance = (particleB.position - particleA.position);
	float distanceSquared = distance.MagnitudeSquared();
	distanceSquared = std::clamp(distanceSquared, minDistance, maxDistance);
	Vec2 attractionDirection = distance.UnitVector();
	float attractionMagnitude = grav * (particleA.mass * particleB.mass) / distanceSquared;
	return attractionDirection * attractionMagnitude;
}

Vec2 Force::GenerateSpringForce(const Particle& particle, Vec2 anchor, float restLength, float k) {
	Vec2 distance = particle.position - anchor;
	float displacement = distance.Magnitude() - restLength;
	Vec2 springDirection = distance.UnitVector();
	float springMagnitude = -k * displacement;
	return springDirection * springMagnitude;
}

Vec2 Force::GenerateSpringForce(const Particle& particleA, const Particle& particleB, float restLength, float k) {
	Vec2 distance = particleA.position - particleB.position;
	float displacement = distance.Magnitude() - restLength;
	Vec2 springDirection = distance.UnitVector();
	float springMagnitude = -k * displacement;
	return springDirection * springMagnitude;
}