#include "./Force.h"

Vec2 Force::GenerateDragForce(const Particle& particle, float magnitude) {
	if (particle.velocity.MagnitudeSquared() > 0) {
		Vec2 dragDirection = particle.velocity.UnitVector() * -1.0f;
		float dragMagnitude = magnitude * particle.velocity.MagnitudeSquared();
		return dragDirection * dragMagnitude;
	}
	else {
		return Vec2(0.0f, 0.0f);
	}
}

Vec2 Force::GenerateFrictionForce(const Particle& particle, float magnitude) {
	Vec2 frictionDirection = particle.velocity.UnitVector() * -1.0f;
	return frictionDirection * magnitude;
}