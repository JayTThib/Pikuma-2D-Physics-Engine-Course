#include "./Force.h"

Vec2 Force::GenerateDragForce(const Particle& particle, float k) {
	Vec2 dragForce = Vec2(0.0f, 0.0f);

	if (particle.velocity.MagnitudeSquared() > 0) {
		Vec2 dragDirection = particle.velocity.UnitVector() * -1.0f;
		float dragMagnitude = k * particle.velocity.MagnitudeSquared();
		dragForce = dragDirection * dragMagnitude;
	}

	return dragForce;
}