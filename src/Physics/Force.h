#ifndef FORCE_H
#define FORCE_H

#include "./Vec2.h"
#include "./Particle.h"

struct Force {
	static Vec2 GenerateDragForce(const Particle& particle, float k);
	static Vec2 GenerateFrictionForce(const Particle& particle, float k);
	static Vec2 GenerateSpringForce(const Particle& particle, Vec2 anchor, float restLength, float k);
	static Vec2 GenerateSpringForce(const Particle& particleA, const Particle& particleB, float restLength, float k);
	static Vec2 GenerateGravitationalForce(const Particle& particleA, const Particle& particleB, float grav, float minDistance, float maxDistance);
};

#endif
