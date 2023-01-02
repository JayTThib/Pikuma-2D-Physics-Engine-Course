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
	float fric = std::min(body1->friction, body2->friction);

	//The distance between the center of mass and the contact point.
	Vec2 body1Dist = end - body1->position;
	Vec2 body2Dist = start - body2->position;

	Vec2 body1Vel = body1->velocity + Vec2(-body1->angularVelocity * body1Dist.y, body1->angularVelocity * body1Dist.x);
	Vec2 body2Vel = body2->velocity + Vec2(-body2->angularVelocity * body2Dist.y, body2->angularVelocity * body2Dist.x);
	const Vec2 relativeVelocity = body1Vel - body2Vel;

	//Calc collision impulse along the normal
	float relativeVelocityDotNormal = relativeVelocity.Dot(normal);
	const Vec2 impulseDirectionNormal = normal;
	const float impulseMagnitudeNormal = -(1 + elasticity) * relativeVelocityDotNormal / ((body1->inverseMass + body2->inverseMass) + body1Dist.Cross(normal) * body1Dist.Cross(normal) * body1->inverseRotationalInertia + body2Dist.Cross(normal) * body2Dist.Cross(normal) * body2->inverseRotationalInertia);
	Vec2 impulseNormal = impulseDirectionNormal * impulseMagnitudeNormal;
	
	//Calc collision impulse along the tangent
	Vec2 tangent = normal.Normal();
	float relativeVelocityDotTangent = relativeVelocity.Dot(tangent);
	const Vec2 impulseDirectionTangent = tangent;
	const float impulseMagnitudeTangent = fric * -(1 + elasticity) * relativeVelocityDotTangent / ((body1->inverseMass + body2->inverseMass) + body1Dist.Cross(tangent) * body1Dist.Cross(tangent) * body1->inverseRotationalInertia + body2Dist.Cross(tangent) * body2Dist.Cross(tangent) * body2->inverseRotationalInertia);
	Vec2 impulseTangent = impulseDirectionTangent * impulseMagnitudeTangent;

	Vec2 finalImpulse = impulseNormal + impulseTangent;
	body1->ApplyImpulse(finalImpulse, body1Dist);
	body2->ApplyImpulse(-finalImpulse, body2Dist);
}