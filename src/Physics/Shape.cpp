#include "Shape.h"

CircleShape::CircleShape(float radius) {
	this->radius = radius;
}

CircleShape::~CircleShape() {
	
}

ShapeType CircleShape::GetType() const {
	return CIRCLE;
}

Shape* CircleShape::Clone() const {
	return new CircleShape(radius);
}

float CircleShape::GetMomentOfInertia() const {
	//For solid circles, the moment of inertia is 1/2 * r^2
	//But this still needs to be multiplied by the rigidbody's mass
	return 0.5f * (radius * radius);
}

PolygonShape::PolygonShape(const std::vector<Vec2> vertices) {
	
}

PolygonShape::~PolygonShape() {

}

ShapeType PolygonShape::GetType() const {
	return POLYGON;
}

Shape* PolygonShape::Clone() const {
	return new PolygonShape(localVertices);
}

float PolygonShape::GetMomentOfInertia() const {
	//TODO
	return 0.0f;
}

void PolygonShape::UpdateVertices(float angle, const Vec2& position) {
	for (int i = 0; i < localVertices.size(); i++) {
		worldVertices[i] = localVertices[i].Rotate(angle);
		worldVertices[i] += position;
	}
}

BoxShape::BoxShape(float width, float height) {
	this->width = width;
	this->height = height;

	localVertices.push_back(Vec2(-width / 2.0f, -height / 2.0f));//Top left
	localVertices.push_back(Vec2(+width / 2.0f, -height / 2.0f));//Top right
	localVertices.push_back(Vec2(+width / 2.0f, +height / 2.0f));//Bottom right
	localVertices.push_back(Vec2(-width / 2.0f, +height / 2.0f));//Bottom left

	worldVertices.push_back(Vec2(-width / 2.0f, -height / 2.0f));//Top left
	worldVertices.push_back(Vec2(+width / 2.0f, -height / 2.0f));//Top right
	worldVertices.push_back(Vec2(+width / 2.0f, +height / 2.0f));//Bottom right
	worldVertices.push_back(Vec2(-width / 2.0f, +height / 2.0f));//Bottom left
}

BoxShape::~BoxShape() {

}

ShapeType BoxShape::GetType() const {
	return BOX;
}

Shape* BoxShape::Clone() const {
	return new BoxShape(width, height);
}

float BoxShape::GetMomentOfInertia() const {
	//For a rectangle, the moment of inertia is 1/12 * (w^2 + h^2)
	//But this still needs to be multiplied by the rigidbody's mass
	return (0.83333f) * ((width * width) + (height * height));
}