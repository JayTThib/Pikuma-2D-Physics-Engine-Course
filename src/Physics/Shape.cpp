#include "Shape.h"

CircleShape::CircleShape(float radius) {
	this->radius = radius;
}

CircleShape::~CircleShape() {
	
}

ShapeType CircleShape::GetType() const {
	return CIRCLE;
}

PolygonShape::PolygonShape(const std::vector<Vec2> vertices) {
	
}

PolygonShape::~PolygonShape() {

}

ShapeType PolygonShape::GetType() const {
	return POLYGON;
}

BoxShape::BoxShape(float width, float height) {

}

BoxShape::~BoxShape() {

}

ShapeType BoxShape::GetType() const {
	return BOX;
}