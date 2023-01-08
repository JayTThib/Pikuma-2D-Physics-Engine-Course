#include "Shape.h"
#include <limits>

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

void CircleShape::UpdateVertices(float angle, const Vec2& position) {
	return; //Do nothing since circles don't have vertices
}

float CircleShape::GetMomentOfInertia() const {
	//For solid circles, the moment of inertia is 1/2 * r^2
	//But this still needs to be multiplied by the rigidbody's mass
	return 0.5f * (radius * radius);
}

PolygonShape::PolygonShape(const std::vector<Vec2> vertices) {
	localVertices = vertices;
	worldVertices = vertices;
}

PolygonShape::~PolygonShape() {}

ShapeType PolygonShape::GetType() const {
	return POLYGON;
}

Shape* PolygonShape::Clone() const {
	return new PolygonShape(localVertices);
}

float PolygonShape::GetMomentOfInertia() const {
	//TODO
	return 5000.0f;
}

Vec2 PolygonShape::EdgeAt(int index) const {
	int nextVertex = (index + 1) % worldVertices.size();//Avoids going out of bounds
	return worldVertices[nextVertex] - worldVertices[index];
}

float PolygonShape::FindMinSeparation(const PolygonShape* other, Vec2& bestAxisOfPenetration, Vec2& vertexInOtherPolyWithMinProjection) const {
	float separation = std::numeric_limits<float>::lowest();

	for (int poly1Index = 0; poly1Index < this->worldVertices.size(); poly1Index++) {
		Vec2 vertex1 = this->worldVertices[poly1Index];
		Vec2 normal = this->EdgeAt(poly1Index).Normal();
		float minSeparation = std::numeric_limits<float>::max();
		Vec2 minVertex;

		for (int poly2Index = 0; poly2Index < other->worldVertices.size(); poly2Index++) {
			Vec2 vertex2 = other->worldVertices[poly2Index];
			float projection = (vertex2 - vertex1).Dot(normal);

			if (projection < minSeparation) {
				minSeparation = projection;
				minVertex = vertex2;
			}

			minSeparation = std::min(minSeparation, (vertex2 - vertex1).Dot(normal));//Project vertex b onto the normal axis 
		}

		if (minSeparation > separation) {
			separation = minSeparation;
			bestAxisOfPenetration = this->EdgeAt(poly1Index);
			vertexInOtherPolyWithMinProjection = minVertex;
		}
	}

	return separation;
}

void PolygonShape::UpdateVertices(float angle, const Vec2& position) {
	//Rotate and translate the polygon vertices from local space to world space.
	for (int i = 0; i < localVertices.size(); i++) {
		worldVertices[i] = localVertices[i].Rotate(angle);
		worldVertices[i] += position;
	}
}

BoxShape::BoxShape(float width, float height) {
	this->width = width;
	this->height = height;

	localVertices = 
	{
		Vec2(-width / 2.0f, -height / 2.0f),//Top left
		Vec2(+width / 2.0f, -height / 2.0f),//Top right
		Vec2(+width / 2.0f, +height / 2.0f),//Bottom right
		Vec2(-width / 2.0f, +height / 2.0f)//Bottom left
	};
	worldVertices = localVertices;
}

BoxShape::~BoxShape() {
	//todo
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