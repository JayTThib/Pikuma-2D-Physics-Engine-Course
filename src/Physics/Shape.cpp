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

float PolygonShape::PolygonArea() const {
	float area = 0.0;
	for (int i = 0; i < localVertices.size(); i++) {
		int j = (i + 1) % localVertices.size();
		area += localVertices[i].Cross(localVertices[j]);
	}
	return area / 2.0;
}

Vec2 PolygonShape::PolygonCentroid() const {
	Vec2 cg { 0,0 };

	for (int i = 0; i < localVertices.size(); i++) {
		int j = (i + 1) % localVertices.size();
		cg += (localVertices[i] + localVertices[j]) * localVertices[i].Cross(localVertices[j]);
	}
	return cg / 6 / PolygonArea();
}

float PolygonShape::GetMomentOfInertia() const {
	float acc0 = 0;
	float acc1 = 0;

	for (int i = 0; i < localVertices.size(); i++) {
		auto a = localVertices[i];
		auto b = localVertices[(i + 1) % localVertices.size()];
		auto cross = abs(a.Cross(b));

		acc0 += cross * (a.Dot(a) + b.Dot(b) + a.Dot(b));
		acc1 += cross;
	}

	return acc0 / 6 / acc1;
}

Vec2 PolygonShape::EdgeAt(int index) const {
	int nextVertex = (index + 1) % worldVertices.size();//Avoids going out of bounds
	return worldVertices[nextVertex] - worldVertices[index];
}

float PolygonShape::FindMinSeparation(const PolygonShape* other, int& indexReferenceEdge, Vec2& supportPoint) const {
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
			indexReferenceEdge = poly1Index;
			supportPoint = minVertex;
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

int PolygonShape::FindIncidentEdge(const Vec2& referenceEdgeNormal) const {
	int indexIncidentEdge = 0;
	float minProjection = std::numeric_limits<float>::max();

	for (int i = 0; i < this->worldVertices.size(); i++) {
		Vec2 edgeNormal = this->EdgeAt(i).Normal();
		float proj = edgeNormal.Dot(referenceEdgeNormal);

		if (proj < minProjection) {
			minProjection = proj;
			indexIncidentEdge = i;
		}
	}

	return indexIncidentEdge;
}

int PolygonShape::ClipSegmentToLine(const std::vector<Vec2>& contactsIn, std::vector<Vec2>& contactsOut, const Vec2& clip1, const Vec2& clip2) const {
	int numOut = 0;
	
	//Calc the distance of end points to the line
	Vec2 normal = (clip2 - clip1).Normalize();
	float dist1 = (contactsIn[0] - clip1).Cross(normal);
	float dist2 = (contactsIn[1] - clip1).Cross(normal);
	
	//If the points are behind the plane
	if (dist1 <= 0) {
		contactsOut[numOut++] = contactsIn[0];
	}
	if (dist2 <= 0) {
		contactsOut[numOut++] = contactsIn[1];
	}

	//If the points are on different sides of the plane (one distance is negative and the other is positive)
	if (dist1 * dist2 < 0) {
		float totalDist = dist1 - dist2;

		//Find the intersection using linear interpolation
		float percent = dist1 / (totalDist);
		Vec2 contact = contactsIn[0] + (contactsIn[1] - contactsIn[0]) * percent;
		contactsOut[numOut] = contact;
		numOut++;
	}

	return numOut;
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