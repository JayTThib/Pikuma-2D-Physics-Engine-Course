#include "VecN.h"

VecN::VecN(): componentNum(0), data(nullptr) {}

VecN::VecN(int componentNum) : componentNum(componentNum) {
	data = new float[componentNum];
}

VecN::VecN(const VecN& vecN) {
	componentNum = vecN.componentNum;
	data = new float[componentNum];
	for (int i = 0; i < componentNum; i++) {
		data[i] = vecN.data[i];
	}
}

VecN::~VecN() {
	delete[] data;
}

void VecN::Zero() {
	for (int i = 0; i < componentNum; i++) {
		data[i] = 0.0f;
	}
}

float VecN::Dot(const VecN& vecN) const {
	float sum = 0.0f;
	for (int i = 0; i < componentNum; i++) {
		sum += data[i] * vecN.data[i];
	}
	return sum;
}

VecN& VecN::operator = (const VecN& vecN) {
	delete[] data;
	componentNum = vecN.componentNum;
	data = new float[componentNum];

	for (int i = 0; i < componentNum; i++) {
		data[i] = vecN.data[i];
	}
	return *this;
}

VecN VecN::operator * (float multiplier) const {
	VecN result = *this;
	result *= multiplier;
	return result;
}

VecN VecN::operator + (const VecN& vecN) const {
	VecN result = *this;
	for (int i = 0; i < componentNum; i++) {
		result.data[i] += vecN.data[i];
	}
	return result;
}

VecN VecN::operator - (const VecN& vecN) const {
	VecN result = *this;
	for (int i = 0; i < componentNum; i++) {
		result.data[i] -= vecN.data[i];
	}
	return result;
}

const VecN& VecN::operator *= (float multiplier) {
	for (int i = 0; i < componentNum; i++) {
		data[i] *= multiplier;
	}
	return *this;
}

const VecN& VecN::operator += (const VecN& vecN) {
	for (int i = 0; i < componentNum; i++) {
		data[i] += vecN.data[i];
	}
	return *this;
}

const VecN& VecN::operator -= (const VecN& vecN) {
	for (int i = 0; i < componentNum; i++) {
		data[i] -= vecN.data[i];
	}
	return *this;
}

float VecN::operator [] (const int index) const {
	return data[index];
}

float& VecN::operator [] (const int index) {
	return data[index];
}