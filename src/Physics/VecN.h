#ifndef VECN_H
#define VECN_H

struct VecN {
	int componentNum;
	float* data;

	VecN();
	VecN(int componentNum);
	VecN(const VecN& vecN);
	~VecN();

	void Zero();//Resets data
	float Dot(const VecN& vecN) const;//vecN1.Dot(vecN2)

	VecN& operator = (const VecN& vecN);				//vecN1 = vecN2
	VecN operator + (const VecN& vecN) const;			//vecN1 + vecN2
	VecN operator - (const VecN& vecN) const;			//vecN1 - vecN2
	VecN operator * (const float multiplier) const;		//vecN1 * multiplier
	const VecN& operator += (const VecN& vecN);			//vecN1 += vecN2
	const VecN& operator -= (const VecN& vecN);			//vecN1 -= vecN2
	const VecN& operator *= (const float multiplier);	//vecN1 *= multiplier
	float operator [] (const int index) const;			//vecN1[index]
	float& operator [] (const int index);				//vecN1[index]
};

#endif