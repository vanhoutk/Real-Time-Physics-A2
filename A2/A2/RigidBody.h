#pragma once

#include <GL/glew.h>
#include "Antons_maths_funcs.h"

class RigidBody {
public:
	// Constants
	GLfloat mass;
	mat4 Ibody;
	mat4 IbodyInv;

	// State Variables
	vec4 position;			// x(t)
	versor orientation;		// q(t)
	vec4 linearMomentum;	// P(t)
	vec4 angularMomentum;	// L(t)

	// Derived Quantities
	mat4 Iinv;				// I-1(t) = R(t) * IbodyInv * R(t)T
	mat4 rotation;			// Rotation Matrix R(t)
	vec4 velocity;			// v(t)  = P(t) / mass
	vec4 angularVelocity;	// w(t)  = I-1(t) * L(t)

	// Computed Quantities
	vec4 torque;			// T(t)
	vec4 force;				// F(t)

	RigidBody();
};

RigidBody::RigidBody()
{
	this->mass = 1.0f;
	
	// Calculate Ibody and IbodyInv
	// this->Ibody = 
	// this->IbodyInv = 

	this->orientation.q[0] = 0.0f;
	this->orientation.q[1] = 0.0f;
	this->orientation.q[2] = 1.0f;
	this->orientation.q[3] = 0.0f;

	this->position = vec4(0.0f, 0.0f, 0.0f, 0.0f);
	this->rotation = identity_mat4();
	this->linearMomentum = vec4(0.0f, 0.0f, 0.0f, 0.0f);
	this->angularMomentum = vec4(0.0f, 0.0f, 0.0f, 0.0f);

	this->torque = vec4(0.0f, 0.0f, 0.0f, 0.0f);
	this->force = vec4(0.0f, 0.0f, 0.0f, 0.0f);

	this->velocity = vec4(0.0f, 0.0f, 0.0f, 0.0f);
	this->angularVelocity = vec4(0.0f, 0.0f, 0.0f, 0.0f);
	this->Iinv = identity_mat4();
}

/*mat3 star(vec4 a)
{
	mat3 result = zero_mat3();
	result.m[1] =  a.v[2];
	result.m[2] = -a.v[1];
	result.m[3] = -a.v[2];
	result.m[5] =  a.v[0];
	result.m[6] =  a.v[1];
	result.m[7] = -a.v[0];
	return result;
}*/

void multiplyQuat(versor &result, versor r, versor s)
{
	result.q[0] = s.q[0] * r.q[0] - s.q[1] * r.q[1] -
		s.q[2] * r.q[2] - s.q[3] * r.q[3];
	result.q[1] = s.q[0] * r.q[1] + s.q[1] * r.q[0] -
		s.q[2] * r.q[3] + s.q[3] * r.q[2];
	result.q[2] = s.q[0] * r.q[2] + s.q[1] * r.q[3] +
		s.q[2] * r.q[0] - s.q[3] * r.q[1];
	result.q[3] = s.q[0] * r.q[3] - s.q[1] * r.q[2] +
		s.q[2] * r.q[1] + s.q[3] * r.q[0];
	normalise(result); // Re-normalise
}

float quatMagnitude(versor v)
{
	float sum = v.q[0] * v.q[0] + v.q[1] * v.q[1] + v.q[2] * v.q[2] + v.q[3] * v.q[3];
	float result = sqrt(sum);
	return result;
}