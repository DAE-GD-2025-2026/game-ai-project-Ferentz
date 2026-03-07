#pragma once
#include "Movement/SteeringBehaviors/Steering/SteeringBehaviors.h"
class Flock;

//COHESION - FLOCKING
//*******************
class Cohesion final : public Seek
{
public:
	Cohesion(Flock* const pFlock)
		:Seek{},
		pFlock(pFlock)
	{};
	virtual ~Cohesion() = default;

	//Cohesion Behavior
	SteeringOutput CalculateSteering(float deltaT, ASteeringAgent& Agent) override;

private:
	Flock* pFlock = nullptr;
};

//SEPARATION - FLOCKING
//*********************
class Separation final : public Flee
{
public:
	Separation(Flock* const pFlock)
		:Flee{},
		pFlock(pFlock) //,
		//evade{}
	{};
	virtual ~Separation() = default;
	SteeringOutput CalculateSteering(float deltaT, ASteeringAgent& Agent) override;

private:
	Flock* pFlock = nullptr;
};

//VELOCITY MATCH - FLOCKING
//************************

class VelocityMatch final : public Seek
{
public:
	VelocityMatch(Flock* const pFlock)
		:Seek{},
		pFlock(pFlock)
	{
	};
	virtual ~VelocityMatch() = default;
	SteeringOutput CalculateSteering(float deltaT, ASteeringAgent& Agent) override;

private:
	Flock* pFlock = nullptr;
};