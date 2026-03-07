#include "FlockingSteeringBehaviors.h"
#include "Flock.h"
#include "../SteeringAgent.h"
#include "../SteeringHelpers.h"


//*******************
//COHESION (FLOCKING)
SteeringOutput Cohesion::CalculateSteering(float deltaT, ASteeringAgent& Agent)
{
	if (pFlock->GetNrOfNeighbors() == 0)
	{
		SteeringOutput Steering{};
		Steering.LinearVelocity = FVector2D::ZeroVector;
		Steering.IsValid = false;
		return Steering;
	}

	FTargetData target;
	target.Position = pFlock->GetAverageNeighborPos();
	//target.LinearVelocity = pFlock->GetAverageNeighborVelocity();

	SetTarget(target);
	//Target.LinearVelocity = pFlock->GetAverageNeighborPos() + pFlock->GetAverageNeighborVelocity();
	
	return  Seek::CalculateSteering(deltaT, Agent);
}

//*********************
//SEPARATION (FLOCKING)

SteeringOutput Separation::CalculateSteering(float deltaT, ASteeringAgent& Agent)
{
	SteeringOutput separateSteering = {};

	if (pFlock->GetNrOfNeighbors() == 0)
	{
		separateSteering.LinearVelocity = FVector2D::ZeroVector;
		separateSteering.IsValid = false;
		return separateSteering;
	}

	
	for (auto& bird : pFlock->GetNeighbors())
	{
		float dist{ static_cast<float>(FVector2D::Distance(Agent.GetPosition(), Target.Position)) };
		float ratio{ dist / pFlock->GetNeighborhoodRadius() };

		FTargetData target;
		target.Position = bird->GetPosition();
		target.Orientation = bird->GetRotation();
		target.LinearVelocity = bird->GetLinearVelocity();
		target.AngularVelocity = bird->GetAngularVelocity();

		SetTarget(target);

		SteeringOutput tempSteering = Flee::CalculateSteering(deltaT, Agent);
		tempSteering *= ratio * Agent.GetMaxLinearSpeed();
		separateSteering += tempSteering;
	}

	return separateSteering;
}


//*************************
//VELOCITY MATCH (FLOCKING)

SteeringOutput VelocityMatch::CalculateSteering(float deltaT, ASteeringAgent& Agent)
{
	if (pFlock->GetNrOfNeighbors() == 0)
	{
		SteeringOutput Steering{};
		Steering.LinearVelocity = FVector2D::ZeroVector;
		Steering.IsValid = false;
		return Steering;
	}

	SteeringOutput separateSteering = {};
	separateSteering.LinearVelocity = pFlock->GetAverageNeighborVelocity();
	return separateSteering;
}
