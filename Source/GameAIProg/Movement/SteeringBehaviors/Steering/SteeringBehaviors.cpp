#include "SteeringBehaviors.h"
#include "GameAIProg/Movement/SteeringBehaviors/SteeringAgent.h"

//SEEK
//*******
// TODO: Do the Week01 assignment :^)

SteeringOutput Seek::CalculateSteering(float DeltaTime, ASteeringAgent& Agent)
{
	SteeringOutput Steering{};
	Steering.LinearVelocity = Target.Position - Agent.GetPosition();
	return Steering;
}

SteeringOutput Flee::CalculateSteering(float DeltaTime, ASteeringAgent& Agent)
{
	SteeringOutput Steering{};
	if (FVector2D::Distance(Agent.GetPosition(), Target.Position) < 1000.f)
	{
		Steering.LinearVelocity = Agent.GetPosition() - Target.Position;
	}
	else
	{
		Steering.LinearVelocity =  -Agent.GetPosition();
	}
	return Steering;
}

SteeringOutput Arrive::CalculateSteering(float DeltaTime, ASteeringAgent& Agent)
{
	SteeringOutput Steering{};

	double const dist{ FVector2D::Distance(Agent.GetPosition(), Target.Position) };
	if (Agent.GetMaxLinearSpeed() > OGSpeed)
	{
		OGSpeed = Agent.GetMaxLinearSpeed();
		SetLastAgent(Agent);
	}
	
	if (dist < GetSlowRadius())
	{
		// we have our distance, it is smaller so we need to modify the speed
		// we want to map the dist ontoo 1-0 with slowspeed = 1 and targt = 0
		// we subtract target, and devide by slow-target aka mult speedMapper
		
		Agent.SetMaxLinearSpeed(OGSpeed * (dist - GetTargetRadius()) * GetSpeedMapper());
	}
	else
	{
		Agent.SetMaxLinearSpeed(OGSpeed);
	}

	Steering.LinearVelocity = Target.Position - Agent.GetPosition();
	return Steering;
}

SteeringOutput Face::CalculateSteering(float DeltaTime, ASteeringAgent& Agent)
{
	SteeringOutput Steering{};
	Agent.SetIsAutoOrienting(false);
	// we can use x and y to get angle,
	Steering.AngularVelocity = atan2(Target.Position.Y - Agent.GetPosition().Y, Target.Position.X - Agent.GetPosition().X);
	return Steering;
}