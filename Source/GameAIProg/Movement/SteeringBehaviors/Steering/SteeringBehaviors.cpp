#include "SteeringBehaviors.h"
#include "GameAIProg/Movement/SteeringBehaviors/SteeringAgent.h"

#include "imgui.h"
#include "CoreMinimal.h"
#include "DrawDebugHelpers.h"

//SEEK
//*******
// TODO: Do the Week01 assignment :^)

SteeringOutput Seek::CalculateSteering(float DeltaTime, ASteeringAgent& Agent)
{
	SteeringOutput Steering{};
	Steering.LinearVelocity = Target.Position - Agent.GetPosition();
	Steering.LinearVelocity.Normalize();
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
	Steering.LinearVelocity.Normalize();
	return Steering;
}

SteeringOutput Arrive::CalculateSteering(float DeltaTime, ASteeringAgent& Agent)
{
	SteeringOutput Steering{};

	double const dist{ FVector2D::Distance(Agent.GetPosition(), Target.Position) };
	if (Agent.GetMaxLinearSpeed() > OGSpeed)
	{
		OGSpeed = Agent.GetMaxLinearSpeed();
		LastAgent = &Agent;
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
	Steering.LinearVelocity.Normalize();
	return Steering;
}

Arrive::~Arrive()
{
	LastAgent->SetMaxLinearSpeed(OGSpeed);
}

SteeringOutput Face::CalculateSteering(float DeltaTime, ASteeringAgent& Agent)
{
	if (Agent.GetMaxLinearSpeed() > OGSpeed)
	{
		OGSpeed = Agent.GetMaxLinearSpeed();
		LastAgent = &Agent;
	}

	SteeringOutput Steering{};
	Agent.SetIsAutoOrienting(false);
	LastAgent = &Agent;
	// we can use x and y to get angle,

	double angle{ FMath::Atan2(Target.Position.Y - Agent.GetPosition().Y, Target.Position.X - Agent.GetPosition().X) };
	
	float curYaw_radians = FMath::DegreesToRadians(Agent.GetActorRotation().Yaw);
	float rotation = FMath::FindDeltaAngleRadians(curYaw_radians, angle);

	Steering.AngularVelocity = rotation;

	Agent.SetMaxLinearSpeed(0);
	return Steering;
}
Face::~Face()
{
	LastAgent->SetIsAutoOrienting(true);
	LastAgent->SetMaxLinearSpeed(OGSpeed);
}

SteeringOutput Persuit::CalculateSteering(float DeltaTime, ASteeringAgent& Agent)
{

	SteeringOutput Steering{};
	//Target.LinearVelocity
	double time{ FVector2D::Distance(Target.Position , Agent.GetPosition()) / Agent.GetMaxLinearSpeed() };

	Steering.LinearVelocity = (Target.Position + Target.LinearVelocity * time) - Agent.GetPosition();
	Steering.LinearVelocity.Normalize();
	return Steering;
}

SteeringOutput Evade::CalculateSteering(float DeltaTime, ASteeringAgent& Agent)
{

	SteeringOutput Steering{};

	DrawDebugSphere(
		Agent.GetWorld(),
		FVector((Target.Position), 0.f),
		15.f,
		12,
		FColor::Blue,
		false,
		0.f
	);

	if (FVector2D::Distance(Agent.GetPosition(), Target.Position) < 600.f)
	{
		Steering.IsValid = true;
		
	}
	else
	{
		Steering.IsValid = false;
		Steering.LinearVelocity = FVector2D::ZeroVector;
		Steering.AngularVelocity = 0.f;
		return Steering;
	}

	//Target.LinearVelocity
	double time{ FVector2D::Distance(Target.Position , Agent.GetPosition()) / Agent.GetMaxLinearSpeed() };

	Steering.LinearVelocity = Agent.GetPosition() - (Target.Position + Target.LinearVelocity * time);

	// Predicted position marker where the pursuer is expected to be
	DrawDebugSphere(
		Agent.GetWorld(),
		FVector((Target.Position + Target.LinearVelocity * time), 0.f),
		15.f,
		12,
		FColor::Yellow,
		false,
		0.f
	);

	// Line from agent to predicted position
	DrawDebugLine(
		Agent.GetWorld(),
		FVector(Agent.GetPosition(), 0.f),
		FVector((Target.Position + Target.LinearVelocity * time), 0.f),
		FColor::Red,
		false,
		0.f,
		0,
		2.f
	);

	
	
	Steering.LinearVelocity.Normalize();
	return Steering;
}

SteeringOutput Wander::CalculateSteering(float DeltaTime, ASteeringAgent& Agent)
{
	
	SteeringOutput Steering{};
	// we got the agent pos, we got the direction, we got the radius
	
	m_WanderAngle += FMath::RandRange(-m_MaxAngleChange, m_MaxAngleChange);
	if (m_WanderAngle > PI)
	{
		m_WanderAngle -= 2.f * PI;
	}
	if (m_WanderAngle < -PI)
	{
		m_WanderAngle += 2.f * PI;
	}
	FVector2D const direction = { Agent.GetActorForwardVector().X , Agent.GetActorForwardVector().Y };
	FVector2D const circleMid{ Agent.GetPosition() + direction * m_OffsetDistance };
	FVector2D const  circlePoint(FMath::Cos(m_WanderAngle), FMath::Sin(m_WanderAngle));
	Steering.LinearVelocity = (circleMid + circlePoint * m_Radius) - Agent.GetPosition();

	Steering.LinearVelocity.Normalize();

	if (Agent.GetDebugRenderingEnabled())
	{
		// need an fvector for circle mid
		FVector mid{ circleMid.X, circleMid.Y, 0};
		DrawDebugCircle(Agent.GetWorld(), mid, m_Radius, 50, FColor(0, 104, 167), false, -1, 0, 1, FVector{1,0,0}, FVector{ 0,1,0 }, true);
	}


	return Steering;
}
