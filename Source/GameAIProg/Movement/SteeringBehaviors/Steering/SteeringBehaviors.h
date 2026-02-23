#pragma once

#include <Movement/SteeringBehaviors/SteeringHelpers.h>
#include "Kismet/KismetMathLibrary.h"

class ASteeringAgent;

// SteeringBehavior base, all steering behaviors should derive from this.
class ISteeringBehavior
{
public:
	ISteeringBehavior() = default;
	virtual ~ISteeringBehavior() = default;

	// Override to implement your own behavior
	virtual SteeringOutput CalculateSteering(float DeltaT, ASteeringAgent & Agent) = 0;

	void SetTarget(const FTargetData& NewTarget) { Target = NewTarget; }
	
	template<class T, std::enable_if_t<std::is_base_of_v<ISteeringBehavior, T>>* = nullptr>
	T* As()
	{ return static_cast<T*>(this); }

protected:
	FTargetData Target;
};

// Your own SteeringBehaviors should follow here...

class Seek :public ISteeringBehavior
{
public:
	Seek() = default;
	virtual ~Seek() = default;

	virtual SteeringOutput CalculateSteering(float DeltaTime, ASteeringAgent& Agent) override;
};

class Flee :public ISteeringBehavior
{
public:
	Flee() = default;
	virtual ~Flee() = default;

	virtual SteeringOutput CalculateSteering(float DeltaTime, ASteeringAgent& Agent) override;
};


class Arrive :public ISteeringBehavior
{
public:
	Arrive()
		:SpeedMapper{ 1/(SlowRadius - TargetRadius) }
	{}
	virtual ~Arrive();

	virtual SteeringOutput CalculateSteering(float DeltaTime, ASteeringAgent& Agent) override;

	float GetSlowRadius() const { return SlowRadius; }
	float GetTargetRadius() const { return TargetRadius; }
	float GetSpeedMapper() const { return SpeedMapper; }

private:
	ASteeringAgent* LastAgent;
	float OGSpeed{};
	float SlowRadius{500.f};
	float TargetRadius{150.f};
	float SpeedMapper;
};

class Face :public ISteeringBehavior
{
public:
	Face() = default;
	virtual ~Face();

	virtual SteeringOutput CalculateSteering(float DeltaTime, ASteeringAgent& Agent) override;

private:
	ASteeringAgent* LastAgent;
	float OGSpeed{};
};

class Persuit :public ISteeringBehavior
{
public:
	Persuit() = default;
	virtual ~Persuit() = default;

	virtual SteeringOutput CalculateSteering(float DeltaTime, ASteeringAgent& Agent) override;

private:
};


class Evade :public ISteeringBehavior
{
public:
	Evade() = default;
	virtual ~Evade() = default;

	virtual SteeringOutput CalculateSteering(float DeltaTime, ASteeringAgent& Agent) override;

private:
};

class Wander :public ISteeringBehavior
{
public:
	Wander() = default;
	virtual ~Wander() = default;

	virtual SteeringOutput CalculateSteering(float DeltaTime, ASteeringAgent& Agent) override;

	void SetWanderOffset(float offset) { m_OffsetDistance = offset; }
	void SetWanderRadius(float offset) { m_Radius = offset; }
	void SetMaxAngleChange(float offset) { m_MaxAngleChange = offset; }
private:
	float m_OffsetDistance = 60.f;
	float m_Radius = 40.f;
	float m_MaxAngleChange = FMath::DegreesToRadians(45);
	float m_WanderAngle = 6.f;
};