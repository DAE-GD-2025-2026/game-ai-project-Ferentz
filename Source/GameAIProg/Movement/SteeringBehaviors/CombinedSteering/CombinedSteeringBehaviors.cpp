
#include "CombinedSteeringBehaviors.h"
#include <algorithm>
#include "../SteeringAgent.h"

BlendedSteering::BlendedSteering(const std::vector<WeightedBehavior>& WeightedBehaviors)
	:WeightedBehaviors(WeightedBehaviors)
{};

//****************
//BLENDED STEERING
SteeringOutput BlendedSteering::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput BlendedSteering = {};
	// TODO: Calculate the weighted average steeringbehavior
	for (auto & behaviour : WeightedBehaviors)
	{
		// for eache behaviour, we need to get the output, combine them acording to their weight.
		SteeringOutput tempSteering = behaviour.pBehavior->CalculateSteering(DeltaT, Agent);
		if (!tempSteering.IsValid) continue;
		tempSteering.AngularVelocity *= behaviour.Weight;
		tempSteering.LinearVelocity *= behaviour.Weight;
		BlendedSteering += tempSteering;

	}

	// TODO: Add debug 
	const FVector start = FVector(Agent.GetPosition(), 0.f);
	const FVector2D dir2D = BlendedSteering.LinearVelocity.GetSafeNormal();
	const FVector end = start + FVector(dir2D, 0.f) * 200.f;

	DrawDebugLine(Agent.GetWorld(), start, end, FColor::Magenta, false, 0.f, 0, 2.f);

	return BlendedSteering;
}

float* BlendedSteering::GetWeight(ISteeringBehavior* const SteeringBehavior)
{
	auto it = find_if(WeightedBehaviors.begin(),
		WeightedBehaviors.end(),
		[SteeringBehavior](const WeightedBehavior& Elem)
		{
			return Elem.pBehavior == SteeringBehavior;
		}
	);

	if(it!= WeightedBehaviors.end())
		return &it->Weight;
	
	return nullptr;
}

//*****************
//PRIORITY STEERING
SteeringOutput PrioritySteering::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput Steering = {};

	for (ISteeringBehavior* const pBehavior : m_PriorityBehaviors)
	{
		Steering = pBehavior->CalculateSteering(DeltaT, Agent);

		if (Steering.IsValid)
			break;
	}

	//If non of the behavior return a valid output, last behavior is returned
	return Steering;
}

void PrioritySteering::SetTarget(const FTargetData& NewTarget)
{
	for (ISteeringBehavior* const pBehavior : m_PriorityBehaviors)
	{
		pBehavior->SetTarget(NewTarget);
	}
}