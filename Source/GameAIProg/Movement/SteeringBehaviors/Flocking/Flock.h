#pragma once

// Toggle this define to enable/disable spatial partitioning
//#define GAMEAI_USE_SPACE_PARTITIONING

#include "FlockingSteeringBehaviors.h"
#include "Movement/SteeringBehaviors/SteeringAgent.h"
#include "Movement/SteeringBehaviors/SteeringHelpers.h"
#include "Movement/SteeringBehaviors/CombinedSteering/CombinedSteeringBehaviors.h"
#include <memory>
#include "imgui.h"

#include "../SpacePartitioning/SpacePartitioning.h"
#ifdef GAMEAI_USE_SPACE_PARTITIONING
#include "../SpacePartitioning/SpacePartitioning.h"
#endif

class Flock final
{
public:
	Flock(
	UWorld* pWorld,
	TSubclassOf<ASteeringAgent> AgentClass,
	int FlockSize = 10, 
	float WorldSize = 100.f, 
	ASteeringAgent* const pAgentToEvade = nullptr, 
	bool bTrimWorld = false);

	~Flock();

	void Tick(float DeltaTime);
	void RenderDebug();
	void ImGuiRender(ImVec2 const& WindowPos, ImVec2 const& WindowSize);

#ifdef GAMEAI_USE_SPACE_PARTITIONING
	const TArray<ASteeringAgent*>& GetNeighbors() const { return pPartitionedSpace->GetNeighbors(); }
	int GetNrOfNeighbors() const { return pPartitionedSpace->GetNrOfNeighbors(); }
#else // No space partitioning
	void RegisterNeighbors(ASteeringAgent* const Agent);

	int GetNrOfNeighbors() const
	{
		if (useSpatialPartitioning)
		{
			return pPartitionedSpace->GetNrOfNeighbors();
		}
		return NrOfNeighbors;
	}
	const TArray<ASteeringAgent*>& GetNeighbors() const
	{ 
		if (useSpatialPartitioning)
		{
			return pPartitionedSpace->GetNeighbors();
		}
		return Neighbors;
	}

	float GetNeighborhoodRadius() { return NeighborhoodRadius; }

#endif // USE_SPACE_PARTITIONING

	FVector2D GetAverageNeighborPos() const;
	FVector2D GetAverageNeighborVelocity() const;

	void SetTarget_Seek(FSteeringParams const & Target);

private:
	// For debug rendering purposes
	UWorld* pWorld{nullptr};
	
	int FlockSize{50};
	TArray<ASteeringAgent*> Agents{};

	bool useSpatialPartitioning{false};
	std::unique_ptr<CellSpace> pPartitionedSpace{};
	int NrOfCellsX{ 10 };
	TArray<FVector2D> OldPositions{};

#ifdef GAMEAI_USE_SPACE_PARTITIONING
	std::unique_ptr<CellSpace> pPartitionedSpace{};
	int NrOfCellsX{ 10 };
	TArray<FVector2D> OldPositions{};
#else // No space partitioning
	TArray<ASteeringAgent*> Neighbors{};
#endif // USE_SPACE_PARTITIONING
	
	float NeighborhoodRadius{300.f};
	int NrOfNeighbors{5};

	ASteeringAgent* pAgentToEvade{nullptr};
	
	//Steering Behaviors
	std::unique_ptr<Separation> pSeparationBehavior{};
	std::unique_ptr<Cohesion> pCohesionBehavior{};
	std::unique_ptr<VelocityMatch> pVelMatchBehavior{};
	std::unique_ptr<Seek> pSeekBehavior{};
	std::unique_ptr<Wander> pWanderBehavior{};
	std::unique_ptr<Evade> pEvadeBehavior{};
	
	std::unique_ptr<BlendedSteering> pBlendedSteering{};
	std::unique_ptr<PrioritySteering> pPrioritySteering{};

	// UI and rendering
	bool DebugRenderSteering{true};
	bool DebugRenderNeighborhood{true};
	bool DebugRenderPartitions{true};

	void RenderNeighborhood();
};
