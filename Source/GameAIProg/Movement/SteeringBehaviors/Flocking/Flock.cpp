#include "Flock.h"
#include "FlockingSteeringBehaviors.h"
#include "Shared/ImGuiHelpers.h"


Flock::Flock(
	UWorld* pWorld,
	TSubclassOf<ASteeringAgent> AgentClass,
	int FlockSize_in,
	float WorldSize,
	ASteeringAgent* const pAgentToEvade,
	bool bTrimWorld)
	: pWorld{ pWorld }
	, FlockSize{ FlockSize_in }
	, pAgentToEvade{ pAgentToEvade }
{

	pSeparationBehavior = std::make_unique<Separation>(this);
	pCohesionBehavior = std::make_unique<Cohesion>(this);
	pVelMatchBehavior = std::make_unique<VelocityMatch>(this);
	pSeekBehavior = std::make_unique<Seek>();
	pWanderBehavior = std::make_unique<Wander>();
	pEvadeBehavior = std::make_unique<Evade>();

	pBlendedSteering = std::make_unique<BlendedSteering>(
		std::vector<BlendedSteering::WeightedBehavior>{
			{ pSeparationBehavior.get(), 0.f },
			{ pCohesionBehavior.get(),    0.f },
			{ pVelMatchBehavior.get(),    0.f },
			{ pWanderBehavior.get(),      1.f }
				}
			);

	pPrioritySteering = std::make_unique<PrioritySteering>(std::vector<ISteeringBehavior*>{ pEvadeBehavior.get(), pBlendedSteering.get() });


	//Agents.SetNum(FlockSize);
	Agents.Reserve(FlockSize);
	Neighbors.Reserve(NrOfNeighbors);

	const float spawnArea{ 800 };

 // TODO: initialize the flock and the memory pool
	int newFlockSize{ FlockSize_in };
	
	for (int i{}; i < FlockSize; ++i)
	{
		ASteeringAgent* agent{};
		int tries{};
		while (!IsValid(agent) || tries > 10)
		{
			tries++;
			agent = pWorld->SpawnActor<ASteeringAgent>(AgentClass, FVector{ FMath::FRandRange(-spawnArea, spawnArea), FMath::FRandRange(-spawnArea, spawnArea),90 }, FRotator::ZeroRotator);
			if (IsValid(agent))
			{
				agent->SetSteeringBehavior(pPrioritySteering.get());
				Agents.Add(std::move(agent));
			}
			if (tries > 10)
			{
				newFlockSize--;
			}
		}
		
		
	}
	FlockSize = newFlockSize;
}

Flock::~Flock()
{
 // TODO: Cleanup any additional data
}

void Flock::Tick(float DeltaTime)
{
 // TODO: update the flock
 // TODO: for every agent:
  // TODO: register the neighbors for this agent (-> fill the memory pool with the neighbors for the currently evaluated agent)
  // TODO: update the agent (-> the steeringbehaviors use the neighbors in the memory pool)
  // TODO: trim the agent to the world

	for (ASteeringAgent* boid : Agents)
	{
		RegisterNeighbors(boid);
		boid->Tick(DeltaTime);
	}
}
 

void Flock::RenderDebug()
{
 // TODO: Render all the agents in the flock
	RenderNeighborhood();
}

void Flock::ImGuiRender(ImVec2 const& WindowPos, ImVec2 const& WindowSize)
{
#ifdef PLATFORM_WINDOWS
#pragma region UI
	//UI
	{
		//Setup
		bool bWindowActive = true;
		ImGui::SetNextWindowPos(WindowPos);
		ImGui::SetNextWindowSize(WindowSize);
		ImGui::Begin("Gameplay Programming", &bWindowActive, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse);

		//Elements
		ImGui::Text("CONTROLS");
		ImGui::Indent();
		ImGui::Text("LMB: place target");
		ImGui::Text("RMB: move cam.");
		ImGui::Text("Scrollwheel: zoom cam.");
		ImGui::Unindent();

		ImGui::Spacing();
		ImGui::Separator();
		ImGui::Spacing();
		ImGui::Spacing();

		ImGui::Text("STATS");
		ImGui::Indent();
		ImGui::Text("%.3f ms/frame", 1000.0f / ImGui::GetIO().Framerate);
		ImGui::Text("%.1f FPS", ImGui::GetIO().Framerate);
		ImGui::Unindent();

		ImGui::Spacing();
		ImGui::Separator();
		ImGui::Spacing();

		ImGui::Text("Flocking");
		ImGui::Spacing();

  // TODO: implement ImGUI checkboxes for debug rendering here

		/*
		bool DebugRenderSteering{false};
		bool DebugRenderNeighborhood{true};
		bool DebugRenderPartitions{true};
		*/

		bool isChecked = DebugRenderSteering;
		if (ImGui::Checkbox("Debug Rendering steering", &isChecked))
		{
			DebugRenderSteering = isChecked;
			for (ASteeringAgent* boid : Agents)
			{
				boid->SetDebugRenderingEnabled(isChecked);
			}
		}
		bool isChecked2 = DebugRenderNeighborhood;
		if (ImGui::Checkbox("Debug Rendering neighborhood", &isChecked2))
		{
			DebugRenderNeighborhood = isChecked2;
		}
		bool isChecked3 = DebugRenderPartitions;
		if (ImGui::Checkbox("Debug Rendering partitioning", &isChecked3))
		{
			DebugRenderPartitions = isChecked3;
		}
		ImGui::Spacing();
		

  // TODO: implement ImGUI sliders for steering behavior weights here
		//End

		ImGui::Text("Behavior Weights");
		ImGui::Spacing();
		if (pBlendedSteering)
		{
			// get pointers to weights
			float* wSep = pBlendedSteering->GetWeight(pSeparationBehavior.get());
			float* wCoh = pBlendedSteering->GetWeight(pCohesionBehavior.get());
			float* wAli = pBlendedSteering->GetWeight(pVelMatchBehavior.get());
			float* wWan = pBlendedSteering->GetWeight(pWanderBehavior.get());

			// local copies
			if (wSep)
			{
				float tmp = *wSep;
				if (ImGui::SliderFloat("Separation", &tmp, 0.f, 3.f, "%.2f"))
					*wSep = tmp;
			}
			if (wCoh)
			{
				float tmp = *wCoh;
				if (ImGui::SliderFloat("Cohesion", &tmp, 0.f, 3.f, "%.2f"))
					*wCoh = tmp;
			}
			if (wAli)
			{
				float tmp = *wAli;
				if (ImGui::SliderFloat("Alignment", &tmp, 0.f, 3.f, "%.2f"))
					*wAli = tmp;
			}
			if (wWan)
			{
				float tmp = *wWan;
				if (ImGui::SliderFloat("Wander", &tmp, 0.f, 3.f, "%.2f"))
					*wWan = tmp;
			}
		}

		ImGui::End();
	}
#pragma endregion
#endif
}

void Flock::RenderNeighborhood()
{
 // TODO: Debugrender the neighbors for the first agent in the flock
	if (!DebugRenderNeighborhood) return;
	ASteeringAgent* const first = Agents[0];
	if (!IsValid(first)) return;
	const float Z = first->GetActorLocation().Z; // keep it at agent height
	const FVector center(first->GetPosition(), Z);
	DrawDebugCircle(
		pWorld,
		center,
		NeighborhoodRadius,
		64,
		FColor::Magenta,
		false,
		0.f,   // one frame
		0,
		2.f,
		FVector(1, 0, 0),
		FVector(0, 1, 0),
		false
	);
}

#ifndef GAMEAI_USE_SPACE_PARTITIONING
void Flock::RegisterNeighbors(ASteeringAgent* const pAgent)
{
	Neighbors.Reset();
	for (int i{}; i < FlockSize; ++i)
	{
		if (pAgent == Agents[i]) continue;
		if (FVector2D::Distance(Agents[i]->GetPosition(), pAgent->GetPosition()) < NeighborhoodRadius)
		{
			if (Neighbors.Num() < NrOfNeighbors - 1)
			{
				Neighbors.Add(Agents[i]);
			}
		}
	}
}
#endif

FVector2D Flock::GetAverageNeighborPos() const
{
	FVector2D avgPosition = FVector2D::ZeroVector;

	
	for (ASteeringAgent* neighbor : Neighbors)
	{
		avgPosition += neighbor->GetPosition();
	}

	avgPosition /= Neighbors.Num();

	return avgPosition;
}

FVector2D Flock::GetAverageNeighborVelocity() const
{
	FVector2D avgVelocity = FVector2D::ZeroVector;
	
	for (ASteeringAgent* neighbor : Neighbors)
	{
		avgVelocity += neighbor->GetLinearVelocity();
	}

	avgVelocity /= Neighbors.Num();

	return avgVelocity;
}

void Flock::SetTarget_Seek(FSteeringParams const& Target)
{
	pSeekBehavior->SetTarget(Target);
}

