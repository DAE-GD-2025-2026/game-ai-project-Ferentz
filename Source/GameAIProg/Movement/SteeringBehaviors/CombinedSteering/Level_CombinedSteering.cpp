#include "Level_CombinedSteering.h"

#include "imgui.h"

#include "CombinedSteeringBehaviors.h"

// Sets default values
ALevel_CombinedSteering::ALevel_CombinedSteering()
{
	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;
}

// Called when the game starts or when spawned
void ALevel_CombinedSteering::BeginPlay()
{
	Super::BeginPlay();

	m_pCombinedAgent = GetWorld()->SpawnActor<ASteeringAgent>(SteeringAgentClass, FVector{ 0,0,90 }, FRotator::ZeroRotator);
	m_pSeek = new Seek();
	m_pWander = new Wander();

	std::vector<BlendedSteering::WeightedBehavior> behaviours{ { m_pSeek, 0.5f },{ m_pWander, 0.5f } };
	m_pCombined = new BlendedSteering(behaviours);

	m_pCombinedAgent->SetSteeringBehavior(m_pCombined);


	m_pPriorityAgent = GetWorld()->SpawnActor<ASteeringAgent>(SteeringAgentClass, FVector{ 0,0,90 }, FRotator::ZeroRotator);
	m_pEvade = new Evade();
	m_pWander2 = new Wander();

	//std::vector<BlendedSteering::WeightedBehavior> behaviours{ { m_pSeek, 0.5f },{ m_pWander, 0.5f } };
	m_pPrioritized = new PrioritySteering({ m_pEvade, m_pWander2 });

	

	m_pPriorityAgent->SetSteeringBehavior(m_pPrioritized);

	
}

void ALevel_CombinedSteering::BeginDestroy()
{
	Super::BeginDestroy();
	
	
}

// Called every frame
void ALevel_CombinedSteering::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
	
	if (m_pSeek)
	{
		m_pSeek->SetTarget(MouseTarget);
	}

	if (m_pPrioritized && m_pCombinedAgent)
	{
		FTargetData Target;
		Target.Position = m_pCombinedAgent->GetPosition();
		Target.Orientation = m_pCombinedAgent->GetRotation();
		Target.LinearVelocity = m_pCombinedAgent->GetLinearVelocity();
		Target.AngularVelocity = m_pCombinedAgent->GetAngularVelocity();

		m_pPrioritized->SetTarget(Target);
	}

#pragma region UI
	//UI
	{
		//Setup
		bool windowActive = true;
		ImGui::SetNextWindowPos(WindowPos);
		ImGui::SetNextWindowSize(WindowSize);
		ImGui::Begin("Game AI", &windowActive, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse);
	
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
		ImGui::Spacing();
	
		ImGui::Text("Flocking");
		ImGui::Spacing();
		ImGui::Spacing();
	
		if (ImGui::Checkbox("Debug Rendering", &CanDebugRender))
		{
   // TODO: Handle the debug rendering of your agents here :)
		}
		ImGui::Checkbox("Trim World", &TrimWorld->bShouldTrimWorld);
		if (TrimWorld->bShouldTrimWorld)
		{
			ImGuiHelpers::ImGuiSliderFloatWithSetter("Trim Size",
				TrimWorld->GetTrimWorldSize(), 1000.f, 3000.f,
				[this](float InVal) { TrimWorld->SetTrimWorldSize(InVal); });
		}
		
		ImGui::Spacing();
		ImGui::Spacing();
		ImGui::Spacing();
	
		ImGui::Text("Behavior Weights");
		ImGui::Spacing();

		if (m_pCombined && m_pSeek && m_pWander)
		{
			// Get pointers to weights inside blended steering
			float* SeekW = m_pCombined->GetWeight(m_pSeek);
			float* WanderW = m_pCombined->GetWeight(m_pWander);

			if (SeekW && WanderW)
			{
				// One slider controls both: Wander = 1 - Seek
				float seekWeight = *SeekW; // current value

				if (ImGui::SliderFloat("Seek vs Wander", &seekWeight, 0.f, 1.f, "Seek %.2f"))
				{
					*SeekW = seekWeight;
					*WanderW = 1.f - seekWeight;
				}

				// Optional: show numbers
				ImGui::Text("Seek: %.2f   Wander: %.2f", *SeekW, *WanderW);
			}
			else
			{
				ImGui::Text("Weights not found (did you add both behaviors to BlendedSteering?)");
			}
		}
		else
		{
			ImGui::Text("Blended/Seek/Wander not initialized yet");
		}

		
		

		// ImGuiHelpers::ImGuiSliderFloatWithSetter("Seek",
		// 	pBlendedSteering->GetWeightedBehaviorsRef()[0].Weight, 0.f, 1.f,
		// 	[this](float InVal) { pBlendedSteering->GetWeightedBehaviorsRef()[0].Weight = InVal; }, "%.2f");
		//
		// ImGuiHelpers::ImGuiSliderFloatWithSetter("Wander",
		// pBlendedSteering->GetWeightedBehaviorsRef()[1].Weight, 0.f, 1.f,
		// [this](float InVal) { pBlendedSteering->GetWeightedBehaviorsRef()[1].Weight = InVal; }, "%.2f");
	
		//End
		ImGui::End();
	}
#pragma endregion
	
	// Combined Steering Update
 // TODO: implement handling mouse click input for seek
 // TODO: implement Make sure to also evade the wanderer
}
