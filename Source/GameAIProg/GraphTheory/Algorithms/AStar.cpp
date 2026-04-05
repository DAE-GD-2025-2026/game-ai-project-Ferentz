#include "AStar.h"

using namespace GameAI;

AStar::AStar(Graph* const pGraph, HeuristicFunctions::Heuristic hFunction)
	: pGraph(pGraph)
	, HeuristicFunction(hFunction)
{
}

inline bool ExistInList_AndRemoveIf(std::vector<AStar::NodeRecord>& list, Node* pNextNode, float currentCost)
{
	auto it = std::find_if(list.begin(), list.end(),
		[&](const AStar::NodeRecord& record)
		{
			return record.pNode == pNextNode;
		});

	if (it != list.end())
	{
		if (it->costSoFar > currentCost)
		{
			list.erase(it);
			return false;
		}
		return true;
	}

	return false;
}

std::vector<Node*>AStar::FindPath(Node* const pStartNode, Node* const pGoalNode)
{
	std::vector<Node*> path{};
	std::vector<NodeRecord> openList{};
	std::vector<NodeRecord> closedList{};

	NodeRecord currentNodeRecord{};
	NodeRecord startRecord{ };
	startRecord.pNode = pStartNode;
	startRecord.pConnection = nullptr;
	startRecord.costSoFar = 0;
	startRecord.estimatedTotalCost = GetHeuristicCost(pStartNode, pGoalNode);
	openList.push_back(startRecord);

	while (!openList.empty())
	{
		auto curNodeRecItter = std::min_element(openList.begin(), openList.end());
		currentNodeRecord = *curNodeRecItter;
		if (currentNodeRecord.pNode == pGoalNode) break;
		else
		{
			std::vector<Connection*> connections 
				= pGraph->FindConnectionsFrom(currentNodeRecord.pNode->GetId());
			// C
			for (Connection* connection : connections)
			{
				Node* pNextNode = pGraph->GetNode(connection->GetToId()).get();
				float gCostSF = currentNodeRecord.costSoFar + connection->GetWeight(); // why slides sat GetCost?

				if (ExistInList_AndRemoveIf(closedList, pNextNode, gCostSF)) continue;

				if (ExistInList_AndRemoveIf(openList, pNextNode, gCostSF)) continue;

				NodeRecord newNodeRecord{};
				newNodeRecord.costSoFar = gCostSF;
				newNodeRecord.pConnection = connection;
				newNodeRecord.pNode = pNextNode;
				newNodeRecord.estimatedTotalCost = gCostSF + GetHeuristicCost(pNextNode, pGoalNode);
				
				openList.push_back(newNodeRecord);
			}

			// G
			openList.erase( std::remove(openList.begin(), openList.end(), currentNodeRecord), openList.end());
			closedList.push_back(currentNodeRecord);
		}
	}

	if (currentNodeRecord.pNode != pGoalNode) return path;

	while (currentNodeRecord.pNode != pStartNode)
	{
		path.push_back(currentNodeRecord.pNode);
		int fromId = currentNodeRecord.pConnection->GetFromId();

		currentNodeRecord = *std::find_if(closedList.begin(), closedList.end(),
			[&](const NodeRecord& record)
			{
				return record.pNode->GetId() == fromId;
			});
	}
	
	path.push_back(pStartNode);
	std::reverse(path.begin(), path.end());

	return path;
}

float AStar::GetHeuristicCost(Node* const pStartNode, Node* const pEndNode) const
{
	FVector2D toDestination = pGraph->GetNode(pEndNode->GetId())->GetPosition() - pGraph->GetNode(pStartNode->GetId())->GetPosition();
	return HeuristicFunction(abs(toDestination.X), abs(toDestination.Y));
}