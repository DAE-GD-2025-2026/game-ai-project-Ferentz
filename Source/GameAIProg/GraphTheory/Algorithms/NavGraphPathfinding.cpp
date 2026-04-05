#include "NavGraphPathfinding.h"

#include "AStar.h"
#include "PathSmoothing.h"
#include "VectorTypes.h"
#include "Shared/Graph/NavGraph/NavGraph.h"
#include "Shared/Graph/NavGraph/NavGraphNode.h"

using namespace GameAI;

std::vector<FVector2D> NavMeshPathfinding::FindPath(const FVector2D& startPos, const FVector2D& endPos,
	NavGraph* const pNavGraph, std::vector<FVector2D>& debugNodePositions, std::vector<NavLine>& debugPortals) 
{
	//Create the path to return
	std::vector<FVector2D> finalPath{};

	//Get the start and endTriangle
	auto startTriangle = pNavGraph->GetNavPolygon()->GetTriangleAtPosition(startPos, true);
	auto endTriangle = pNavGraph->GetNavPolygon()->GetTriangleAtPosition(endPos, true);

	if (!startTriangle || !endTriangle)
	{
		return finalPath;
	}

	if (startTriangle == endTriangle)
	{
		//make path froom start to end and return
		finalPath.emplace_back(startPos);
		finalPath.emplace_back(endPos);
		return finalPath;
	}

	//We have valid start/end triangles and they are not the same
	//=> Start looking for a path
	//Copy the graph

	auto newGraph = pNavGraph->Clone();

	
	auto AddNewConnection = [&](int insertNodeId, std::array<TriPolygon::Edge, 3> edges) {
		for (auto const& edge : edges)
		{
			auto edgeIndices = pNavGraph->GetNavPolygon()->FindEdgeIndex(edge);
			int nodeIDs = newGraph->GetNodeIdFromEdgeIndex(edgeIndices.value());

			if (nodeIDs != Graphs::InvalidNodeId)
			{
				auto const& firstNode = newGraph->GetNode(insertNodeId);
				auto const& secondNode = newGraph->GetNode(nodeIDs);
				float dist = FVector2D::Distance(firstNode->GetPosition(), secondNode->GetPosition());
				newGraph->AddConnection(insertNodeId, nodeIDs, dist);
				newGraph->AddConnection(nodeIDs, insertNodeId, dist);

			}
		}
		};
	//Create Extra node for the Start Node (Agent's position
	int startNodeId = newGraph->AddNode(std::make_unique<NavGraphNode>(startPos, -1));
	auto const& startTriangleEdges{ startTriangle->GetEdges() };
	AddNewConnection(startNodeId, startTriangleEdges);

	//Create extra node for the endNode
	int endNodeId = newGraph->AddNode(std::make_unique<NavGraphNode>(endPos, -1));
	auto const& endTriangleEdges{ endTriangle->GetEdges() };
	AddNewConnection(endNodeId, endTriangleEdges);

	//apparently this existss
	newGraph->SetConnectionCostsToDistances();
	//Run A star on new graph
	AStar pathfinder(newGraph.get(), HeuristicFunctions::Euclidean);
	/*TerrainNode* const startNode = newGraph.get()->GetNodeAs<TerrainNode>(startNodeId);
	TerrainNode* const endNode = newGraph.get()->GetNodeAs<TerrainNode>(endNodeId);*/

	auto FoundPath = pathfinder.FindPath(
		newGraph->GetNode(startNodeId).get(),
		newGraph->GetNode(endNodeId).get());

	finalPath.reserve(FoundPath.size());
	debugNodePositions.reserve(FoundPath.size());
	for (auto node : FoundPath)
	{
		finalPath.push_back(node->GetPosition());
		debugNodePositions.push_back(node->GetPosition());
	}
	

	//Debug Visualisation

	// Extra: Run optimiser on new graph (First check if everything works without SSFA!)
	 debugPortals = SSFA::FindPortals(FoundPath, *pNavGraph->GetNavPolygon());
	 finalPath = SSFA::OptimizePortals(debugPortals, *pNavGraph->GetNavPolygon());
	
	return finalPath;
}

std::vector<FVector2D> NavMeshPathfinding::FindPath(const FVector2D& startPos, const FVector2D& endPos, NavGraph* const pNavGraph)
{
	std::vector<FVector2D> debugNodePositions{};
	std::vector<NavLine> debugPortals{};

	return FindPath(startPos, endPos, pNavGraph, debugNodePositions, debugPortals);
}