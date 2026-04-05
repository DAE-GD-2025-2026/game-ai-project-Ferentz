#include "NavGraph.h"

#include "NavGraphNode.h"

GameAI::NavGraph::NavGraph(std::unique_ptr<TriPolygon>&& NavPoly)
	: Graph{ false }
	, pNavPoly{ std::move(NavPoly) }
{
	CreateNavigationGraph();
}

GameAI::NavGraph::NavGraph(const NavGraph& Other)
	: Graph(false)
{
	Nodes.reserve(Other.Nodes.size());
	for (std::unique_ptr<Node> const& OtherNode : Other.Nodes)
	{
		Nodes.push_back(std::make_unique<NavGraphNode>(*dynamic_cast<NavGraphNode*>(OtherNode.get())));
	}

	Connections.reserve(Other.Connections.size());
	for (std::unique_ptr<Connection> const& OtherConnection : Other.Connections)
	{
		Connections.push_back(std::make_unique<Connection>(*OtherConnection.get()));
	}
}

std::unique_ptr<GameAI::NavGraph> GameAI::NavGraph::Clone() const
{
	return std::make_unique<NavGraph>(*this);
}

int GameAI::NavGraph::GetNodeIdFromEdgeIndex(int EdgeIdx) const
{
	if (EdgeIdx >= 0)
	{
		for (auto const& pNode : Nodes)
		{
			if (reinterpret_cast<NavGraphNode*>(pNode.get())->GetEdgeIdx() == EdgeIdx)
			{
				return pNode->GetId();
			}
		}
	}

	return Graphs::InvalidNodeId;
}

bool GameAI::NavGraph::DoesEdgeHaveNode(int EdgeIdx) const
{
	if (EdgeIdx >= 0)
	{
		for (auto const& pNode : Nodes)
		{
			if (reinterpret_cast<NavGraphNode*>(pNode.get())->GetEdgeIdx() == EdgeIdx)
			{
				return true;
			}
		}
	}

	return false;
}

void GameAI::NavGraph::CreateNavigationGraph()
{
	auto& edges{ pNavPoly->GetEdges() };
	auto& triangles{ pNavPoly->GetTriangles() };
	for (auto& edge : edges)
	{
		auto count{
			std::count_if(
				triangles.begin(), triangles.end(),
				[&](auto triangle) { return triangle.HasEdge(edge); })
		};

		if (count > 1)
		{
			AddNode(std::make_unique<NavGraphNode>(
				FVector2D((edge.GetP1(*pNavPoly) + edge.GetP2(*pNavPoly)) / 2),
				pNavPoly->FindEdgeIndex(edge).value()
			));
		}
	}

	for (auto& triangle : triangles)
	{
		auto const& triangleEdges{ triangle.GetEdges() };
		std::optional<int> edgeIndices[3]{};
		int nodeIDs[3]{};
		bool bools[3]{};

		for (int i{}; i < 3; i++)
		{
			edgeIndices[i] = pNavPoly->FindEdgeIndex(triangleEdges[i]);
			nodeIDs[i] = GetNodeIdFromEdgeIndex(edgeIndices[i].value());
			bools[i] = (nodeIDs[i] == Graphs::InvalidNodeId) ?
				false : true;
		}

		auto AddNewConnection = [&](int one, int two) {
			auto const& firstNode = GetNode(nodeIDs[one]);
			auto const& secondNode = GetNode(nodeIDs[two]);
			float dist = FVector2D::Distance(firstNode->GetPosition(), secondNode->GetPosition());
			AddConnection(nodeIDs[one], nodeIDs[two], dist);
			};

		for (int i{}; i < 3; i++)
		{
			int j = i + 1;
			if (j > 2) j = 0;

			if(bools[i] && bools[j])
				AddNewConnection(i, j);
		}
	}
}
