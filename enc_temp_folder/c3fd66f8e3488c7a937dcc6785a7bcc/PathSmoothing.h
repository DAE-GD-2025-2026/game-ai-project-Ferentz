#pragma once
#include <vector>

#include "NavGraphPathfinding.h"
#include "Movement/Pathfinding/Navmesh/TriPolygon.h"
#include "Shared/Graph/Graph.h"
#include "Shared/Graph/NavGraph/NavGraphNode.h"

namespace GameAI
{
	class SSFA final
{
		static float Cross(FVector2D a, FVector2D b, FVector2D c) {
			return FVector2D::CrossProduct((b - a), (c - a));
		}
public:
	//=== SSFA Functions ===
	//--- References ---
	//http://digestingduck.blogspot.be/2010/03/simple-stupid-funnel-algorithm.html
	//https://gamedev.stackexchange.com/questions/68302/how-does-the-simple-stupid-funnel-algorithm-work
	static std::vector<NavLine> FindPortals(std::vector<Node*> const & Path, TriPolygon const & NavPoly)
	{
		//Container
		std::vector<NavLine> Portals = {};
		
		NavLine startPos;
		startPos.P1 = Path[0]->GetPosition();
		startPos.P2 = Path[0]->GetPosition();
		Portals.emplace_back(startPos);

		//For each node received, get it's corresponding line
		for (int i = 1; i < Path.size()-1; ++i)
		{
			
			auto node = Path[i];

			int edgeIndex = static_cast<NavGraphNode*>(node)->GetEdgeIdx();
			auto const & edges{ NavPoly.GetEdges() };
			if (edgeIndex > edges.size() || edgeIndex < 0)
			{
				continue;
			}

			auto prevNode = Path[i - 1];
			auto edge{ edges[edgeIndex] };

			FVector2D a{ FVector2D(prevNode->GetPosition()) };
			FVector2D b{ FVector2D(node->GetPosition()) };
			FVector2D c{ edge.GetP1(NavPoly)};
			FVector2D d{ edge.GetP2(NavPoly) };

			NavLine newPortal{};

			//Redetermine it's "orientation" based on the required path (left-right vs right-left) - p1 should be right point

			//float cross{ (b.X - a.X) * (c.Y - a.Y) - (b.Y - a.Y) * (c.X - a.X) };
			float crossResult(FVector2D::CrossProduct((b - a), (c - a)));
			if (Cross(a,b,c) < 0)
			{
				//is right
				// p1/c is right, p2/dis left
				newPortal.P1 = c;
				newPortal.P2 = d;
			}
			else
			{
				// is left
				//p1/c is left, p2/d is right
				newPortal.P1 = d;
				newPortal.P2 = c;
			}
			
			Portals.emplace_back(newPortal);
			//Store portal
		}

		NavLine endPortal{};
		endPortal.P1 = Path.back()->GetPosition();
		endPortal.P2 = Path.back()->GetPosition();
		Portals.emplace_back(endPortal);

		//Add degenerate portal to force end evaluation

		return Portals;
	}

	static std::vector<FVector2D> OptimizePortals( std::vector<NavLine> const & Portals, TriPolygon const & NavPoly)
	{
		std::vector<FVector2D> Path{};
		Path.push_back(Portals[0].P1);


		
		//P1 == right point of portal, P2 == left point of portal
		//--- RIGHT CHECK ---
			//1. See if moving funnel inwards - RIGHT

				//2. See if new line degenerates a line segment - RIGHT

					//Leftleg becomes new apex point

					//Calculate new legs (if not the end)
		int apexIndex = 0;
		int leftLegIndex = 0;
		int rightLegIndex = 0;

		auto prefPoint{ Portals[apexIndex].P1 };
		auto rightLeg{ Portals[rightLegIndex].P1 - prefPoint };
		auto leftLeg{ Portals[leftLegIndex].P2 - prefPoint };

		// angle can't increase, so need to save the angle
		for (int portalIdx = 1; portalIdx < Portals.size(); portalIdx++)
		{
			NavLine const & portal = Portals[portalIdx];
			//// Right Check
			//Create a vector2 newRightLeg = from the apexPoint to the p1 point of the portal
			auto pRight{ Portals[portalIdx].P1 - prefPoint };
			FVector2D newRight = portal.P1 - prefPoint;

			auto pLeft{ Portals[portalIdx].P2 - prefPoint };
			//	Check if going inwards(CCW) or not (Tip: Use Cross) (Tip: there’s a difference between < and <= )
			if (FVector2D::CrossProduct(rightLeg, pRight) <= 0)
			{
				//	If going inwards :
				
				// Check if we cross over the leftLeg(Tip : Use Cross again with the left leg)
				if (FVector2D::CrossProduct(leftLeg, pRight) > 0)
				{
					//	If not crossing over the leftLeg :
					// rightLeg = the newRightLeg
					rightLeg = pRight;
					//	rightLegIndex = the current portalIndex
					rightLegIndex = portalIdx;
				}
				else
				{
					//	If we do cross over the leftLeg :
					// Move the apexPos by adding the leftLeg to the apexPos
					prefPoint = Portals[leftLegIndex].P2;
					//	Set apexIndex to be the leftLegIndex
					//	Set portalIdx to be leftLegIndex + 1 (next portal to check)
					//	Set leftLegIndex and rightLegIndex to be equal to that new portalIdx

					Path.push_back(prefPoint);
					apexIndex = leftLegIndex;
					portalIdx = leftLegIndex + 1;
					leftLegIndex = portalIdx;
					rightLegIndex = portalIdx;

					rightLeg = Portals[rightLegIndex].P1 - prefPoint;
					leftLeg = Portals[leftLegIndex].P2 - prefPoint;
					continue;
				}
				
			}
			//	If not going inwards, do nothing and just go to the left check.
			if (FVector2D::CrossProduct(leftLeg, pLeft) >= 0)
			{
				//	If going inwards :

				// Check if we cross over the leftLeg(Tip : Use Cross again with the left leg)
				if (FVector2D::CrossProduct(rightLeg, pLeft) < 0)
				{
					//	If not crossing over the leftLeg :
					// rightLeg = the newRightLeg
					leftLeg = pLeft;
					//	rightLegIndex = the current portalIndex
					leftLegIndex = portalIdx;
				}
				else
				{
					//	If we do cross over the leftLeg :
					// Move the apexPos by adding the leftLeg to the apexPos
					prefPoint = Portals[rightLegIndex].P1;
					//	Set apexIndex to be the leftLegIndex
					//	Set portalIdx to be leftLegIndex + 1 (next portal to check)
					//	Set leftLegIndex and rightLegIndex to be equal to that new portalIdx

					Path.push_back(prefPoint);
					apexIndex = rightLegIndex;
					portalIdx = rightLegIndex + 1;
					leftLegIndex = portalIdx;
					rightLegIndex = portalIdx;

					rightLeg = Portals[rightLegIndex].P1 - prefPoint;
					leftLeg = Portals[leftLegIndex].P2 - prefPoint;
					continue;
				}
			}
		}

		Path.push_back(Portals.back().P1);


			//--- LEFT CHECK ---
			//1. See if moving funnel inwards - LEFT

				//2. See if new line degenerates a line segment - LEFT

					//Rightleg becomes new apex point

					//Calculate new legs (if not the end)


		// Add last path point

		return Path;
	}
private:
	SSFA() {};
	~SSFA() {};

	//inline float triarea2(const float* a, const float* b, const float* c)
	//{
	//	const float ax = b[0] - a[0];
	//	const float ay = b[1] - a[1];
	//	const float bx = c[0] - a[0];
	//	const float by = c[1] - a[1];
	//	return bx * ay - ax * by;
	//}

	//inline bool vequal(const float* a, const float* b)
	//{
	//	static const float eq = 0.001f * 0.001f;
	//	return vdistsqr(a, b) < eq;
	//}

	//int stringPull(const float* portals, int nportals,
	//	float* pts, const int maxPts)
	//{
	//	// Find straight path.
	//	int npts = 0;
	//	// Init scan state
	//	float portalApex[2], portalLeft[2], portalRight[2];
	//	int apexIndex = 0, leftIndex = 0, rightIndex = 0;
	//	vcpy(portalApex, &portals[0]);
	//	vcpy(portalLeft, &portals[0]);
	//	vcpy(portalRight, &portals[2]);

	//	// Add start point.
	//	vcpy(&pts[npts * 2], portalApex);
	//	npts++;

	//	for (int i = 1; i < nportals && npts < maxPts; ++i)
	//	{
	//		const float* left = &portals[i * 4 + 0];
	//		const float* right = &portals[i * 4 + 2];

	//		// Update right vertex.
	//		if (triarea2(portalApex, portalRight, right) <= 0.0f)
	//		{
	//			if (vequal(portalApex, portalRight) || triarea2(portalApex, portalLeft, right) > 0.0f)
	//			{
	//				// Tighten the funnel.
	//				vcpy(portalRight, right);
	//				rightIndex = i;
	//			}
	//			else
	//			{
	//				// Right over left, insert left to path and restart scan from portal left point.
	//				vcpy(&pts[npts * 2], portalLeft);
	//				npts++;
	//				// Make current left the new apex.
	//				vcpy(portalApex, portalLeft);
	//				apexIndex = leftIndex;
	//				// Reset portal
	//				vcpy(portalLeft, portalApex);
	//				vcpy(portalRight, portalApex);
	//				leftIndex = apexIndex;
	//				rightIndex = apexIndex;
	//				// Restart scan
	//				i = apexIndex;
	//				continue;
	//			}
	//		}

	//		// Update left vertex.
	//		if (triarea2(portalApex, portalLeft, left) >= 0.0f)
	//		{
	//			if (vequal(portalApex, portalLeft) || triarea2(portalApex, portalRight, left) < 0.0f)
	//			{
	//				// Tighten the funnel.
	//				vcpy(portalLeft, left);
	//				leftIndex = i;
	//			}
	//			else
	//			{
	//				// Left over right, insert right to path and restart scan from portal right point.
	//				vcpy(&pts[npts * 2], portalRight);
	//				npts++;
	//				// Make current right the new apex.
	//				vcpy(portalApex, portalRight);
	//				apexIndex = rightIndex;
	//				// Reset portal
	//				vcpy(portalLeft, portalApex);
	//				vcpy(portalRight, portalApex);
	//				leftIndex = apexIndex;
	//				rightIndex = apexIndex;
	//				// Restart scan
	//				i = apexIndex;
	//				continue;
	//			}
	//		}
	//	}
	//	// Append last point to path.
	//	if (npts < maxPts)
	//	{
	//		vcpy(&pts[npts * 2], &portals[(nportals - 1) * 4 + 0]);
	//		npts++;
	//	}

	//	return npts;
	//}
};
}
