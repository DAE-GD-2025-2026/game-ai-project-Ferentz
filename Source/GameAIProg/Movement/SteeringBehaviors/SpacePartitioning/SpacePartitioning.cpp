#include "SpacePartitioning.h"

// --- Cell ---
// ------------
Cell::Cell(float Left, float Bottom, float Width, float Height)
{
	BoundingBox.Min = { Left, Bottom };
	BoundingBox.Max = { BoundingBox.Min.X + Width, BoundingBox.Min.Y + Height };
}

std::vector<FVector2D> Cell::GetRectPoints() const
{
	const float left = BoundingBox.Min.X;
	const float bottom = BoundingBox.Min.Y;
	const float width = BoundingBox.Max.X - BoundingBox.Min.X;
	const float height = BoundingBox.Max.Y - BoundingBox.Min.Y;

	std::vector<FVector2D> rectPoints =
	{
		{ left , bottom  },
		{ left , bottom + height  },
		{ left + width , bottom + height },
		{ left + width , bottom  },
	};

	return rectPoints;
}

// --- Partitioned Space ---
// -------------------------
CellSpace::CellSpace(UWorld* pWorld, float Width, float Height, int Rows, int Cols, int MaxEntities)
	: pWorld{pWorld}
	, SpaceWidth{Width}
	, SpaceHeight{Height}
	, NrOfRows{Rows}
	, NrOfCols{Cols}
	, NrOfNeighbors{0}
{
	Neighbors.SetNum(MaxEntities);
	
	//calculate bounds of a cell
	CellWidth = Width / Cols;
	CellHeight = Height / Rows;

	// TODO create the cells
	// we got rows and colls, rows * colls = amount of cells
	Cells.reserve(Rows * Cols);
	for (int i = 0; i < Cols; ++i)
	{
		for (int j = 0; j < Rows; ++j)
		{
			Cells.emplace_back(
				Cell(
				-Width * 0.5f + CellWidth * i,
				-Height * 0.5f + CellHeight * j,
				CellWidth, CellHeight)
			);
		}
	}
}

void CellSpace::AddAgent(ASteeringAgent& Agent)
{
	// TODO Add the agent to the correct cell
	int const idx = PositionToIndex(FVector2D{ Agent.GetActorLocation().X,Agent.GetActorLocation().Y });

	if (std::find(
		Cells[idx].Agents.begin(),
		Cells[idx].Agents.end(),
		&Agent)	 != Cells[idx].Agents.end()) return;


	Cells[idx].Agents.push_back(&Agent);
}

void CellSpace::UpdateAgentCell(ASteeringAgent& Agent, const FVector2D& OldPos)
{
	//TODO Check if the agent needs to be moved to another cell.
	//TODO Use the calculated index for oldPos and currentPos for this

	int oldIdx = PositionToIndex(OldPos);

	FVector2D currPos{ Agent.GetActorLocation().X,Agent.GetActorLocation().Y };
	int curIdx = PositionToIndex(currPos);


	if (curIdx == oldIdx) return;

	if (oldIdx >= 0 && oldIdx < static_cast<int>(Cells.size()))
	{
		Cells[oldIdx].Agents.remove(&Agent);
	}
		

	if (curIdx >= 0 && curIdx < static_cast<int>(Cells.size()))
	{
		auto& list = Cells[curIdx].Agents;
		if (std::find(list.begin(), list.end(), &Agent) == list.end())
		{
			list.push_back(&Agent);
		}
	}
}

void CellSpace::RegisterNeighbors(ASteeringAgent& Agent, float QueryRadius)
{
	// TODO Register the neighbors for the provided agent
	// TODO Only check the cells that are within the radius of the neighborhood

	FVector2D pos{ Agent.GetActorLocation().X,Agent.GetActorLocation().Y };

	FRect NHRect{
		{pos.X - QueryRadius,pos.Y - QueryRadius},
		{ pos.X + QueryRadius, pos.Y + QueryRadius} };

	int const cellIDX{ PositionToIndex(pos) };
	int const cellIDXMin{ PositionToIndex(NHRect .Min) };
	int const cellIDXMax { PositionToIndex(NHRect.Max) };

	int const minRow{ (cellIDXMin - 1) / NrOfCols };
	int const minCol{ (cellIDXMin - 1) % NrOfCols };
	int const maxRow{ (cellIDXMax - 1) / NrOfCols };
	int const maxCol{ (cellIDXMax - 1) % NrOfCols };

	NrOfNeighbors = 0;
	for (int row = minRow; row <= maxRow; ++row)
	{
		for (int col = minCol; col <= maxCol; ++col)
		{
			// feels dumb... but the brains arent braining.
			// TODO:: clean up! going from index to rows/cols to index again is stupid
			int idx = row * NrOfCols + col;
			Cell& cell = Cells[idx];

			if (DoRectsOverlap(cell.BoundingBox, NHRect))
			{
				for (ASteeringAgent* potNeighbor : cell.Agents)
				{
					if (!IsValid(potNeighbor) || potNeighbor == &Agent)
						continue;

					const FVector2D neighPos{ potNeighbor->GetActorLocation().X, potNeighbor->GetActorLocation().Y };
					float dist{ FVector2D::Distance(neighPos, pos) };

					if (dist <= QueryRadius)
					{
						Neighbors[NrOfNeighbors] = potNeighbor;
						NrOfNeighbors++;

						if (NrOfNeighbors >= Neighbors.Num())return;
					}
				}
			}
		}
	}
}

void CellSpace::EmptyCells()
{
	for (Cell& c : Cells)
		c.Agents.clear();
}

void CellSpace::RenderCells() const
{
	// TODO Render the cells with the number of agents inside of it
	for (Cell cell : Cells)
	{
		DrawDebugBox(pWorld,
			FVector{ cell.BoundingBox.Min.X,cell.BoundingBox.Min.Y, 0 },
			FVector{ cell.BoundingBox.Max.X - cell.BoundingBox.Min.X,
			cell.BoundingBox.Max.Y - cell.BoundingBox.Min.Y, 0 }
			, FColor::Cyan);
	}
	
}

int CellSpace::PositionToIndex(FVector2D const & Pos) const
{
	// TODO Calculate the index of the cell based on the position
	float localX = Pos.X - CellOrigin.X;
	float localY = Pos.Y - CellOrigin.Y;

	//variable = (condition) ? expressionTrue : expressionFalse;
	int col = (localX >= SpaceWidth) ? NrOfCols - 1 : static_cast<int>(localX / CellWidth);
	int row = (localY >= SpaceHeight) ? NrOfCols - 1 : static_cast<int>(localY / CellHeight);

	return row * NrOfCols + col;
}

bool CellSpace::DoRectsOverlap(FRect const & RectA, FRect const & RectB)
{
	// Check if the rectangles are separated on either axis
	if (RectA.Max.X < RectB.Min.X || RectA.Min.X > RectB.Max.X) return false;
	if (RectA.Max.Y < RectB.Min.Y || RectA.Min.Y > RectB.Max.Y) return false;
    
	// If they are not separated, they must overlap
	return true;
}