#ifndef __SINGLEFINDPATH_H__
#define __SINGLEFINDPATH_H__

#include "Map.h"

namespace Liar
{
	class SingleFindPath
	{
	public:
		SingleFindPath();
		~SingleFindPath();

	private:
		Map* _map;
		int _pathsession;
		Heap* _openList;
		Cell** _closeList;
		int _closeCount;

#ifdef FindNearest
		Cell** _nearstCells;
		int _nearstCount;
#endif // FindNearest


	public:
		void Init(Map*);
		Vector2f* Path(NAVDTYPE startX, NAVDTYPE startY, NAVDTYPE endX, NAVDTYPE endY, int&, bool rw = true);

	private:
		Cell* FindClosestCell(const Vector2f& pt, bool rw = true);
		Cell* FindClosestCell(NAVDTYPE, NAVDTYPE, bool = true);
		Vector2f* BuildPath(Cell*, const Vector2f&, Cell*, const Vector2f&, int&, bool rw = true);
		Vector2f* BuildPath(Cell* startCell, NAVDTYPE, NAVDTYPE
			, Cell* endCell, NAVDTYPE, NAVDTYPE, int&, bool rw = true);
		Vector2f* GetPath(const Vector2f&, const Vector2f&, int&, bool rw = true);
		Vector2f* GetPath(NAVDTYPE, NAVDTYPE, NAVDTYPE, NAVDTYPE, int&, bool rw = true);
		Cell** GetCellPath(int&);
		void GetFurthestWayPoint(WayPoint* w, Cell* CellPath[], int cellCount, const Vector2f&, Line2d*, Line2d*, bool rw = true);
		void GetFurthestWayPoint(WayPoint* w, Cell* CellPath[], int cellCount, NAVDTYPE, NAVDTYPE, Line2d*, Line2d*, bool rw = true);
		Cell* AddTestCell(Cell*);
		void AddCrossCell(Cell*);
		void DestoryTestCell();
		bool TestOneLine2D(const Vector2f&, const Vector2f&, Cell*, Cell*);
		bool TestOneLine2D(NAVDTYPE, NAVDTYPE, NAVDTYPE, NAVDTYPE, Cell*, Cell*);
		int TestLineCell(const Cell&, const Vector2f& start, const Vector2f& end);
		int TestLineCell(const Cell&, NAVDTYPE, NAVDTYPE, NAVDTYPE, NAVDTYPE);
		Cell* GetCrossCell(const Vector2f&, const Vector2f&, Cell&, Cell**, int, const Cell&, bool findCross = false);
		Cell* GetCrossCell(NAVDTYPE, NAVDTYPE, NAVDTYPE, NAVDTYPE, Cell&, Cell**, int, const Cell&, bool findCross = false);
		int LineIntersectSide(const Vector2f& A, const Vector2f& B, const Vector2f& C, const Vector2f& D);
		int LineIntersectSide(NAVDTYPE, NAVDTYPE, NAVDTYPE, NAVDTYPE, const Vector2f& C, const Vector2f& D);
		int LineIntersectSide(const Vector2f& A, const Vector2f& B, NAVDTYPE, NAVDTYPE, NAVDTYPE, NAVDTYPE);
		int LineIntersectSide(NAVDTYPE, NAVDTYPE, NAVDTYPE, NAVDTYPE, NAVDTYPE, NAVDTYPE, NAVDTYPE, NAVDTYPE);
		bool GetCrossVector2f(const Vector2f&, const Vector2f&, const Vector2f&, const Vector2f&, NAVDTYPE&, NAVDTYPE&);

#ifdef FindNearest
		void FindNearestPath(int, Vector2f*, int&);
#endif // FindNearest

#ifdef EditorMod
	private:
		Cell** _crossList;
		int _crossCount;

	public:
		Cell** GetCrossList() { return _crossList; };
		int GetCrossCount() { return _crossCount; };
#endif // EditorMod
	};
}
#endif // !__SINGLEFINDPATH_H__