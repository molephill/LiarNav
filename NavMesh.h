#ifndef __NAVMESH_H__
#define __NAVMESH_H__

#include "Heap.h"
#include "WayPoint.h"

#ifdef  EditorMod  
#define  NAVMESH_API _declspec(dllexport)  
#else  
#define  NAVMESH_API _declspec(dllimport)  
#endif 


namespace Liar
{
#ifdef EditorMod
	class NAVMESH_API NavMesh
#else
	class NavMesh
#endif // EditorMod
	{
	public:
		NavMesh();
		~NavMesh();

	private:
		Liar::Cell** m_cells;
		Liar::Uint m_numberCell;

#ifdef ShareFind
		Liar::Heap* m_openList;
		Liar::Cell** m_closeList;
		Liar::Uint m_closeCount;

#ifdef FindNearest
		Liar::Cell** m_nearstCells;
		Liar::Uint m_nearstCount;
#endif // FindNearest

#ifdef EditorMod
		Cell** m_crossList;
		Liar::Uint m_crossCount;
#endif // EditorMod


		static int PATHSESSIONID;
#endif // ShareFind

	public:
#ifdef ShareFind
		Liar::Vector2f** FindPath(NAVDTYPE startX, NAVDTYPE startY, NAVDTYPE endX, NAVDTYPE endY, Liar::Uint&, bool rw = true);
#endif // ShareFind
		void Set();
		void AddCell(Liar::Cell*);
		void LinkCells(bool = true);

#ifdef ShareFind
	private:
		Liar::Cell* FindClosestCell(const Vector2f&, bool = true);
		Liar::Cell* FindClosestCell(Liar::NAVDTYPE, Liar::NAVDTYPE, bool = true);
		
		Cell** GetCellPath(int&);
		void GetFurthestWayPoint(WayPoint* w, Cell* CellPath[], int cellCount, Vector2f* end, Line2d*, Line2d*, bool rw = true);
		void AddCrossCell(Cell*);

		Liar::Vector2f** GetPath(const Vector2f&, const Vector2f&, Liar::Uint&, bool = true);
		Liar::Vector2f** GetPath(Liar::NAVDTYPE, Liar::NAVDTYPE, Liar::NAVDTYPE, Liar::NAVDTYPE, Liar::Uint&, bool = true);

		Liar::Vector2f** AddPathPoint(Liar::Vector2f**, const Liar::Vector2f&, Liar::Uint&);
		Liar::Vector2f** AddPathPoint(Liar::Vector2f**, Liar::NAVDTYPE, Liar::NAVDTYPE, Liar::Uint&);

		Liar::Vector2f** BuildPath(Liar::Cell*, const Liar::Vector2f&, Liar::Cell*, const Liar::Vector2f&, Liar::Uint&, bool = true);
		Liar::Vector2f** BuildPath(Liar::Cell*, Liar::NAVDTYPE, Liar::NAVDTYPE, Liar::Cell*, Liar::NAVDTYPE, Liar::NAVDTYPE, Liar::Uint&, bool = true);

		bool TestOneLine2D(const Liar::Vector2f&, const Liar::Vector2f&, Liar::Cell*, Liar::Cell*);
		bool TestOneLine2D(Liar::NAVDTYPE, Liar::NAVDTYPE, Liar::NAVDTYPE, Liar::NAVDTYPE, Liar::Cell*, Liar::Cell*);

		Liar::Cell* GetCrossCell(const Vector2f&, const Vector2f&, Liar::Cell&, Liar::Cell**, int, bool findCross = false);
		Liar::Cell* GetCrossCell(Liar::NAVDTYPE, Liar::NAVDTYPE, Liar::NAVDTYPE, Liar::NAVDTYPE, Liar::Cell&, Liar::Cell**, int, bool findCross = false);

		int TestLineCell(const Cell&, const Vector2f& start, const Vector2f& end);
		int TestLineCell(const Cell&, Liar::NAVDTYPE, Liar::NAVDTYPE, Liar::NAVDTYPE, Liar::NAVDTYPE);

		int LineIntersectSide(const Vector2f&, const Vector2f&, const Vector2f&, const Vector2f&);
		int LineIntersectSide(const Vector2f&, const Vector2f&, Liar::NAVDTYPE, Liar::NAVDTYPE, Liar::NAVDTYPE, Liar::NAVDTYPE);
		int LineIntersectSide(Liar::NAVDTYPE, Liar::NAVDTYPE, Liar::NAVDTYPE, Liar::NAVDTYPE, const Vector2f&, const Vector2f&);
		int LineIntersectSide(Liar::NAVDTYPE, Liar::NAVDTYPE, Liar::NAVDTYPE, Liar::NAVDTYPE, Liar::NAVDTYPE, Liar::NAVDTYPE, Liar::NAVDTYPE, Liar::NAVDTYPE);

		bool GetCrossVector2f(const Vector2f&, const Vector2f&, const Vector2f&, const Vector2f&, NAVDTYPE&, NAVDTYPE&);

#if FindNearest
		void FindNearestPath(int, Vector2f*, int&);
#endif // FindNearest

#ifdef EditorMod
	public:
		void DisposeFindCtr();
#endif // EditorMod

#endif // ShareFind
	};
}

#endif // !__NAVMESH_H__