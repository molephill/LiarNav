#ifndef __NAVMESH_H__
#define __NAVMESH_H__

#include "Heap.h"
#include "WayPoint.h"
#include "MapSource.h"

#ifdef  EditorMod  
#define  NAVMESH_API _declspec(dllexport)  
#else  
#define  NAVMESH_API _declspec(dllimport)  
#endif 


namespace Liar
{
#ifdef EditorMod
	class NAVMESH_API NavMesh:public Liar::MapSource
#else
	class NavMesh:public Liar::MapSource
#endif // EditorMod
	{
	public:
		NavMesh(const Liar::Map*);
		~NavMesh();

	private:
		bool m_isLock;
		Liar::Heap* m_openList;

		Liar::Cell** m_closeList;
		Liar::Uint m_closeCount;

		Liar::Vector2f** m_path;
		Liar::Uint m_numPath;

		static Liar::Uint PATHMAX;

#ifdef FindNearest
		Liar::Cell** m_nearstCells;
		Liar::Uint m_nearstCount;
#endif // FindNearest

#ifdef EditorMod
		Cell** m_crossList;
		Liar::Uint m_crossCount;
#endif // EditorMod

#ifndef ShareFind
		Liar::Cell** m_testCells;
		Liar::Uint m_testCount;
		Liar::Cell* m_tmpClosetCell;
		Liar::Int m_pathSessionId;
#else
		static int PATHSESSIONID;
#endif // !ShareFind

	public:
		Liar::Vector2f** FindPath(NAVDTYPE startX, NAVDTYPE startY, NAVDTYPE endX, NAVDTYPE endY, Liar::Uint&, bool rw = true);
		void Init(const Liar::Map*);
		void Set(const Liar::Map*);
		void Dispose();
		void SetLock(bool lock) { m_isLock = lock; };

	private:
		Liar::Cell* FindClosestCell(const Vector2f&, bool = true);
		Liar::Cell* FindClosestCell(Liar::NAVDTYPE, Liar::NAVDTYPE, bool = true);
		
		Cell** GetCellPath(int&);
		
		void AddCrossCell(Cell*);

		Liar::Vector2f** GetPath(const Vector2f&, const Vector2f&, Liar::Uint&, bool = true);
		Liar::Vector2f** GetPath(Liar::NAVDTYPE, Liar::NAVDTYPE, Liar::NAVDTYPE, Liar::NAVDTYPE, Liar::Uint&, bool = true);

		Liar::Vector2f** AddPathPoint(const Liar::Vector2f&, Liar::Uint&);
		Liar::Vector2f** AddPathPoint(Liar::NAVDTYPE, Liar::NAVDTYPE, Liar::Uint&);

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
		void DisposePath();

#ifndef ShareFind
		void DisposeTestCells();
#endif // ShareFind


#if FindNearest
		void FindNearestPath(Liar::Uint, Liar::Vector2f**, Liar::Uint&);
#endif // FindNearest

#ifdef EditorMod
	public:
		void DisposeFindCtr();
		void GetCrossInfo(Liar::Cell**, Liar::Uint&);
#endif // EditorMod

#ifndef ShareFind
		Liar::Cell* AddTestCell(Liar::Cell*);
#endif // !ShareFind

	};
}

#endif // !__NAVMESH_H__