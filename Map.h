
#ifndef __MAP_H__
#define __MAP_H__

#include "Polygon.h"
#include "NavMesh.h"

#ifdef  EditorMod  
#define  MAP_API _declspec(dllexport)  
#else  
#define  MAP_API _declspec(dllimport)  
#endif 

namespace Liar
{
#ifdef EditorMod
	class MAP_API Map
#else
	class Map
#endif // EditorMod
	{
	public:
		Map();
		~Map();

	private:
		Liar::Vector2f** m_vertexs;
		Liar::Uint m_numberVertex;

		Liar::Polygon** m_polygons;
		Liar::Uint m_numberPolygon;

		Liar::NAVDTYPE m_minX;
		Liar::NAVDTYPE m_minY;
		Liar::NAVDTYPE m_maxX;
		Liar::NAVDTYPE m_maxY;

		// navMesh
		Liar::NavMesh* m_navMesh;

	public:
		void Init();
		Liar::Vector2f* GetVertex(Liar::Uint) const;
		Liar::Uint AddPolygon(Liar::Vector2f*, Liar::Uint);
		Liar::Polygon& AutoAddPolygon();
		void DisposePolygon(Liar::Polygon*);
		Liar::Uint AddVertex(const Liar::Vector2f&);
		Liar::Uint AddVertex(Liar::NAVDTYPE, Liar::NAVDTYPE);

		Liar::Uint GetNumPolygon() const { return m_numberPolygon; };
		Liar::Uint GetNumPoints() const { return m_numberVertex; };
		Liar::Polygon* GetPolygon(Liar::Uint);

		void CalcBound(const Liar::Vector2f&);
		void CalcBound(const Liar::Polygon&);

		// add NavMesh cells;
		void AddNavMeshCell(Liar::Cell*);
		Liar::Uint NavMeshLinkCells(bool = true);

		bool InMap(Liar::NAVDTYPE, Liar::NAVDTYPE);
		bool CanWalk(Liar::NAVDTYPE, Liar::NAVDTYPE);
		Liar::Vector2f** FindPath(Liar::NAVDTYPE, Liar::NAVDTYPE, Liar::NAVDTYPE, Liar::NAVDTYPE, Liar::Uint&, bool = true);
		void CalcBound(Liar::Int = 0, bool = false);

		Liar::NAVDTYPE GetMinX() const { return m_minX; };
		Liar::NAVDTYPE GetMinY() const { return m_minY; };
		Liar::NAVDTYPE GetMaxX() const { return m_maxX; };
		Liar::NAVDTYPE GetMaxY() const { return m_maxY; };

#ifdef EditorMod
		Liar::Polygon** GetPolygons() const { return m_polygons; };
		Liar::Uint GetPolygonSize() const { return m_numberPolygon; };
		Liar::Cell** GetCells() const { return m_navMesh ? m_navMesh->GetCells() : nullptr; };
		Liar::Uint GetCellCount() const { return m_navMesh ? m_navMesh->GetCellCount() : 0; };
		void SetCrossInfo(Liar::Cell** crossList, Liar::Uint& crossCout) { m_navMesh->GetCrossInfo(crossList, crossCout); };
#endif // EditorMod

#ifdef UNION_POLYGON
		void UnionAll(bool = true);
#endif // UNION_POLYGON

#if defined(DEBUG_NIF) || defined(EditorMod)
		void WriteErlang(std::ofstream&);
#endif // DEBUG_NIF


	};
}

#endif // !__MAP_H__