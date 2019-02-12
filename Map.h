
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
	private:
		class Node;

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
		Liar::Uint AddVertex(const Liar::Vector2f&);
		Liar::Uint AddVertex(Liar::NAVDTYPE, Liar::NAVDTYPE);

		Liar::Uint GetNumPolygon() const { return m_numberPolygon; };
		Liar::Polygon* GetPolygon(Liar::Uint);

		void CalcBound(const Liar::Vector2f&);
		void CalcBound(const Liar::Polygon&);

		// add NavMesh cells;
		void AddNavMeshCell(Liar::Cell*);
		Liar::Uint NavMeshLinkCells(bool = true);

		bool InMap(Liar::NAVDTYPE, Liar::NAVDTYPE, bool = false);
		bool CanWalk(Liar::NAVDTYPE, Liar::NAVDTYPE);
		Liar::Vector2f** FindPath(Liar::NAVDTYPE, Liar::NAVDTYPE, Liar::NAVDTYPE, Liar::NAVDTYPE, Liar::Uint&, bool = true);

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

	private:
		void CalcBound();

#ifdef UNION_POLYGON
	private:
		Liar::Uint LinkToPolygon(Liar::Map::Node**, Liar::Uint, Liar::Map::Node**, Liar::Uint);
		int IntersectPoint(Liar::Map::Node**, Liar::Map::Node**, Liar::Uint&, Liar::Uint&, bool = true);
		int GetNodeIndex(Liar::Map::Node** cv, int, const Liar::Vector2f&);
		Liar::Uint UnionPolygons(const Liar::Polygon&, const Liar::Polygon&, bool = true);

	private:
		static Liar::Map::Node** m_nodes1;
		static Liar::Uint m_numberNode1;
		static Liar::Map::Node** m_nodes2;
		static Liar::Uint m_numberNode2;

		static void CreateNodes(Liar::Map const*, Liar::Uint, Liar::Uint, Liar::Map::Node**, Liar::Map::Node**, bool = true);
		static Liar::Map::Node** ExpandNodes(Liar::Map const*, Liar::Uint, Liar::Map::Node**, Liar::Uint&);

		bool CheckCross(Liar::NAVDTYPE r1[], Liar::NAVDTYPE r2[]);

	public:
		void UnionAll(bool = true);
		static void DisposeNodes();
#endif // UNION_POLYGON
	};

	class Map::Node :public Liar::MapSource
	{
	public:
		Node(Liar::Map const*);
		~Node();

	public:
		void Init(Liar::Map const*);
		void Set(Liar::Uint, bool isInters, bool main);

	public:
		/** 坐标点索引 */
		Liar::Uint v;
		/** 是否是交点 */
		bool i;
		/** 是否已处理过 */
		bool p;
		/** 进点--false； 出点--true */
		bool o;
		/** 交点的双向引用 */
		Node* other;
		/** 点是否在主多边形中*/
		bool isMain;
		/** 多边形的下一个点 */
		Node* next;
	};
}

#endif // !__MAP_H__