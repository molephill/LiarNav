
#ifndef __MAP_H__
#define __MAP_H__

#ifdef  EditorMod  
#define  MAP_API _declspec(dllexport)  
#else  
#define  MAP_API _declspec(dllimport)  
#endif 

#include "Polygon.h"
#include "NavMesh.h"

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
		Liar::Uint AddVertex(const Liar::Vector2f&);

		Liar::Uint GetNumPolygon() const { return m_numberPolygon; };
		const Liar::Polygon* GetPolygon(Liar::Uint) const;
		Liar::Polygon* GetPolygon(Liar::Uint);

		void CalcBound(const Liar::Vector2f&);
		void CalcBound(const Liar::Polygon&);

		// add NavMesh cells;
		void AddNavMeshCell(const Liar::Cell*);
		void NavMeshLinkCells(bool = true);

#ifdef UNION_POLYGON
	private:
		void UnionAll(bool rw = true);
		Liar::Uint LinkToPolygon(Liar::Map::Node**, Liar::Uint, Liar::Map::Node**, Liar::Uint);
		int IntersectPoint(Liar::Map::Node**, Liar::Map::Node**, Liar::Uint&, Liar::Uint&, bool = true);
		int GetNodeIndex(Liar::Map::Node** cv, int, const Liar::Vector2f&);
		Liar::Uint UnionPolygons(const Liar::Polygon&, const Liar::Polygon&, bool = true);

	private:
		Liar::Map::Node** m_nodes1;
		Liar::Uint m_numberNode1;
		Liar::Map::Node** m_nodes2;
		Liar::Uint m_numberNode2;

	public:
		void CreateNodes(Liar::Map const*, Liar::Uint, Liar::Uint, Liar::Map::Node**, Liar::Map::Node**, bool = true);
		void ExpandNodes(Liar::Map const*, Liar::Uint, Liar::Map::Node**, Liar::Uint&);
		void DisposeNodes();
		bool CheckCross(NAVDTYPE r1[], NAVDTYPE r2[]);
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