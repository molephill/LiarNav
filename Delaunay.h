
#ifndef __DELAUNAY_H__
#define __DELAUNAY_H__

#include "Map.h"
#include "Cell.h"

#ifdef  EditorMod  
#define  DELAUNAY_API _declspec(dllexport)  
#else  
#define  DELAUNAY_API _declspec(dllimport)  
#endif 

namespace Liar
{
#ifdef EditorMod
	class DELAUNAY_API Delaunay
#else
	class Delaunay
#endif // EditorMod
	{
	private:
		class Circle;
		class Node;

	private:
		static Liar::Line2d** m_line2ds;
		static Liar::Uint m_totalLines;
		static Liar::Uint m_curNumLines;
		static Liar::Delaunay::Circle* m_circle;
		static Liar::NAVDTYPE* m_tmpRange;
		// findDT
		static Liar::Uint* m_findDTPoints;
		static Liar::Uint m_numFindDTPoints;

		static Liar::Line2d* m_edge;
		static Liar::Vector2f* m_interscetVector;

#ifdef INFLATE
		static Liar::Vector2f** m_inflates;
		static Liar::Uint m_allNumberInflate;
		static Liar::Uint m_curNumberInflate;
#endif // INFLATE


	public:
#ifdef EditorMod
		static Liar::Int mapId;
#endif // EditorMod

		static Liar::Uint Set(Liar::Map&, bool = true, Liar::Uint = 0);
		static void Dispose();

		static Liar::LineClassification Intersection(const Liar::Line2d&, const Liar::Line2d&, Liar::Vector2f*);
		static Liar::LineClassification Intersection(const Liar::Vector2f&, const Liar::Vector2f&, const Liar::Line2d&, Liar::Vector2f*);
		static Liar::LineClassification Intersection(const Liar::Vector2f&, const Liar::Vector2f&, const Liar::Vector2f&, const Liar::Vector2f&, Liar::Vector2f*);

#ifdef INFLATE
		static void AddVertex(Liar::NAVDTYPE, Liar::NAVDTYPE);
		static void Inflate(Liar::Map&, Liar::Polygon&, bool = true, Liar::NAVDTYPE = 0.01f);
		static void ResetInFlate();
#endif // INFLATE

		static void AddVertex(Liar::Map&, Liar::Polygon&, Liar::NAVDTYPE, Liar::NAVDTYPE);
		static void AddVertex(Liar::Map&, Liar::Polygon&, const Liar::Vector2f&);

	private:
		static void BuildEdges(Liar::Map&);
		static void BuildTrianges(Liar::Map&, bool = true, Liar::Uint = 0);
		static Liar::Line2d** BuildTrianges(const Liar::Vector2f&, const Liar::Vector2f&, Liar::Line2d**, Liar::Int&, Liar::Map&, Liar::Uint, Liar::Uint);
		static Liar::Line2d& GetBoundEdage(Liar::Map&, bool = true, Liar::Uint = 0);

		static Liar::Int FindDT(Liar::Map&, const Liar::Line2d&, Liar::Vector2f&, bool = true);

		static bool IsVisiblePointOfLine(const Vector2f&, const Line2d&, Liar::Vector2f&, bool = true);
		static bool IsVisibleIn2Point(const Liar::Vector2f&, const Liar::Vector2f&, Liar::Vector2f&);

		static void CircumCircle(const Liar::Vector2f&, const Liar::Vector2f&, const Liar::Vector2f&, Liar::Delaunay::Circle& cir);
		static void CircleBounds(const Liar::Delaunay::Circle&, Liar::NAVDTYPE*);

		static Liar::NAVDTYPE LineAngle(const Liar::Vector2f&, const Liar::Vector2f&, const Liar::Vector2f&);

		static Liar::Line2d** RemovePosLine(Liar::Line2d** lv, int&, int pos);
		static Liar::Int FindLinePos(const Liar::Vector2f&, const Liar::Vector2f&, Liar::Line2d**, Liar::Uint);

#ifdef EditorMod
		static void PrintEdges(const Liar::Map&);
#endif // EditorMod

#ifdef INFLATE
		static bool PointIsConcave(Liar::Uint);
#endif // INFLATE



#ifdef UNION_POLYGON
	private:
		static int IntersectPoint(Liar::Map&, Liar::Uint&, Liar::Uint&, bool, bool = true);
		static Liar::Uint LinkToPolygon(Liar::Map&, Liar::Uint, Liar::Uint, bool);
		static int GetNodeIndex(Liar::Delaunay::Node** cv, int, const Liar::Vector2f&);

		// expand
		static bool Expand(Liar::Uint, Liar::Uint);
		static void ExpandNodes(Liar::Uint, Liar::Uint);

	private:
		static Liar::Delaunay::Node** m_nodes1;
		static Liar::Uint m_numberNode1;
		static Liar::Delaunay::Node** m_nodes2;
		static Liar::Uint m_numberNode2;

		static bool CheckCross(Liar::NAVDTYPE r1[], Liar::NAVDTYPE r2[]);

	public:
		static Liar::Uint UnionPolygons(Liar::Map&, const Liar::Polygon&, const Liar::Polygon&, bool = true);
#endif // UNION_POLYGON
	};

	/**
	* Բ
	* @author liar
	*/
	class Delaunay::Circle
	{
	public:
		Circle();
		~Circle();

	public:
		void Init();
		void Set(const Liar::Vector2f&, Liar::NAVDTYPE);
		void Set(Liar::NAVDTYPE, Liar::NAVDTYPE, Liar::NAVDTYPE);

	public:
		Liar::Vector2f* center;
		Liar::NAVDTYPE r;
	};

	class Delaunay::Node :public Liar::MapSource
	{
	public:
		Node(Liar::Map const*);
		~Node();

	public:
		void Init(Liar::Map const*);
		void Set(Liar::Uint, bool isInters, bool main);

	public:
		Liar::Uint v;
		bool i;
		bool p;
		bool o;
		Node* other;
		bool isMain;
		Node* next;
	};
}

#endif  // !__DELAUNAY_H__