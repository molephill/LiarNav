
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

	public:
		Delaunay();
		~Delaunay();

	private:
		Liar::Line2d** m_line2ds;
		Liar::Uint m_totalLines;
		Liar::Uint m_curNumLines;

	public:
		void Init();
		Liar::Uint Set(Liar::Map&, bool = true, Liar::Uint = 0);

		static Liar::LineClassification Intersection(const Liar::Line2d&, const Liar::Line2d&, Liar::Vector2f*);
		static Liar::LineClassification Intersection(const Liar::Vector2f&, const Liar::Vector2f&, const Liar::Line2d&, Liar::Vector2f*);
		static Liar::LineClassification Intersection(const Liar::Vector2f&, const Liar::Vector2f&, const Liar::Vector2f&, const Liar::Vector2f&, Liar::Vector2f*);

	private:
		void BuildEdges(Liar::Map&);
		void BuildTrianges(Liar::Map&, bool = true, Liar::Uint = 0);
		Liar::Line2d** BuildTrianges(const Liar::Vector2f&, const Liar::Vector2f&, Liar::Line2d**, Liar::Int&, Liar::Map&, Liar::Uint, Liar::Uint);
		Liar::Line2d& GetBoundEdage(Liar::Map&, bool = true, Liar::Uint = 0);

		Liar::Int FindDT(Liar::Map&, const Liar::Line2d&, Liar::Vector2f&, bool = true);

		bool IsVisiblePointOfLine(const Vector2f&, const Line2d&, Liar::Vector2f&, bool = true);
		bool IsVisibleIn2Point(const Liar::Vector2f&, const Liar::Vector2f&, Liar::Vector2f&);

		void CircumCircle(const Liar::Vector2f&, const Liar::Vector2f&, const Liar::Vector2f&, Liar::Delaunay::Circle& cir);
		void CircleBounds(const Liar::Delaunay::Circle&, Liar::NAVDTYPE*);

		Liar::NAVDTYPE LineAngle(const Liar::Vector2f&, const Liar::Vector2f&, const Liar::Vector2f&);

		Liar::Line2d** RemovePosLine(Liar::Line2d** lv, int&, int pos);
		Liar::Int FindLinePos(const Liar::Vector2f&, const Liar::Vector2f&, Liar::Line2d**, Liar::Uint);

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
		static void DisposeNodes();
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