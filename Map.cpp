#include "Map.h"

namespace Liar
{
#ifdef UNION_POLYGON
	Map::Map() :
		m_vertexs(nullptr), m_numberVertex(0),
		m_polygons(nullptr), m_numberPolygon(0),
		m_nodes1(nullptr), m_numberNode1(0),
		m_nodes2(nullptr), m_numberNode2(0),
		m_navMesh(nullptr)
#else
	Map::Map() :
		m_vertexs(nullptr), m_numberVertex(0),
		m_polygons(nullptr), m_numberPolygon(0)
#endif // UNION_POLYGON
	{
	}


	Map::~Map()
	{
		Liar::Uint i = 0;
		if (m_vertexs)
		{
			for (i = 0; i < m_numberVertex; ++i)
			{
				m_vertexs[i]->~Vector2f();
				m_vertexs[i] = nullptr;
			}
			free(m_vertexs);
			m_vertexs = nullptr;
		}

		if (m_polygons)
		{
			for (i = 0; i < m_numberPolygon; ++i)
			{
				m_polygons[i]->~Polygon();
				m_polygons[i] = nullptr;
			}
			free(m_polygons);
			m_polygons = nullptr;
		}

		if (m_navMesh)
		{
			m_navMesh->~NavMesh();
			free(m_navMesh);
			m_navMesh = nullptr;
		}

#ifdef UNION_POLYGON
		DisposeNodes();
#endif // UNION_POLYGON

	}

	void Map::Init()
	{
		m_vertexs = nullptr;
		m_numberVertex = 0;

		m_polygons = nullptr;
		m_numberPolygon = 0;
	}

	Liar::Vector2f* Map::GetVertex(Liar::Uint index) const
	{
		if (index >= m_numberVertex) return nullptr;
		return m_vertexs[index];
	}

	const Liar::Polygon* Map::GetPolygon(Liar::Uint index) const
	{
		return GetPolygon(index);
	}

	Liar::Polygon* Map::GetPolygon(Liar::Uint index)
	{
		if (index >= m_numberPolygon) return nullptr;
		return m_polygons[index];
	}

	Liar::Uint Map::AddPolygon(Liar::Vector2f* v, Liar::Uint size)
	{
		if (size <= 0) return m_numberPolygon;

		m_numberPolygon++;
		size_t len = sizeof(Liar::Polygon*)*m_numberPolygon;
		if (!m_polygons) m_polygons = (Liar::Polygon**)malloc(len);
		else m_polygons = (Liar::Polygon**)realloc(m_polygons, len);

		Liar::Polygon* polygon = (Liar::Polygon*)malloc(sizeof(Liar::Polygon));
		polygon->Set(this);

		for (Liar::Uint i = 0; i < size; ++i)
		{
			Liar::Uint addIndex = AddVertex(v[i]);
			polygon->AddPointIndex(addIndex);
		}
	}

	Liar::Uint Map::AddVertex(const Liar::Vector2f& source)
	{
		for (Liar::Uint i = 0; i < m_numberVertex; ++i)
		{
			if (source.Equals(*m_vertexs[i])) return i;
		}

		m_numberVertex++;
		size_t size = sizeof(Liar::Vector2f*)*m_numberVertex;
		if (!m_vertexs) m_vertexs = (Liar::Vector2f**)malloc(size);
		else m_vertexs = (Liar::Vector2f**)realloc(m_vertexs, size);
		Liar::Vector2f* copy = (Liar::Vector2f*)malloc(sizeof(Liar::Vector2f));
		copy->Set(source);
		m_vertexs[m_numberVertex - 1] = copy;
		return m_numberVertex - 1;
	}

	void Map::CalcBound(const Liar::Vector2f& point)
	{
		Liar::NAVDTYPE x = point.GetX();
		Liar::NAVDTYPE y = point.GetY();

		m_minX = x < m_minX ? x : m_minX;
		m_maxX = x > m_maxX ? x : m_maxX;
		m_minY = y < m_minY ? y : m_minY;
		m_maxY = y > m_maxY ? y : m_maxX;
	}

	void Map::CalcBound(const Liar::Polygon& polygon)
	{
		Liar::NAVDTYPE* rect = polygon.GetRect();
		if (rect)
		{
			Liar::NAVDTYPE minx = rect[0];
			Liar::NAVDTYPE maxx = rect[1];
			Liar::NAVDTYPE miny = rect[2];
			Liar::NAVDTYPE maxy = rect[3];

			m_minX = minx < m_minX ? minx : m_minX;
			m_maxX = maxx > m_maxX ? maxx : m_maxX;
			m_minY = miny < m_minY ? miny : m_minY;
			m_maxY = maxy > m_maxY ? maxy : m_maxY;
		}
	}

	void Map::AddNavMeshCell(const Liar::Cell* cell)
	{
		if (!m_navMesh)
		{
			m_navMesh = (Liar::NavMesh*)malloc(sizeof(Liar::NavMesh));
			m_navMesh->Set();
		}
	}

	void Map::NavMeshLinkCells(bool isCW)
	{
		if (m_navMesh) m_navMesh->LinkCells(isCW);
	}

#ifdef UNION_POLYGON
	void Map::UnionAll(bool rw)
	{
		for (Liar::Uint n = 0; n < m_numberPolygon; ++n)
		{
			Liar::Polygon* p0 = m_polygons[n];
			for (Liar::Uint m = 1; m < m_numberPolygon; ++m)
			{
				Liar::Polygon* p1 = m_polygons[m];
				p0->AutoCW(rw);
				p1->AutoCW(rw);
				if (n != m)
				{
					p0->Rectangle();
					p1->Rectangle();
					if (UnionPolygons(*p0, *p1, rw) > 0)
					{
						// delete p0 & p1;
						Liar::Uint replace0 = 0;
						Liar::Uint replace1 = 0;
						if (m > n)
						{
							replace0 = n;
							replace1 = m;
						}
						else
						{
							replace0 = m;
							replace1 = n;
						}

						// TODO delete null refrence point index
						// at least 1
						m_polygons[replace0]->~Polygon();
						m_polygons[replace0] = nullptr;
						m_polygons[replace0] = m_polygons[m_numberPolygon - 1];

						// last move
						free(m_polygons[m_numberPolygon - 1]);
						m_polygons[m_numberPolygon - 1] = nullptr;

						m_polygons[replace1]->~Polygon();
						if (replace1 != m_numberPolygon - 2)
						{
							m_polygons[replace1] = m_polygons[m_numberPolygon - 2];
							free(m_polygons[m_numberPolygon - 2]);
							m_polygons[m_numberPolygon - 2] = nullptr;
						}

						m_numberPolygon -= 2;


						n = 1;
					}
				}
			}
		}
	}

	Liar::Uint Map::UnionPolygons(const Liar::Polygon& p0, const Liar::Polygon& p, bool rw)
	{
		Liar::NAVDTYPE* sr = p0.GetRect();
		Liar::NAVDTYPE* pr = p.GetRect();

		Liar::Uint unionNum = 0;

		if (CheckCross(sr, pr))
		{
			Liar::Uint num = p0.GetNumPoints();
			Liar::Uint pnum = p.GetNumPoints();

			Liar::Map::Node** cv0 = nullptr;
			Liar::Map::Node** cv1 = nullptr;

			Liar::Map::CreateNodes(this, num, pnum, cv0, cv1);

			Liar::Uint i = 0;
			for (i = 0; i < num; ++i)
			{
				cv0[i]->Set(i, false, true);
				if (i > 0) cv0[i - 1]->next = cv0[i];
			}

			for (i = 0; i < pnum; ++i)
			{
				cv1[i]->Set(i, false, false);
				if (i > 0) cv1[i - 1]->next = cv1[i];
			}

			if (IntersectPoint(cv0, cv1, num, pnum, rw) > 0)
			{
				unionNum = LinkToPolygon(cv0, num, cv1, pnum);
			}
		}

		return unionNum;
	}

	int Map::IntersectPoint(Liar::Map::Node** cv0, Liar::Map::Node** cv1, Liar::Uint& cv0c, Liar::Uint& cv1c, bool rw)
	{
		Liar::Int insCnt = 0;
		bool hasIns = false;
		Liar::Map::Node* startNode0 = cv0[0];
		Liar::Map::Node* startNode1 = nullptr;
		Liar::Map::Node* test = nullptr;

		Liar::Vector2f* ins = (Liar::Vector2f*)malloc(sizeof(Liar::Vector2f));
		ins->Set(0.0f, 0.0f);

		size_t size = sizeof(Liar::Line2d);
		Liar::Line2d* line0 = (Liar::Line2d*)malloc(size);
		Liar::Line2d* line1 = (Liar::Line2d*)malloc(size);
		line0->Set(this);
		line1->Set(this);

		Liar::Int prec0 = 0;
		Liar::Int prec1 = 0;

		while (startNode0)
		{
			test = startNode0->next ? startNode0->next : cv0[0];
			line0->Set(startNode0->v, test->v);

			startNode1 = cv1[0];
			hasIns = false;

			while (startNode1)
			{
				test = startNode1->next ? startNode1->next : cv1[0];
				line1->Set(startNode1->v, test->v);

				ins->Set(0.0f, 0.0f);

				if (line0->Intersection(*line1, ins) == Liar::LineClassification::SEGMENTS_INTERSECT)
				{
					if (GetNodeIndex(cv0, cv0c, *ins) == -1)
					{
						++insCnt;

						prec0 = cv0c;
						prec1 = cv1c;
						Liar::Map::CreateNodes(this, ++cv0c, ++cv1c, cv0, cv1, false);
						Liar::Uint addPointIndex = AddVertex(*ins);

						Liar::Map::Node* Node0 = cv0[prec0];
						Liar::Map::Node* Node1 = cv1[prec1];

						Node0->Set(addPointIndex, true, true);
						Node1->Set(addPointIndex, true, false);

						Node0->other = Node1;
						Node1->other = Node0;


						Node0->next = startNode0->next;
						startNode0->next = Node0;
						Node1->next = startNode1->next;
						startNode1->next = Node1;

						Liar::Vector2f& line1PointB = line1->GetPointB();
						if (line0->ClassifyPoint(line1PointB, rw) == Liar::PointClassification::RIGHT_SIDE)
						{
							Node0->o = true;
							Node1->o = true;
						}

						hasIns = true;
						break;
					}
				}

				startNode1 = startNode1->next;
			}

			if (hasIns == false)
			{
				startNode0 = startNode0->next;
			}
		}

		ins->~Vector2f();
		free(ins);
		ins = nullptr;

		line0->~Line2d();
		free(line0);
		line0 = nullptr;

		line1->~Line2d();
		free(line1);
		line1 = nullptr;

		return insCnt;
	}

	Liar::Uint Map::LinkToPolygon(Liar::Map::Node** cv0, Liar::Uint cv0c, Liar::Map::Node** cv1, Liar::Uint cv1c)
	{
		Liar::Uint linkNum = 0;

		for (int i = 0; i < cv0c; ++i)
		{
			Liar::Map::Node* testNode = cv0[i];
			if (testNode->i == true && testNode->p == false)
			{
				Vector2f* rcNodes = nullptr;
				int rcNum = 0;
				int test = 0;
				while (testNode)
				{
					testNode->p = true;

					if (testNode->i == true)
					{
						testNode->other->p = true;

						if (testNode->o == false)
						{
							if (testNode->isMain == true) testNode = testNode->other;
						}
						else
						{
							if (testNode->isMain == false) testNode = testNode->other;
						}
					}

					++rcNum;
					if (rcNodes) rcNodes = (Vector2f*)realloc(rcNodes, sizeof(Vector2f)*rcNum);
					else rcNodes = (Vector2f*)malloc(sizeof(Vector2f)*rcNum);

					Liar::Vector2f* testNodePoint = testNode->GetVertex(testNode->v);
					rcNodes[rcNum - 1].Set(*testNodePoint);
					++test;
					if (!testNode->next) testNode = testNode->isMain ? cv0[0] : cv1[0];
					else testNode = testNode->next;

					if (testNodePoint->Equals(rcNodes[0], EPSILON)) break;
				}

				if (rcNum > 0)
				{
					++linkNum;
					AddPolygon(rcNodes, rcNum);
				}

			}
		}
		return linkNum;
	}

	int Map::GetNodeIndex(Liar::Map::Node** cv, int ncount, const Liar::Vector2f& Node)
	{
		for (int i = 0; i < ncount; ++i)
		{
			Liar::Map::Node& node = *(cv[i]);
			Liar::Vector2f* vec = node.GetVertex(node.v);
			if (vec->Equals(Node)) return i;
		}
		return -1;
	}

	void Map::CreateNodes(Liar::Map const* map,
		Liar::Uint num1, Liar::Uint num2,
		Liar::Map::Node** nodes1, Liar::Map::Node** nodes2,
		bool autoExpand)
	{
		if (autoExpand)
		{
			Liar::Uint min = 0;
			Liar::Uint max = 0;
			Liar::Map::Node** setMinNodes = nullptr;
			Liar::Map::Node** setMaxNodes = nullptr;

			if (num1 > num2)
			{
				min = num2;
				max = num1;

				setMinNodes = nodes2;
				setMaxNodes = nodes1;
			}
			else
			{
				min = num1;
				max = num2;
				setMinNodes = nodes1;
				setMaxNodes = nodes2;

			}

			Liar::Uint curMin = 0;
			Liar::Uint curMax = 0;
			Liar::Map::Node** curMinNodes = nullptr;
			Liar::Map::Node** curMaxNodes = nullptr;

			if (Liar::Map::m_numberNode1 > Liar::Map::m_numberNode2)
			{
				curMin = Liar::Map::m_numberNode2;
				curMax = Liar::Map::m_numberNode1;

				curMinNodes = Liar::Map::m_nodes2;
				curMaxNodes = Liar::Map::m_nodes1;
			}
			else
			{
				curMin = Liar::Map::m_numberNode1;
				curMax = Liar::Map::m_numberNode2;

				curMinNodes = Liar::Map::m_nodes1;
				curMaxNodes = Liar::Map::m_nodes2;
			}

			if (curMin < min && curMax >= max)
			{
				ExpandNodes(map, min, curMinNodes, curMin);
			}
			else if (curMin >= min && curMax < max)
			{
				ExpandNodes(map, min, curMaxNodes, curMin);
			}

			setMinNodes = curMinNodes;
			setMaxNodes = curMaxNodes;
		}
		else
		{
			if (nodes1 == Liar::Map::m_nodes1 && nodes2 == Liar::Map::m_nodes2)
			{
				if (Liar::Map::m_numberNode1 < num1) ExpandNodes(map, num1, nodes1, Liar::Map::m_numberNode1);
				if (Liar::Map::m_numberNode2 < num2) ExpandNodes(map, num2, nodes2, Liar::Map::m_numberNode2);
			}
			else if (nodes1 == Liar::Map::m_nodes2 && nodes2 == Liar::Map::m_nodes1)
			{
				if (Liar::Map::m_numberNode2 < num1) ExpandNodes(map, num1, nodes1, Liar::Map::m_numberNode2);
				if (Liar::Map::m_numberNode1 < num2) ExpandNodes(map, num1, nodes2, Liar::Map::m_numberNode1);
			}
			else
			{
				CreateNodes(map, num1, num2, nodes1, nodes2, true);
			}
		}
	}

	void Map::ExpandNodes(Liar::Map const* map, Liar::Uint curNum, Liar::Map::Node** nodes, Liar::Uint& oldNum)
	{
		if (curNum > oldNum)
		{
			size_t size = sizeof(Liar::Map::Node*)*curNum;
			if (nodes) nodes = (Liar::Map::Node**)realloc(nodes, size);
			else nodes = (Liar::Map::Node**)malloc(size);

			size = sizeof(Liar::Map::Node);
			for (Liar::Uint i = oldNum; i < curNum; ++i)
			{
				Liar::Map::Node* node = (Liar::Map::Node*)malloc(size);
				nodes[i] = node;
			}

			oldNum = curNum;
		}
	}

	void Map::DisposeNodes()
	{
		Liar::Uint i = 0;
		if (m_nodes1)
		{
			for (i = 0; i < m_numberNode1; ++i)
			{
				m_nodes1[i]->~Node();
				m_nodes1[i] = nullptr;
			}
			free(m_nodes1);
			m_nodes1 = nullptr;
			m_numberNode1 = 0;
		}

		if (m_nodes2)
		{
			for (i = 0; i < m_numberNode2; ++i)
			{
				m_nodes2[i]->~Node();
				m_nodes2[i] = nullptr;
			}

			free(m_nodes2);
			m_nodes2 = nullptr;
			m_numberNode2 = 0;
		}
	}

	bool Map::CheckCross(Liar::NAVDTYPE r1[], Liar::NAVDTYPE r2[])
	{
		NAVDTYPE minx1 = r1[0];
		NAVDTYPE maxx1 = r1[1];
		NAVDTYPE miny1 = r1[2];
		NAVDTYPE maxy1 = r1[3];

		NAVDTYPE minx2 = r2[0];
		NAVDTYPE maxx2 = r2[1];
		NAVDTYPE miny2 = r2[2];
		NAVDTYPE maxy2 = r2[3];

		NAVDTYPE minx = minx1 > minx2 ? minx1 : minx2;
		NAVDTYPE miny = miny1 > miny2 ? miny1 : miny2;
		NAVDTYPE maxx = maxx1 > maxx2 ? maxx2 : maxx1;
		NAVDTYPE maxy = maxy1 > maxy2 ? maxy2 : maxy1;

		if (minx > maxx || miny > maxy)
		{
			return false;
		}
		else
		{
			return true;
		}
	}
#endif // UNION_POLYGON

	// ============================== Node =========================
	Map::Node::Node(Liar::Map const* map) :
		Liar::MapSource(map),
		v(0), i(false), p(false), o(false), isMain(false),
		other(nullptr), next(nullptr)
	{}

	Map::Node::~Node()
	{
		m_map = nullptr;
		other = nullptr;
		next = nullptr;
	}

	void Map::Node::Init(Liar::Map const* map)
	{
		Liar::MapSource::Set(map);

		v = 0;
		i = p = o = isMain = false;
		other = next = nullptr;
	}

	void Map::Node::Set(Liar::Uint v0, bool isInters, bool main)
	{
		v = v0;
		i = isInters;
		isMain = main;
	}
	// ============================== Node =========================
}
