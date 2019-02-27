#include "Map.h"
#include "Delaunay.h"

namespace Liar
{
	Map::Map() :
		m_vertexs(nullptr), m_numberVertex(0),
		m_polygons(nullptr), m_numberPolygon(0),
		m_cells(nullptr), m_numberCell(0),
		m_minX(Liar::ZERO), m_minY(Liar::ZERO), m_maxX(Liar::ZERO), m_maxY(Liar::ZERO)
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
				free(m_vertexs[i]);
				m_vertexs[i] = nullptr;
			}
			free(m_vertexs);
			m_vertexs = nullptr;
		}

		DisposePolygons();

		if (m_cells)
		{
			for (Liar::Uint i = 0; i < m_numberCell; ++i)
			{
				m_cells[i]->~Cell();
				free(m_cells[i]);
				m_cells[i] = nullptr;
			}
			free(m_cells);
			m_cells = nullptr;
		}

	}

	void Map::Init()
	{
		m_vertexs = nullptr;
		m_numberVertex = 0;

		m_polygons = nullptr;
		m_numberPolygon = 0;

		m_cells = nullptr;
		m_numberCell = 0;

		m_minX = m_minY = FLT_MAX;
		m_maxX = m_maxY = FLT_MIN;
	}

	bool Map::CanWalk(Liar::NAVDTYPE x, Liar::NAVDTYPE y)
	{
		for (Liar::Uint k = 0; k < m_numberCell; ++k)
		{
			if (m_cells[k]->IsPointIn(x, y))
			{
				return true;
			}
		}
		return false;
	}

	bool Map::InMap(Liar::NAVDTYPE x, Liar::NAVDTYPE y)
	{
		if (x >= m_minX && x <= m_maxX && y >= m_minY && y <= m_maxY)
		{
			return true;
		}
		return false;
	}



	void Map::CalcBound(Liar::Int index, bool force)
	{
		if (m_minX == Liar::ZERO && m_minY == Liar::ZERO && m_maxX == Liar::ZERO && m_maxY == Liar::ZERO)
		{
			if (index < 0)
			{
				for (Liar::Uint i = 0; i < m_numberPolygon; ++i) CalcBound(i, force);
			}
			else
			{
				Polygon& polygon = *(m_polygons[index]);
				NAVDTYPE* rect = polygon.Rectangle();
				m_minX = m_minX < rect[0] ? m_minX : rect[0];
				m_maxX = m_maxX > rect[1] ? m_maxX : rect[1];
				m_minY = m_minY < rect[2] ? m_minY : rect[2];
				m_maxY = m_maxY < rect[3] ? m_maxY : rect[3];
			}
		}
	}

	Liar::Vector2f* Map::GetVertex(Liar::Uint index) const
	{
		if (index >= m_numberVertex) return nullptr;
		return m_vertexs[index];
	}

	Liar::Polygon* Map::GetPolygon(Liar::Uint index) const
	{
		if (index >= m_numberPolygon) return nullptr;
		return m_polygons[index];
	}

	Liar::Cell* Map::GetCell(Liar::Uint index) const
	{
		if (index >= m_numberCell) return nullptr;
		return m_cells[index];
	}

	Liar::Uint Map::AddPolygon(Liar::Vector2f* v, Liar::Uint size)
	{
		if (size <= 0) return m_numberPolygon;

		Liar::Polygon& polygon = AutoAddPolygon();

		for (Liar::Uint i = 0; i < size; ++i)
		{
			Liar::Uint addIndex = AddVertex(v[i]);
			polygon.AddPointIndex(addIndex);
		}
		return m_numberPolygon;
	}

	Liar::Polygon& Map::AutoAddPolygon()
	{
		m_numberPolygon++;
		size_t blockSize = sizeof(Liar::Polygon*)*m_numberPolygon;
		if (!m_polygons) m_polygons = (Liar::Polygon**)malloc(blockSize);
		else m_polygons = (Liar::Polygon**)realloc(m_polygons, blockSize);

		Liar::Polygon* polygon = (Liar::Polygon*)malloc(sizeof(Liar::Polygon));
		polygon->Set(this);
		m_polygons[m_numberPolygon - 1] = polygon;
		return *polygon;
	}

	void Map::DisposePolygon(Liar::Polygon* polygon)
	{
		if (polygon)
		{
			Liar::Uint i = 0;
			Liar::Int findIndex = -1;
			for (i = 0; i < m_numberPolygon; ++i)
			{
				if (m_polygons[i] == polygon)
				{
					m_polygons[i]->~Polygon();
					free(m_polygons[i]);
					m_polygons[i] = nullptr;
					findIndex = i;
				}
			}

			if (findIndex >= 0)
			{
				for (i = findIndex + 1; i < m_numberPolygon; ++i)
				{
					m_polygons[i - 1] = m_polygons[i];
				}
				--m_numberPolygon;
			}
			
		}
	}

	void Map::DisposePolygons()
	{
		if (m_polygons)
		{
			for (Liar::Uint i = 0; i < m_numberPolygon; ++i)
			{
				m_polygons[i]->~Polygon();
				free(m_polygons[i]);
				m_polygons[i] = nullptr;
			}
			free(m_polygons);
			m_polygons = nullptr;
		}
	}

	Liar::Uint Map::AddVertex(const Liar::Vector2f& source)
	{
		return AddVertex(source.GetX(), source.GetY());
	}

	Liar::Uint Map::AddVertex(Liar::NAVDTYPE x, Liar::NAVDTYPE y)
	{

#ifdef UNIQUE_POINT
		for (Liar::Uint i = 0; i < m_numberVertex; ++i)
		{
			if (m_vertexs[i]->Equals(x, y)) return i;
		}
#endif // UNIQUE_POINT

		m_numberVertex++;
		size_t blockSize = sizeof(Liar::Vector2f*)*m_numberVertex;
		if (!m_vertexs) m_vertexs = (Liar::Vector2f**)malloc(blockSize);
		else m_vertexs = (Liar::Vector2f**)realloc(m_vertexs, blockSize);
		Liar::Vector2f* copy = (Liar::Vector2f*)malloc(sizeof(Liar::Vector2f));
		copy->Set(x, y);
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

	void Map::AddNavMeshCell(Liar::Cell* cell)
	{
		cell->SetIndex(m_numberCell++);
		size_t blockSize = sizeof(Liar::Cell*)*m_numberCell;
		if (m_cells) m_cells = (Liar::Cell**)realloc(m_cells, blockSize);
		else m_cells = (Liar::Cell**)malloc(blockSize);
		m_cells[m_numberCell - 1] = cell;
	}

	Liar::Uint Map::NavMeshLinkCells(bool isCW)
	{
		if (!isCW)
		{
			Liar::Uint i = 0;
			Liar::Uint j = m_numberCell - 1;
			for (i = 0, j = m_numberCell - 1; i <= m_numberCell / 2 - 1; ++i, --j)
			{
				Liar::Cell* tmpCell = m_cells[i];
				m_cells[j]->SetIndex(i);
				tmpCell->SetIndex(j);
				m_cells[i] = m_cells[j];
				m_cells[j] = tmpCell;
			}
		}

		for (Liar::Uint i = 0; i < m_numberCell; ++i)
		{
			for (Liar::Uint j = 0; j < m_numberCell; ++j)
			{
				if (i != j) m_cells[i]->CheckAndLink(*(m_cells[j]));
			}
		}

#ifndef EditorMod
		DisposePolygons();
#endif // !EditorMap

		return m_numberCell;
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
					Liar::Uint unionNum = 0;//Liar::Delaunay::UnionPolygons(*this, *p0, *p1, rw);
					if (unionNum > 0)
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

						m_numberPolygon -= 2 + unionNum;

						n = 1;
					}
				}
			}
		}
	}
#endif // UNION_POLYGON

#if defined(DEBUG_NIF) || defined(EditorMod)
	void Map::WriteErlang(std::ofstream& outfile)
	{
		for (Liar::Uint j = 0; j < m_numberPolygon; ++j)
		{
			outfile << "\n[";
			Liar::Polygon& polygon = *(m_polygons[j]);
			Liar::Uint vector2fSize = polygon.GetNumPoints();
			for (Liar::Uint k = 0; k < vector2fSize; ++k)
			{
				Liar::Uint pointIndex = polygon.GetPointIndex(k);
				Liar::Vector2f* v = polygon.GetVertex(pointIndex);
				if (k == vector2fSize - 1) outfile << "{" << v->GetX() << "," << v->GetY() << "}";
				else outfile << "{" << v->GetX() << "," << v->GetY() << "},";
			}
			if (j == m_numberPolygon - 1) outfile << "]";
			else outfile << "],";
		}

		for (Liar::Uint i = 0; i < m_numberCell; ++i)
		{
			outfile << "\n";
			m_cells[i]->WriteErlang(outfile);
		}
	}
#endif
}
