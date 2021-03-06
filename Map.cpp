#include "Map.h"
#include "Delaunay.h"
#include <float.h>

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

	bool Map::CanWalk(Liar::NAVDTYPE x, Liar::NAVDTYPE y, bool CW)
	{
		Liar::Cell* cell = GetCell(x, y, CW);
		return cell != nullptr;
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
		if (m_minX == FLT_MAX && m_minY == FLT_MAX && m_maxX == FLT_MIN && m_maxY == FLT_MIN)
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
				m_maxY = m_maxY > rect[3] ? m_maxY : rect[3];
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

	Liar::Cell* Map::GetCell(Liar::NAVDTYPE x, Liar::NAVDTYPE y, bool CW) const
	{
		for (Liar::Uint k = 0; k < m_numberCell; ++k)
		{
			if (m_cells[k]->IsPointIn(x, y, CW))
			{
				return m_cells[k];
			}
		}
		return nullptr;
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

	Liar::Uint Map::AddNavMeshCell(Liar::Cell* cell)
	{
		cell->SetIndex(m_numberCell++);
		size_t blockSize = sizeof(Liar::Cell*)*m_numberCell;
		if (m_cells) m_cells = (Liar::Cell**)realloc(m_cells, blockSize);
		else m_cells = (Liar::Cell**)malloc(blockSize);
		m_cells[m_numberCell - 1] = cell;
		return m_numberCell;
	}

	Liar::Uint Map::NavMeshLinkCells(bool isCW)
	{
		if (!isCW && m_numberCell > 0)
		{
			Liar::Uint i = 0;
			Liar::Uint j = m_numberCell - 1;
			Liar::NAVDTYPE tmpMax = m_numberCell * 0.5 - 1;
			Liar::Uint max = static_cast<Liar::Uint>(tmpMax);
			for (i = 0, j = m_numberCell - 1; i <= max; ++i, --j)
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

#ifdef FILTER_POLYGON
	void Map::RemoveRedundant()
	{
		for (Liar::Uint i = 0; i < m_numberPolygon; ++i)
		{
			m_polygons[i]->RemoveRedundant();
		}
	}
#endif // !FILTER_POLYGON

#ifdef UNION_POLYGON
	void Map::UnionAll(bool rw)
	{
		for (Liar::Uint n = 1; n < m_numberPolygon; ++n)
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
					Liar::Uint unionNum = Liar::Delaunay::UnionPolygons(*this, *p0, *p1, rw);
					if (unionNum > 0)
					{
						DisposePolygon(p0);
						DisposePolygon(p1);
						UnionAll(rw);
					}
				}
			}
		}
	}
#endif // UNION_POLYGON

#if defined(DEBUG_NIF) || defined(EditorMod)
	void Map::WriteErlang(std::ofstream& outfile, bool writeAll)
	{
		if (writeAll) 
		{
			for (Liar::Uint i = 0; i < m_numberVertex; ++i)
			{
				Liar::Vector2f* v = m_vertexs[i];
				outfile << "i:" << i << " {" << v->GetX() << "," << v->GetY() << "}\n";
			}
		}

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

		if (writeAll) 
		{
			for (Liar::Uint i = 0; i < m_numberCell; ++i)
			{
				outfile << "\n";
				m_cells[i]->WriteErlang(outfile);
			}
		}
	}
#endif
}
