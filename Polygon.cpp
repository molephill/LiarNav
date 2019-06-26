#include "Polygon.h"
#include "Map.h"
#include "Line2d.h"
#include "Delaunay.h"

namespace Liar
{
	Polygon::Polygon(const Liar::Map* map) :
		Liar::MapSource(map),
		m_rect(nullptr), m_isClockWise(Liar::ClockWiseDefine::CLOCK_NO_DEFINE),
		m_pointIndices(nullptr), m_numberPoints(0)
	{
	}


	Polygon::~Polygon()
	{
		Dispose();
	}

	void Polygon::Dispose()
	{
		Liar::MapSource::Dispose();

		if (m_rect)
		{
			free(m_rect);
			m_rect = nullptr;
		}

		if (m_pointIndices)
		{
			free(m_pointIndices);
			m_pointIndices = nullptr;
		}
		m_numberPoints = 0;
	}

	void Polygon::Set(Liar::Map const* map)
	{
		Liar::MapSource::Set(map);
		m_rect = nullptr;
		m_pointIndices = nullptr;
		m_numberPoints = 0;
	}

	void Polygon::Set(const Liar::Polygon& source)
	{
		Liar::MapSource::Set(source);
		if (source.m_rect)
		{
			size_t size = sizeof(Liar::NAVDTYPE) * 4;
			m_rect = (Liar::NAVDTYPE*)malloc(size);
			//memcpy(m_rect, source.m_rect, size);
			for (size_t i = 0; i < 4; ++i) m_rect[i] = source.m_rect[i];
		}

		if (m_pointIndices)
		{
			free(m_pointIndices);
			m_pointIndices = nullptr;
		}
		m_numberPoints = source.m_numberPoints;
		if (m_numberPoints > 0)
		{
			size_t size = sizeof(Liar::Uint)*m_numberPoints;
			m_pointIndices = (Liar::Uint*)malloc(size);
			//memcpy(m_pointIndices, source.m_pointIndices, size);
			for (size_t i = 0; i < m_numberPoints; ++i) m_rect[i] = source.m_pointIndices[i];
		}
	}

	void Polygon::AddPointIndex(Liar::Uint index)
	{
		m_numberPoints++;
		size_t size = sizeof(Liar::Uint)*m_numberPoints;
		if (!m_pointIndices) m_pointIndices = (Liar::Uint*)malloc(size);
		else m_pointIndices = (Liar::Uint*)realloc(m_pointIndices, size);
		m_pointIndices[m_numberPoints - 1] = index;
	}

	Liar::Uint Polygon::GetPointIndex(Liar::Uint index) const
	{
		if (index >= m_numberPoints) return m_numberPoints;
		return m_pointIndices[index];
	}

	/**
	* auto clockwise
	* @return true -- clockwise; false -- counter-clockwise
	*/
	void Polygon::AutoCW(bool isCW)
	{
		if (isCW == IsCW()) return;
		Liar::Uint i, j, k;
		for (i = 0, j = m_numberPoints - 1; i <= m_numberPoints / 2 - 1; i++, j--)
		{
			k = m_pointIndices[i];
			m_pointIndices[i] = m_pointIndices[j];
			m_pointIndices[j] = k;
		}
		m_isClockWise = isCW ? Liar::ClockWiseDefine::CLOCK_WISE_DEFINE : Liar::ClockWiseDefine::COUNT_CLOCK_WISE_DEFINE;
	}

	/**
	* clockwise
	* @return true -- clockwise; false -- counter-clockwise
	*/
	bool Polygon::IsCW()
	{
		if (m_numberPoints < 3) return false;
		if (m_isClockWise != Liar::ClockWiseDefine::CLOCK_NO_DEFINE) return m_isClockWise == Liar::ClockWiseDefine::CLOCK_WISE_DEFINE;

		Liar::Vector2f* topPt = GetVertex(0);
		Liar::Uint topPtId = 0;

		for (Liar::Uint i = 1; i < m_numberPoints; ++i)
		{
			Liar::Vector2f* tmp = GetVertex(i);
			Liar::NAVDTYPE topY = topPt->GetY();
			Liar::NAVDTYPE tmpY = tmp->GetY();
			if (topY > tmpY)
			{
				topPt = tmp;
				topPtId = i;
			}
			else if (topY == tmpY)
			{
				if (topPt->GetX() == tmp->GetX())
				{
					topPt = tmp;
					topPtId = i;
				}
			}
		}

		Liar::Uint lastId = 0, nextId = 0;
		lastId = topPtId >= 1 ? topPtId - 1 : m_numberPoints - 1;
		nextId = (topPtId + 1) >= m_numberPoints ? 0 : topPtId + 1;

		Liar::Vector2f* lastVertex = GetVertex(lastId);
		Liar::Vector2f* nextVertex = GetVertex(nextId);

		Liar::NAVDTYPE r = Multiply(*lastVertex, *nextVertex, *topPt);
		m_isClockWise = r < 0 ? Liar::ClockWiseDefine::CLOCK_WISE_DEFINE : Liar::ClockWiseDefine::COUNT_CLOCK_WISE_DEFINE;
		return r < 0;
	}

	NAVDTYPE Polygon::Multiply(const Vector2f& sp, const Vector2f& ep, const Vector2f& op)
	{
		return (sp.GetX() - op.GetX())*(ep.GetY() - op.GetY()) - (ep.GetX() - op.GetX())*(sp.GetY() - op.GetY());
	}

	Liar::NAVDTYPE* Polygon::Rectangle(bool forceCalce)
	{
		if (m_numberPoints <= 0 || (forceCalce == false && m_rect)) return m_rect;
		if (!m_rect) m_rect = (Liar::NAVDTYPE*)malloc(sizeof(Liar::NAVDTYPE) * 4);

		Liar::Vector2f* tmp = GetVertex(0);
		Liar::NAVDTYPE lx = tmp->GetX();
		Liar::NAVDTYPE rx = lx;
		Liar::NAVDTYPE ty = tmp->GetY();
		Liar::NAVDTYPE by = ty;

		for (Liar::Uint i = 1; i < m_numberPoints; ++i)
		{
			tmp = GetVertex(i);
			Liar::NAVDTYPE curX = tmp->GetX();
			Liar::NAVDTYPE curY = tmp->GetY();

			lx = curX < lx ? curX : lx;
			rx = curX > rx ? curX : rx;
			ty = curY < ty ? curY : ty;
			by = curY > by ? curY : by;
		}

		m_rect[0] = lx;
		m_rect[1] = rx;
		m_rect[2] = ty;
		m_rect[3] = by;
		return m_rect;
	}

	void Polygon::DisposeVector(Liar::Uint findIndex)
	{
		if (findIndex >= 0 && findIndex < m_numberPoints)
		{
			for (Liar::Uint i = findIndex + 1; i < m_numberPoints; ++i)
			{
				m_pointIndices[i - 1] = m_pointIndices[i];
			}
			--m_numberPoints;
		}
	}

#ifdef FILTER_POLYGON
	void Polygon::RemoveRedundant()
	{
		for (Liar::Uint i = 0; i < m_numberPoints - 2; ++i) 
		{
			Liar::Vector2f* first = GetVertex(i);
			Liar::Vector2f* second = GetVertex(i+1);
			Liar::Vector2f* last = GetVertex(i + 2);

			if (Liar::Delaunay::OnLine(*first, *second, *last))
			{
				DisposeVector(i + 1);
				i--;
			}
		}
	}
#endif // FILTER_POLYGON

}
