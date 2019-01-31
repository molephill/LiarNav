#include "Polygon.h"
#include "Map.h"
#include "Line2d.h"

namespace Liar
{
	Polygon::Polygon(const Liar::Map* map):
		Liar::MapSource(map),
		m_rect(nullptr), m_isClockWise(Liar::ClockWiseDefine::CLOCK_NO_DEFINE)
	{
	}


	Polygon::~Polygon()
	{
		Liar::MapSource::~MapSource();

		if (m_rect)
		{
			free(m_rect);
			m_rect = nullptr;
		}
	}

	void Polygon::Set(Liar::Map const* map)
	{
		Liar::MapSource::Set(map);
		m_rect = nullptr;
	}

	void Polygon::Set(const Liar::Polygon& source)
	{
		Liar::MapSource::Set(source);
		if (source.m_rect)
		{
			size_t size = sizeof(Liar::NAVDTYPE) * 4;
			m_rect = (Liar::NAVDTYPE*)malloc(size);
			memcpy(m_rect, source.m_rect, size);
		}
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
		int topPtId = 0;

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

		int lastId = 0, nextId = 0;
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
			rx = curX > lx ? curX : lx;
			ty = curY < ty ? curY : ty;
			by = curY > by ? curY : by;
		}

		m_rect[0] = lx;
		m_rect[1] = rx;
		m_rect[2] = ty;
		m_rect[3] = by;
		return m_rect;
	}

}
