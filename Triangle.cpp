#include "Triangle.h"
#include "Map.h"

namespace Liar
{
	Triangle::Triangle(Liar::Map const* map):
		Liar::MapSource(map),
		m_pointIndexA(0), m_pointIndexB(0), m_pointIndexC(0),
		m_center(nullptr), m_sides(nullptr),
		m_centerCalculated(false)
	{
	}


	Triangle::~Triangle()
	{
		Liar::MapSource::~MapSource();

		m_center->~Vector2f();
		free(m_center);
		m_center = nullptr;

		if (m_sides)
		{
			for (size_t i = 0; i < 3; i++)
			{
				m_sides[i]->~Line2d();
				free(m_sides[i]);
				m_sides[i] = nullptr;
			}

			free(m_sides);
			m_sides = nullptr;
		}
	}

	void Triangle::Set(Liar::Map const* map)
	{
		Liar::MapSource::Set(map);
		m_pointIndexA = 0;
		m_pointIndexB = 0;
		m_pointIndexC = 0;

		m_center = (Liar::Vector2f*)malloc(sizeof(Liar::Vector2f));
		m_sides = nullptr;
		m_centerCalculated = false;
	}

	bool Triangle::Set(Liar::Map const* map, Liar::Uint a, Liar::Uint b, Liar::Uint c)
	{
		Liar::MapSource::Set(map);
		return Set(a, b, c);
	}

	bool Triangle::Set(Liar::Uint a, Liar::Uint b, Liar::Uint c)
	{
		if (a != m_pointIndexA || b != m_pointIndexB || c != m_pointIndexC)
		{
			m_pointIndexA = a;
			m_pointIndexB = b;
			m_pointIndexC = c;
			m_centerCalculated = false;
			CalcInit();
			return true;
		}
		return false;
	}

	void Triangle::Set(const Liar::Triangle& source)
	{
		Liar::MapSource::Set(source);
		m_pointIndexA = source.m_pointIndexA;
		m_pointIndexB = source.m_pointIndexB;
		m_pointIndexC = source.m_pointIndexC;
		m_centerCalculated = source.m_centerCalculated;

		m_center = (Liar::Vector2f*)malloc(sizeof(Liar::Vector2f));
		m_center->Set(*(source.m_center));

		if (source.m_sides)
		{
			size_t size = sizeof(Liar::Line2d*) * 3;
			if(!m_sides) m_sides = (Liar::Line2d**)malloc(size);
			memcpy(m_sides, *(source.m_sides), size);
			//for (size_t i = 0; i < 3; ++i) m_sides[i]->Set(*(source.m_sides[i]));
		}
	}

	bool Triangle::IsPointIn(const Vector2f& testPoint, bool rw)
	{
		return IsPointIn(testPoint.GetX(), testPoint.GetY(), rw);
	}

	bool Triangle::IsPointIn(NAVDTYPE x, NAVDTYPE y, bool rw)
	{
		CalcInit();
		int interiorCount = 0;
		for (int i = 0; i < 3; ++i)
		{
			if (m_sides[i]->ClassifyPoint(x, y, rw) != PointClassification::LEFT_SIDE) ++interiorCount;
		}

		return interiorCount == 3;
	}

	// 判断是否是三角形的边
	bool Triangle::IsSide(const Liar::Vector2f& pa, const Liar::Vector2f& pb)
	{
		for (int i = 0; i < 3; ++i)
		{
			Liar::Line2d* line = m_sides[i];
			if (line->Equals(pa, pb))
			{
				return true;
			}
		}

		return false;
	}

	Liar::Line2d& Triangle::GetSide(Liar::Uint sideIndex) const
	{
		return *(m_sides[sideIndex]);
	}

	Liar::Vector2f& Triangle::GetPointA() const
	{
		return *(GetVertex(m_pointIndexA));
	}

	Liar::Vector2f& Triangle::GetPointB() const
	{
		return *(GetVertex(m_pointIndexB));
	}

	Liar::Vector2f& Triangle::GetPointC() const
	{
		return *(GetVertex(m_pointIndexC));
	}

	bool Triangle::CalcInit()
	{
		if (m_centerCalculated) return false;
		m_center->Set(GetPointA());
		(*m_center) += GetPointB();
		(*m_center) += GetPointC();
		(*m_center) *= (1.0f / 3.0f);
		
		if (!m_sides)
		{
			m_sides = (Liar::Line2d**)malloc(sizeof(Liar::Line2d*) * 3);
			for (size_t i = 0; i < 3; ++i)
			{
				Liar::Line2d* line = (Liar::Line2d*)malloc(sizeof(Liar::Line2d));
				line->Set(m_map);
				m_sides[i] = line;
			}
		}

		m_sides[Liar::TriangleSide::SIDE_AB]->Set(m_pointIndexA, m_pointIndexB);
		m_sides[Liar::TriangleSide::SIDE_BC]->Set(m_pointIndexB, m_pointIndexC);
		m_sides[Liar::TriangleSide::SIDE_CA]->Set(m_pointIndexC, m_pointIndexA);
		m_centerCalculated = true;
		return true;
	}
}
