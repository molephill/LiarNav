#include "Line2d.h"
#include "Delaunay.h"

namespace Liar
{
	Line2d::Line2d(Liar::Map const* map) :
		Liar::MapSource(map),
		m_pointIndexA(0), m_pointIndexB(0)
	{
	}


	Line2d::~Line2d()
	{
		Dispose();
	}

	void Line2d::Dispose()
	{
		Liar::MapSource::Dispose();
	}

	void Line2d::Set(Liar::Map const* map)
	{
		Liar::MapSource::Set(map);
		m_pointIndexA = 0;
		m_pointIndexB = 0;
	}

	void Line2d::Set(Liar::Uint a, Liar::Uint b)
	{
		m_pointIndexA = a;
		m_pointIndexB = b;
	}

	void Line2d::Set(const Liar::Line2d& source)
	{
		Liar::MapSource::Set(source);
		m_pointIndexA = source.m_pointIndexA;
		m_pointIndexB = source.m_pointIndexB;
	}

	void Line2d::Set(Liar::Map const* map, Liar::Uint a, Liar::Uint b)
	{
		Liar::MapSource::Set(map);
		m_pointIndexA = a;
		m_pointIndexB = b;
	}

	Liar::Vector2f& Line2d::GetPointA() const
	{
		return *(GetVertex(m_pointIndexA));
	}

	Liar::Vector2f& Line2d::GetPointB() const
	{
		return *(GetVertex(m_pointIndexB));
	}

	/**
	* 判断两个直线关系
	* this line A = x0, y0 and B = x1, y1
	* other is A = x2, y2 and B = x3, y3
	* @param other 另一条直线
	* @param pIntersectPoint (out)返回两线段的交点
	* @return
	*/
	Liar::LineClassification Line2d::Intersection(const Liar::Line2d& other, Liar::Vector2f* pIntersectPoint)
	{
		return Liar::Delaunay::Intersection(*this, other, pIntersectPoint);
	}

	/**
	* 判断点与直线的关系，假设你站在a点朝向b点，
	* 则输入点与直线的关系分为：Left, Right or Centered on the line
	* @param point 点
	* @param epsilon 精度值
	* @return
	*/
	Liar::PointClassification Line2d::ClassifyPoint(const Liar::Vector2f& v, bool rw, Liar::EPSILONTYPE epsilon) const
	{
		return ClassifyPoint(v.GetX(), v.GetY(), rw, epsilon);
	}

	Liar::PointClassification Line2d::ClassifyPoint(Liar::NAVDTYPE x, Liar::NAVDTYPE y, bool rw, Liar::EPSILONTYPE epsilon) const
	{
		return Liar::Line2d::ClassifyPoint(*this, x, y, rw, epsilon);
	}

	/**
	* 给定点到直线的带符号距离，从a点朝向b点，右向为正，左向为负
	*/
	Liar::NAVDTYPE Line2d::SignedDistance(const Liar::Vector2f& v) const
	{
		return SignedDistance(v.GetX(), v.GetY());
	}

	Liar::NAVDTYPE Line2d::SignedDistance(Liar::NAVDTYPE x, Liar::NAVDTYPE y) const
	{
		return Liar::Line2d::SignedDistance(*this, x, y);
	}

	/**
	* 直线长度
	* @return
	*/
	Liar::NAVDTYPE Line2d::Length() const
	{
		return Liar::Line2d::Length(*this);
	}

	/**
	* 线段是否相等 （忽略方向）
	* @param line
	* @return
	*/
	bool Line2d::Equals(const Liar::Line2d& rhs, bool ignoreDir, Liar::EPSILONTYPE epsilon) const
	{
		return Equals(rhs.GetPointA(), rhs.GetPointB(), ignoreDir, epsilon);
	}

	bool Line2d::Equals(const Liar::Vector2f& pa, const Liar::Vector2f& pb, bool ignoreDir, Liar::EPSILONTYPE epsilon) const
	{
		return Equals(pa.GetX(), pa.GetY(), pb.GetX(), pb.GetY(), ignoreDir, epsilon);
	}

	bool Line2d::Equals(Liar::NAVDTYPE ax, Liar::NAVDTYPE ay, Liar::NAVDTYPE bx, Liar::NAVDTYPE by, bool ignoreDir, Liar::EPSILONTYPE epsilon) const
	{
		Liar::Vector2f& pointA = GetPointA();
		Liar::Vector2f& pointB = GetPointB();
		if (ignoreDir) return (pointA.Equals(ax, ay, epsilon) && pointB.Equals(bx, by, epsilon)) || (pointA.Equals(bx, by, epsilon) && pointB.Equals(ax, ay, epsilon));
		else return pointA.Equals(ax, ay, epsilon) && pointB.Equals(bx, by, epsilon);
	}

	// static function
	Liar::PointClassification Line2d::ClassifyPoint(const Liar::Line2d& line2d, const Liar::Vector2f& point, bool isCW, Liar::EPSILONTYPE eplision)
	{
		Liar::Vector2f& pa = line2d.GetPointA();
		Liar::Vector2f& pb = line2d.GetPointB();
		return Liar::Line2d::ClassifyPoint(pa, pb, point, isCW, eplision);
	}

	Liar::PointClassification Line2d::ClassifyPoint(const Liar::Line2d& line2d, Liar::NAVDTYPE x, Liar::NAVDTYPE y, bool isCW, Liar::EPSILONTYPE eplision)
	{
		Liar::Vector2f& pa = line2d.GetPointA();
		Liar::Vector2f& pb = line2d.GetPointB();
		return Liar::Line2d::ClassifyPoint(pa, pb, x, y, isCW, eplision);
	}

	Liar::PointClassification Line2d::ClassifyPoint(const Liar::Vector2f& pointA, const Liar::Vector2f& pointB, const Liar::Vector2f& point, bool isCW, Liar::EPSILONTYPE epsilon)
	{
		return Liar::Line2d::ClassifyPoint(pointA, pointB, point.GetX(), point.GetY(), isCW, epsilon);
	}

	Liar::PointClassification Line2d::ClassifyPoint(const Liar::Vector2f& pointA, const Liar::Vector2f& pointB, Liar::NAVDTYPE x, Liar::NAVDTYPE y, bool rw, Liar::EPSILONTYPE epsilon)
	{
		Liar::PointClassification result = Liar::PointClassification::ON_LINE;
		Liar::NAVDTYPE distance = Liar::Line2d::SignedDistance(pointA, pointB, x, y);
		if (rw)
		{
			if (distance > epsilon)
			{
				result = Liar::PointClassification::RIGHT_SIDE;
			}
			else if (distance < -epsilon)
			{
				result = Liar::PointClassification::LEFT_SIDE;
			}
		}
		else
		{
			if (distance > epsilon)
			{
				result = Liar::PointClassification::LEFT_SIDE;
			}
			else if (distance < -epsilon)
			{
				result = Liar::PointClassification::RIGHT_SIDE;
			}
		}
		return result;
	}

	Liar::NAVDTYPE Line2d::SignedDistance(const Liar::Line2d& line, Liar::NAVDTYPE x, Liar::NAVDTYPE y)
	{
		Liar::Vector2f& pa = line.GetPointA();
		Liar::Vector2f& pb = line.GetPointB();
		return Liar::Line2d::SignedDistance(pa, pb, x, y);
	}

	Liar::NAVDTYPE Line2d::SignedDistance(const Liar::Vector2f& pointA, const Liar::Vector2f& pointB, Liar::NAVDTYPE x, Liar::NAVDTYPE y)
	{
		Liar::NAVDTYPE pointAX = pointA.GetX();
		Liar::NAVDTYPE pointAY = pointA.GetY();
		Liar::NAVDTYPE pointBX = pointB.GetX();
		Liar::NAVDTYPE pointBY = pointB.GetY();

		NAVDTYPE tmpx = x - pointAX;
		NAVDTYPE tmpy = y - pointAY;

		NAVDTYPE normalx = pointBX - pointAX;
		NAVDTYPE normaly = pointBY - pointAY;
		NAVDTYPE len = normalx * normalx + normaly * normaly;
		len = sqrtf(len);
		len = len == 0 ? 1 : len;
		normalx /= len;
		normaly /= len;

		NAVDTYPE tmppx = normalx;
		normalx = -normaly;
		normaly = tmppx;

		NAVDTYPE out = tmpx * normalx + tmpy * normaly;
		return out;
	}

	Liar::NAVDTYPE Line2d::Length(const Liar::Line2d& line)
	{
		Liar::Vector2f& pa = line.GetPointA();
		Liar::Vector2f& pb = line.GetPointB();
		return Liar::Line2d::Length(pa, pb);
	}

	Liar::NAVDTYPE Line2d::Length(const Liar::Vector2f& pointA, const Liar::Vector2f& pointB)
	{
		NAVDTYPE xdis = pointB.GetX() - pointA.GetX();
		NAVDTYPE ydis = pointB.GetY() - pointA.GetY();
		return sqrt(xdis*xdis + ydis * ydis);
	}
}
