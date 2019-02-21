#include "WayPoint.h"

namespace Liar
{
	WayPoint::WayPoint() :
		m_position(nullptr),
		m_lineAPointA(nullptr), m_lineAPointB(nullptr),
		m_lineBPointA(nullptr), m_lineBPointB(nullptr),
		m_caller(nullptr)
	{
	}


	WayPoint::~WayPoint()
	{
		m_position->~Vector2f();
		free(m_position);
		m_position = nullptr;

		m_lineAPointA->~Vector2f();
		free(m_lineAPointA);
		m_lineAPointA = nullptr;

		m_lineAPointB->~Vector2f();
		free(m_lineAPointB);
		m_lineAPointB = nullptr;

		m_lineBPointA->~Vector2f();
		free(m_lineBPointA);
		m_lineBPointA = nullptr;

		m_lineBPointB->~Vector2f();
		free(m_lineBPointB);
		m_lineBPointB = nullptr;
	}

	void WayPoint::Init()
	{
		m_position = (Liar::Vector2f*)malloc(sizeof(Liar::Vector2f));
		m_position->Set(Liar::ZERO, Liar::ZERO);

		m_lineAPointA = (Liar::Vector2f*)malloc(sizeof(Liar::Vector2f));
		m_lineAPointB = (Liar::Vector2f*)malloc(sizeof(Liar::Vector2f));
		m_lineBPointA = (Liar::Vector2f*)malloc(sizeof(Liar::Vector2f));
		m_lineBPointB = (Liar::Vector2f*)malloc(sizeof(Liar::Vector2f));
		m_lineAPointA->Set(Liar::ZERO, Liar::ZERO);
		m_lineAPointB->Set(Liar::ZERO, Liar::ZERO);
		m_lineBPointA->Set(Liar::ZERO, Liar::ZERO);
		m_lineBPointB->Set(Liar::ZERO, Liar::ZERO);

		m_caller = nullptr;
	}

	void WayPoint::Set(Liar::Cell* call, const Liar::Vector2f& vec)
	{
		m_caller = call;
		m_position->Set(vec);
	}

	void WayPoint::SetPos(const Liar::Vector2f& pos) const
	{
		m_position->Set(pos);
	}

	void WayPoint::SetPos(Liar::NAVDTYPE x, Liar::NAVDTYPE y) const
	{
		m_position->Set(x, y);
	}

	void WayPoint::SetLineA(const Liar::Vector2f& pa, const Liar::Vector2f& pb)
	{
		m_lineAPointA->Set(pa);
		m_lineAPointB->Set(pb);
	}

	void WayPoint::SetLineB(const Liar::Vector2f& pa, const Liar::Vector2f& pb)
	{
		m_lineBPointA->Set(pa);
		m_lineBPointB->Set(pb);
	}

	/**
	* 下一个拐点
	* @param wayPoint 当前所在路点
	* @param cellPath 网格路径
	* @param end 终点
	* @return
	*/
	void WayPoint::GetFurthestWayPoint(Liar::Cell** cellPath, int cellCount, Liar::NAVDTYPE endX, Liar::NAVDTYPE endY, bool rw, Liar::NAVDTYPE epsilon)
	{
		Liar::Vector2f& startPt = *m_position;
		Liar::Cell* caller = m_caller;

		Liar::Line2d* outSide = caller->GetSide(caller->arrivalWall);	//路径线在网格中的穿出边
		Liar::Vector2f& lastPtA = outSide->GetPointA();
		Liar::Vector2f& lastPtB = outSide->GetPointB();

		Liar::NAVDTYPE lastPtAX = lastPtA.GetX();
		Liar::NAVDTYPE lastPtAY = lastPtA.GetY();
		Liar::NAVDTYPE lastPtBX = lastPtB.GetX();
		Liar::NAVDTYPE lastPtBY = lastPtB.GetY();

		SetLineA(startPt, lastPtA);
		SetLineB(startPt, lastPtB);

		int startIndex = 0, i = 0;
		for (i = 0; i < cellCount; ++i)
		{
#ifdef ShareFind
			if (caller == cellPath[i])
#else
			if(caller->Equals(*(cellPath[i])))
#endif // ShareFind
			{
				startIndex = i;
				break;
			}
		}

		int closeA = -1;
		Liar::Vector2f* closeAPoint = nullptr;
		int closeB = -1;
		Liar::Vector2f* closeBPoint = nullptr;

		Liar::NAVDTYPE testPtAX = Liar::ZERO;
		Liar::NAVDTYPE testPtAY = Liar::ZERO;
		Liar::NAVDTYPE testPtBX = Liar::ZERO;
		Liar::NAVDTYPE testPtBY = Liar::ZERO;

		for (i = startIndex + 1; i < cellCount; ++i)
		{
			caller = cellPath[i];
			outSide = caller->GetSide(caller->arrivalWall);

			if (i == cellCount - 1)
			{
				testPtAX = endX;
				testPtAY = endY;
				testPtBX = endX;
				testPtBY = endY;
			}
			else
			{
				const Liar::Vector2f& pa = outSide->GetPointA();
				const Liar::Vector2f& pb = outSide->GetPointB();
				testPtAX = pa.GetX();
				testPtAY = pa.GetY();
				testPtBX = pb.GetX();
				testPtBY = pb.GetY();
			}

			Liar::PointClassification typeA_A = Liar::Line2d::ClassifyPoint(*m_lineAPointA, *m_lineAPointB, testPtAX, testPtAY, rw, epsilon);
			Liar::PointClassification typeA_B = Liar::Line2d::ClassifyPoint(*m_lineAPointA, *m_lineAPointB, testPtBX, testPtBY, rw, epsilon);
			Liar::PointClassification typeB_B = Liar::Line2d::ClassifyPoint(*m_lineBPointA, *m_lineBPointB, testPtBX, testPtBY, rw, epsilon);
			Liar::PointClassification typeB_A = Liar::Line2d::ClassifyPoint(*m_lineBPointA, *m_lineBPointB, testPtAX, testPtAY, rw, epsilon);

			if (Liar::Line2d::Length(*m_lineAPointA, *m_lineAPointB) > epsilon && typeB_A == PointClassification::RIGHT_SIDE)
			{
				//路点
				if (closeB > 0)
				{
					m_caller = cellPath[closeB];
					m_position->Set(*closeBPoint);
				}
				else
				{
					m_caller = caller;
					m_position->Set(lastPtBX, lastPtBY);
				}
				return;
			}
			else if (Liar::Line2d::Length(*m_lineBPointA, *m_lineBPointB) > epsilon && typeA_B == PointClassification::LEFT_SIDE)
			{
				//路点
				if (closeA > 0)
				{
					m_caller = cellPath[closeA];
					m_position->Set(*closeAPoint);
				}
				else
				{
					m_caller = caller;
					m_position->Set(lastPtAX, lastPtAY);
				}
				return;
			}
			else if (typeA_A == Liar::PointClassification::ON_LINE && typeA_B == Liar::PointClassification::ON_LINE)	// 在同一条直线上(一条直线的起点和终点一样)
			{
				if (Liar::Line2d::Length(*m_lineAPointA, *m_lineAPointB) <= epsilon)
				{
					//lastCell = caller;
					// 这里表明lastLineB是一条线，lastLineA是个点
					if (typeB_A == Liar::PointClassification::LEFT_SIDE)
					{
						m_lineAPointB->Set(testPtAX, testPtAY);
						lastPtAX = testPtAX;
						lastPtAY = testPtAY;
						//setLineA = i;
					}
					else
					{
						m_lineBPointB->Set(testPtBX, testPtBY);
						lastPtBX = testPtBX;
						lastPtBY = testPtBY;
						//setLineB = i;
					}
				}
				else if (Liar::Line2d::Length(*m_lineAPointA, *m_lineAPointB) <= epsilon)
				{
					//lastCell = caller;
					// 这里表明lastLineA是一条线，lastLineB是个点
					if (typeA_B == Liar::PointClassification::RIGHT_SIDE)
					{
						m_lineBPointB->Set(testPtBX, testPtBY);
						lastPtBX = testPtBX;
						lastPtBY = testPtBY;
						//setLineB = i;
					}
					else
					{
						m_lineAPointB->Set(testPtAX, testPtAY);
						lastPtAX = testPtAX;
						lastPtAY = testPtAY;
						//setLineA = i;
					}
				}
			}
			else if (
				(typeA_A != Liar::PointClassification::ON_LINE && typeA_A == Liar::PointClassification::RIGHT_SIDE) || 
				(typeA_A == Liar::PointClassification::ON_LINE && Liar::Line2d::Length(*m_lineAPointA, *m_lineAPointB) <= epsilon)
				)
			{
				//lastCell = caller;
				//重设直线
				m_lineAPointB->Set(testPtAX, testPtAY);
				lastPtAX = testPtAX;
				lastPtAY = testPtAY;
				closeA = -1;
				closeAPoint = nullptr;
			}
			else if (
				(typeB_B != Liar::PointClassification::ON_LINE && typeB_B == Liar::PointClassification::LEFT_SIDE) || 
				(typeB_B == Liar::PointClassification::ON_LINE && Liar::Line2d::Length(*m_lineBPointA, *m_lineBPointB) <= epsilon)
				)
			{
				//lastCell = caller;
				//重设直线
				m_lineBPointB->Set(testPtBX, testPtBY);
				lastPtBX = testPtBX;
				lastPtBY = testPtBY;
				closeB = -1;
				closeBPoint = nullptr;

			}
			else if (typeB_B == PointClassification::RIGHT_SIDE || typeA_A == PointClassification::LEFT_SIDE)
			{
				if (typeB_B == PointClassification::RIGHT_SIDE)
				{
					// B遇到过障碍物了
					if (closeB <= 0)
					{
						closeB = i;
						closeBPoint = m_lineBPointB;
						//lastCell = caller;
					}
				}

				if (typeA_A == PointClassification::LEFT_SIDE)
				{
					// A遇到过障碍物了
					if (closeA <= 0)
					{
						closeA = i;
						closeAPoint = m_lineAPointB;
						//lastCell = caller;
					}
				}
			}

		}

		m_caller = cellPath[cellCount - 1];
		m_position->Set(endX, endY);
	}
}
