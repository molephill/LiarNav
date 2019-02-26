#include "Cell.h"

namespace Liar
{
	Cell::Cell(Liar::Map const* map) :
		Liar::Triangle(map),
		m_index(0), m_links((Liar::Int*)malloc(sizeof(Liar::Int) * 3)),
		m_wallDistance((Liar::NAVDTYPE*)malloc(sizeof(Liar::NAVDTYPE) * 3)),
		sessionId(0), f(Liar::ZERO), h(Liar::ZERO),
		isOpen(false), parent(nullptr), arrivalWall(-1), checkLinkCount(0)
	{
	}


	Cell::~Cell()
	{
		Dispose();
	}

	void Cell::Dispose()
	{
		Liar::Triangle::Dispose();
		free(m_links);
		free(m_wallDistance);
	}

	// TODO extend
	bool Cell::Set(Liar::Map const* map, Liar::Uint a, Liar::Uint b, Liar::Uint c)
	{
		Liar::Triangle::Set(map);

		m_links = (Liar::Int*)malloc(sizeof(Liar::Int) * 3);
		m_links[0] = m_links[1] = m_links[2] = -1;
		m_wallDistance = (Liar::NAVDTYPE*)malloc(sizeof(Liar::NAVDTYPE) * 3);
		m_wallDistance[0] = m_wallDistance[1] = m_wallDistance[2] = Liar::ZERO;
		m_index = sessionId = arrivalWall = checkLinkCount = 0;
		parent = nullptr;
		isOpen = false;
		f = h = Liar::ZERO;

		m_pointIndexA = a;
		m_pointIndexB = b;
		m_pointIndexC = c;
		CalcSides();
		return true;
	}

	void Cell::Set(const Liar::Cell& source, bool init)
	{
		Liar::Triangle::Set(source, init);

		size_t blockSize = sizeof(Liar::Int) * 3;

		if(init || !m_links) m_links = (Liar::Int*)malloc(blockSize);
		size_t i = 0;
		for (i = 0; i < 3; ++i) m_links[i] = source.m_links[i];

		if (init || !m_wallDistance)
		{
			blockSize = sizeof(Liar::NAVDTYPE) * 3;
			m_wallDistance = (Liar::NAVDTYPE*)malloc(blockSize);
		}
		for (i = 0; i < 3; ++i) m_wallDistance[i] = source.m_wallDistance[i];

		m_index = source.m_index;
		sessionId = source.sessionId;
		f = source.f;
		h = source.h;
		isOpen = source.isOpen;
		parent = source.parent;
		arrivalWall = source.arrivalWall;
		checkLinkCount = source.checkLinkCount;
	}

	void Cell::SetIndex(Liar::Int index)
	{
		m_index = index;
	}

	bool Cell::Equals(const Liar::Cell& source)
	{
		return m_index == source.m_index;
	}

	bool Cell::CalcSides()
	{
		bool calc = Liar::Triangle::CalcSides();
		if (calc)
		{
			Liar::Vector2f& pointA = GetPointA();
			Liar::Vector2f& pointB = GetPointB();
			Liar::Vector2f& pointC = GetPointC();

			Liar::NAVDTYPE ax = pointA.GetX();
			Liar::NAVDTYPE ay = pointA.GetY();

			Liar::NAVDTYPE bx = pointB.GetX();
			Liar::NAVDTYPE by = pointB.GetY();

			Liar::NAVDTYPE cx = pointC.GetX();
			Liar::NAVDTYPE cy = pointC.GetY();

			size_t size = sizeof(Liar::Vector2f);
			Liar::Vector2f* wallMidPoint = (Liar::Vector2f*)malloc(size);
			Liar::Vector2f* wallVector = (Liar::Vector2f*)malloc(size);

			// p0 - p1
			wallVector->Set((ax + bx)*0.5f, (ay + by)*0.5f); // p0
			wallMidPoint->Set((cx + bx)*0.5f, (cy + by)*0.5f); // p1
			*(wallVector) -= *(wallMidPoint);
			m_wallDistance[0] = wallVector->Length();

			// p1 - p2
			wallVector->Set((cx + bx)*0.5f, (cy + by)*0.5f); // p1
			wallMidPoint->Set((cx + ax)*0.5f, (cy + ay)*0.5f); // p2;
			*(wallVector) -= *(wallMidPoint);
			m_wallDistance[1] = wallVector->Length();

			// p2 - p0
			wallVector->Set((cx + ax)*0.5f, (cy + ay)*0.5f); // p2
			wallMidPoint->Set((ax + bx)*0.5f, (ay + by)*0.5f); // p0;
			*(wallVector) -= *(wallMidPoint);
			m_wallDistance[2] = wallVector->Length();

			wallMidPoint->~Vector2f();
			free(wallMidPoint);
			wallMidPoint = nullptr;

			wallVector->~Vector2f();
			free(wallVector);
			wallVector = nullptr;
		}
		return calc;
	}

	/**
	* 检查并设置当前三角型与CellB的连接关系（方法会同时设置CellB与该三角型的连接）
	* @param CellB
	*/
	void Cell::CheckAndLink(Liar::Cell& cellB)
	{
		Liar::Vector2f& pointA = GetPointA();
		Liar::Vector2f& pointB = GetPointB();
		Liar::Vector2f& pointC = GetPointC();

		if (GetLink(Liar::TriangleSide::SIDE_AB) == -1 && cellB.RequestList(pointA, pointB, *this))
		{
			SetLink(Liar::TriangleSide::SIDE_AB, cellB);
		}
		else if (GetLink(Liar::TriangleSide::SIDE_BC) == -1 && cellB.RequestList(pointB, pointC, *this))
		{
			SetLink(Liar::TriangleSide::SIDE_BC, cellB);
		}
		else if (GetLink(Liar::TriangleSide::SIDE_CA) == -1 && cellB.RequestList(pointC, pointA, *this))
		{
			SetLink(Liar::TriangleSide::SIDE_CA, cellB);
		}
	}

	/**
	* 获得两个点的相邻三角型
	* @param pA
	* @param pB
	* @param caller
	* @return 如果提供的两个点是caller的一个边, 返回true
	*/
	bool Cell::RequestList(const Liar::Vector2f& pa, const Liar::Vector2f& pb, Liar::Cell& caller)
	{
		Liar::Vector2f& pointA = GetPointA();
		Liar::Vector2f& pointB = GetPointB();
		Liar::Vector2f& pointC = GetPointC();

		Liar::NAVDTYPE paX = pa.GetX();
		Liar::NAVDTYPE paY = pa.GetY();

		Liar::NAVDTYPE pbX = pb.GetX();
		Liar::NAVDTYPE pbY = pb.GetY();

		if (pointA.Equals(paX, paY))
		{
			if (pointB.Equals(pbX, pbY))
			{
				SetLink(TriangleSide::SIDE_AB, caller);
				return true;
			}
			else if (pointC.Equals(pbX, pbY))
			{
				SetLink(TriangleSide::SIDE_CA, caller);
				return true;
			}
		}
		else if (pointB.Equals(paX, paY))
		{
			if (pointA.Equals(pbX, pbY))
			{
				SetLink(TriangleSide::SIDE_AB, caller);
				return true;
			}
			else if (pointC.Equals(pbX, pbY))
			{
				SetLink(TriangleSide::SIDE_BC, caller);
				return true;
			}
		}
		else if (pointC.Equals(paX, paY))
		{
			if (pointA.Equals(pbX, pbY))
			{
				SetLink(TriangleSide::SIDE_CA, caller);
				return true;
			}
			else if (pointB.Equals(pbX, pbY))
			{
				SetLink(TriangleSide::SIDE_BC, caller);
				return true;
			}
		}

		// we are not adjacent to the calling Cell
		return false;

	}

	/**
	* 设置side指定的边的连接到caller的索引
	* @param side
	* @param caller
	*/
	void Cell::SetLink(Liar::TriangleSide side, const Liar::Cell& caller)
	{
		m_links[side] = caller.m_index;
	}

	/**
	* 取得指定边的相邻三角型的索引
	* @param side
	* @return
	*/
	Liar::Int Cell::GetLink(Liar::Uint side) const
	{
		return m_links[side];
	}

	Liar::NAVDTYPE Cell::GetWallDistatnce(Liar::TriangleSide side) const
	{
		return m_wallDistance[side];
	}

	// 检测是否检测完也所有的链接
	bool Cell::CheckAllLink() const
	{
		int count = 0;
		for (int i = 0; i < 3; ++i)
		{
			if (m_links[i] > 0) count += 1;
		}
		return checkLinkCount >= count;
	}

	/**
	* 计算估价（h）  Compute the A* Heuristic for this Cell given a Goal point
	* @param goal
	*/
	void Cell::ComputeHeuristic(const Vector2f& goal)
	{
		ComputeHeuristic(goal.GetX(), goal.GetY());
	}

	void Cell::ComputeHeuristic(Liar::NAVDTYPE x, Liar::NAVDTYPE y)
	{
		// our heuristic is the estimated distance (using the longest axis delta) 
		// between our Cell center and the goal location

		NAVDTYPE XDelta = abs(x - m_center->GetX());
		NAVDTYPE YDelta = abs(y - m_center->GetY());

		h = XDelta + YDelta;
	}

	/**
	* 记录路径从上一个节点进入该节点的边（如果从终点开始寻路即为穿出边）
	* @param index	路径上一个节点的索引
	*/
	int Cell::SetAndGetArrivalWall(int index)
	{
		if (index == m_links[0])
		{
			arrivalWall = 0;
			return 0;
		}
		else if (index == m_links[1])
		{
			arrivalWall = 1;
			return 1;
		}
		else if (index == m_links[2])
		{
			arrivalWall = 2;
			return 2;
		}

		return -1;
	}

#if defined(DEBUG_NIF) || defined(EditorMod)
	void Cell::WriteErlang(std::ofstream& outfile)
	{
		Liar::Vector2f& pa = GetPointA();
		Liar::Vector2f& pb = GetPointB();
		Liar::Vector2f& pc = GetPointC();

		outfile << "index:" << m_index << " ";
		outfile << "A:{" << pa.GetX() << "," << pa.GetY() << "} ";
		outfile << "B:{" << pb.GetX() << "," << pb.GetY() << "} ";
		outfile << "C:{" << pc.GetX() << "," << pc.GetY() << "}\n";
		outfile << "links:{" << m_links[0] << "," << m_links[1] << "," << m_links[2] << "}\n";
		outfile << "wallDistance:{" << m_wallDistance[0] << "," << m_wallDistance[1] << "," << m_wallDistance[2] << "}\n";
	}
#endif
}
