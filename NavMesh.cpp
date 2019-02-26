#include "NavMesh.h"
#include "Map.h"

namespace Liar
{
#ifdef EditorMod
#ifdef ShareFind
#ifdef FindNearest
	NavMesh::NavMesh(const Liar::Map* map) :
		Liar::MapSource(map),
		m_openList((Heap*)malloc(sizeof(Heap))), m_closeList(nullptr), m_closeCount(0),
		m_path(nullptr), m_numPath(0),
		m_wayPoint(nullptr),
		m_crossList(nullptr), m_crossCount(0),
		m_nearstCells(nullptr), m_nearstCount(0)
#else
	NavMesh::NavMesh(const Liar::Map* map) :
		Liar::MapSource(map),
		m_openList((Heap*)malloc(sizeof(Heap))), m_closeList(nullptr), m_closeCount(0),
		m_path(nullptr), m_numPath(0),
		m_wayPoint(nullptr),
		m_crossList(nullptr), m_crossCount(0)
#endif // FindNearest
#else
#ifdef FindNearest
	NavMesh::NavMesh(const Liar::Map* map) :
		Liar::MapSource(map),
		m_openList((Heap*)malloc(sizeof(Heap))), m_closeList(nullptr), m_closeCount(0),
		m_wayPoint(nullptr),
		m_crossList(nullptr), m_crossCount(0),
		m_nearstCells(nullptr), m_nearstCount(0),
		m_pathsession(0)
#else
	NavMesh::NavMesh(const Liar::Map* map) :
		Liar::MapSource(map),
		m_openList((Heap*)malloc(sizeof(Heap))), m_closeList(nullptr), m_closeCount(0),
		m_path(nullptr), m_numPath(0),
		m_wayPoint(nullptr),
		m_crossList(nullptr), m_crossCount(0),
		m_pathsession(0)
#endif // FindNearest
#endif // ShareFind
#else
#ifdef ShareFind
#ifdef FindNearest
	NavMesh::NavMesh(const Liar::Map* map) :
		Liar::MapSource(map),
		m_openList((Heap*)malloc(sizeof(Heap))), m_closeList(nullptr), m_closeCount(0),
		m_path(nullptr), m_numPath(0),
		m_wayPoint(nullptr),
		m_nearstCells(nullptr), m_nearstCount(0)
#else
	NavMesh::NavMesh(const Liar::Map* map) :
		Liar::MapSource(map),
		m_openList((Heap*)malloc(sizeof(Heap))), m_closeList(nullptr), m_closeCount(0),
		m_path(nullptr), m_numPath(0),
		m_wayPoint(nullptr)
#endif // FindNearest
#else
#ifdef FindNearest
	NavMesh::NavMesh(const Liar::Map* map) :
		Liar::MapSource(map),
		m_openList((Heap*)malloc(sizeof(Heap))), m_closeList(nullptr), m_closeCount(0),
		m_wayPoint(nullptr),
		m_nearstCells(nullptr), m_nearstCount(0),
		m_pathsession(0)
#else
	NavMesh::NavMesh(const Liar::Map* map) :
		Liar::MapSource(map),
		m_openList((Heap*)malloc(sizeof(Heap))), m_closeList(nullptr), m_closeCount(0),
		m_path(nullptr), m_numPath(0),
		m_wayPoint(nullptr)
		m_pathsession(0)
#endif // FindNearest
#endif // ShareFind
#endif // EditorMod

	{
	}

	NavMesh::~NavMesh()
	{
		Dispose();
	}

	void NavMesh::Dispose()
	{
		if (m_openList)
		{
			m_openList->~Heap();
			free(m_openList);
			m_openList = nullptr;
		}

		if (m_closeList)
		{
			free(m_closeList);
			m_closeList = nullptr;
		}

		if (m_wayPoint)
		{
			m_wayPoint->~WayPoint();
			free(m_wayPoint);
			m_wayPoint = nullptr;
		}

		DisposePath();
#ifdef EditorMod
		DisposeCross();
#endif // EditorMod
	}

	void NavMesh::Init(const Liar::Map* map)
	{
		Liar::MapSource::Set(map);

		m_openList = nullptr;

		m_closeList = nullptr;
		m_closeCount = 0;

		m_path = nullptr;
		m_numPath = 0;

		m_wayPoint = nullptr;

#ifdef EditorMod
		m_crossList = nullptr;
		m_crossCount = 0;
#endif // EditorMod

		m_pathsession = 0;

#ifdef FindNearest
		m_nearstCells = nullptr;
		m_nearstCount = 0;
#endif // FindNearest
	}

	void NavMesh::Set(const Liar::Map* map)
	{
		if (map != m_map)
		{
			if (m_openList)
			{
				m_openList->~Heap();
				free(m_openList);
				m_openList = nullptr;
			}
			m_openList = (Liar::Heap*)malloc(sizeof(Liar::Heap));
			m_openList->Set(map->GetCellCount());
		}
		Liar::MapSource::Set(map);
	}

	Liar::Vector2f** NavMesh::FindPath(Liar::NAVDTYPE startX, Liar::NAVDTYPE startY, Liar::NAVDTYPE endX, Liar::NAVDTYPE endY, Liar::Uint& outLen, bool rw)
	{
		if (m_numPath >= Liar::NavMesh::PATHMAX) DisposePath();

		outLen = 0;

		// release cross;
#ifdef EditorMod
		DisposeCross();
#endif // EditorMod

#ifdef ShareFind
		++Liar::NavMesh::PATHSESSIONID;
#else
		++m_pathsession;
#endif // ShareFind

		Liar::Cell* startCell = FindClosestCell(startX, startY, rw);
#ifndef OutFind
		if (!startCell) return nullptr;
#endif // !OutFind
		Liar::Cell* endCell = FindClosestCell(endX, endY, rw);
		if (!endCell) return nullptr;

#ifdef OutFind
		if (!startCell)
		{
			AddPathPoint(startX, startY, outLen);
			AddPathPoint(endX, endY, outLen);
			return m_path;
		}
#endif // OutFind

#ifdef FindNearest
		if (startCell == endCell || TestOneLine2D(startX, startY, endX, endY, startCell, endCell))
#else
		if (startCell == endCell)
#endif // FindNearest
		{
			AddPathPoint(startX, startY, outLen);
			AddPathPoint(endX, endY, outLen);
		}
		else
		{
			BuildPath(startCell, startX, startY, endCell, endX, endY, outLen, rw);
		}
		return m_path;
	}

	/**
	* 构建路径
	* @param startCell
	* @param startPos
	* @param endCell
	* @param endPos
	* @return Point路径数组
	*/
	void NavMesh::BuildPath(
		Liar::Cell* startCell, const Liar::Vector2f& startPos,
		Liar::Cell* endCell, const Liar::Vector2f& endPos, 
		Liar::Uint& outLen, bool rw)
	{
		BuildPath(startCell, startPos.GetX(), startPos.GetY(), endCell, endPos.GetX(), endPos.GetY(), outLen, rw);
	}

	void NavMesh::BuildPath(
		Liar::Cell* startCell, Liar::NAVDTYPE startX, Liar::NAVDTYPE startY,
		Liar::Cell* endCell, Liar::NAVDTYPE endX, Liar::NAVDTYPE endY, 
		Liar::Uint& outLen, bool rw)
	{
		int SESSIONID = 0;
#ifdef ShareFind
		SESSIONID = Liar::NavMesh::PATHSESSIONID;
#else
		SESSIONID = m_pathsession;
#endif // ShareFind
		m_openList->Clear();
		m_openList->Push(endCell);
		endCell->f = 0;
		endCell->h = 0;
		endCell->isOpen = false;
		endCell->parent = nullptr;
		endCell->sessionId = SESSIONID;

		bool foundPath = false;			//是否找到路径
		Cell* currNode = nullptr;		//当前节点
		Cell* adjacentTmp = nullptr;	//当前节点的邻接三角型
		Cell* sourceAdjacentTmp = nullptr;

		while (m_openList->Size())
		{
			// 1. 把当前节点从开放列表删除, 加入到封闭列表
			currNode = m_openList->Pop();
			++m_closeCount;
			if (m_closeList) m_closeList = (Liar::Cell**)realloc(m_closeList, sizeof(Liar::Cell*)*m_closeCount);
			else m_closeList = (Liar::Cell**)malloc(sizeof(Liar::Cell*)*m_closeCount);
			m_closeList[m_closeCount - 1] = currNode;

			if (currNode == startCell)
			{
				foundPath = true;
				break;
			}

			// 2. 对当前节点相邻的每一个节点依次执行以下步骤:
			//所有邻接三角型
			int adjacentId = 0;
			for (int i = 0; i < 3; ++i)
			{
				adjacentId = currNode->GetLink(i);
				// 3. 如果该相邻节点不可通行或者该相邻节点已经在封闭列表中,
				//    则什么操作也不执行,继续检验下一个节点;
				if (adjacentId < 0)		//不能通过
				{
					continue;
				}
				else
				{
					sourceAdjacentTmp = m_map->GetCell(adjacentId);
				}

				if (sourceAdjacentTmp)
				{
					adjacentTmp = sourceAdjacentTmp;

					if (adjacentTmp->sessionId != SESSIONID)
					{
						// 4. 如果该相邻节点不在开放列表中,则将该节点添加到开放列表中, 
						//    并将该相邻节点的父节点设为当前节点,同时保存该相邻节点的G和F值;
						adjacentTmp->sessionId = SESSIONID;
						adjacentTmp->parent = currNode;
						adjacentTmp->isOpen = true;

						//H和F值
						adjacentTmp->ComputeHeuristic(startX, startY);
						adjacentTmp->f = currNode->f + adjacentTmp->GetWallDistance(abs(i - currNode->arrivalWall));

						//放入开放列表并排序
						m_openList->Push(adjacentTmp);

						// remember the side this caller is entering from
						adjacentTmp->SetAndGetArrivalWall(currNode->GetIndex());
						if (currNode->arrivalWall < 0)
						{
							currNode->SetAndGetArrivalWall(adjacentTmp->GetIndex());
						}
					}
					else
					{
						// 5. 如果该相邻节点在开放列表中, 
						//    则判断若经由当前节点到达该相邻节点的G值是否小于原来保存的G值,
						//    若小于,则将该相邻节点的父节点设为当前节点,并重新设置该相邻节点的G和F值
						if (adjacentTmp->isOpen)	//已经在openList中
						{
							if (currNode->f + adjacentTmp->GetWallDistance(abs(i - currNode->arrivalWall)) < adjacentTmp->f)
							{
								adjacentTmp->f = currNode->f;
								adjacentTmp->parent = currNode;

								// remember the side this caller is entering from
								adjacentTmp->SetAndGetArrivalWall(currNode->GetIndex());
								if (currNode->arrivalWall < 0)
								{
									currNode->SetAndGetArrivalWall(adjacentTmp->GetIndex());
								}
							}
						}
						else
						{
							//已在closeList中
							//adjacentTmp = nullptr;
							continue;
						}
					}
				}
			}
		}

		//由网格路径生成Point数组路径
		if (foundPath)
		{
			GetPath(startX, startY, endX, endY, outLen, rw);
		}
	}

	/**
	* 根据经过的三角形返回路径点(下一个拐角点法)
	* @param start
	* @param end
	* @return Point数组
	*/
	void NavMesh::GetPath(const Liar::Vector2f& start, const Liar::Vector2f& end, Liar::Uint& outLen, bool rw)
	{
		GetPath(start.GetX(), start.GetY(), end.GetX(), end.GetY(), outLen, rw);
	}

	void NavMesh::GetPath(Liar::NAVDTYPE startX, Liar::NAVDTYPE startY, Liar::NAVDTYPE endX, Liar::NAVDTYPE endY, Liar::Uint& outLen, bool rw)
	{
		//经过的三角形
		Liar::Uint numCellPath = 0;
		Liar::Cell** pathCells = GetCellPath(numCellPath);

		if (numCellPath > 0)
		{
			//开始点
			AddPathPoint(startX, startY, outLen);
			//起点与终点在同一三角形中
			if (numCellPath == 1)
			{
				AddPathPoint(endX, endY, outLen);
			}
			else
			{
				if (!m_wayPoint)
				{
					m_wayPoint = (Liar::WayPoint*)malloc(sizeof(Liar::WayPoint));
					m_wayPoint->Init();
				}
				m_wayPoint->Set(pathCells[0], *(m_path[0]));

#ifdef FindNearest
				AddNearestCaller(m_wayPoint->GetCaller());
#endif // FindNearest

				while (!(m_wayPoint->GetPos().Equals(endX, endY)))
				{
					m_wayPoint->GetFurthestWayPoint(pathCells, numCellPath, endX, endY, rw);
					AddPathPoint(m_wayPoint->GetPos(), outLen);

#ifdef FindNearest
					AddNearestCaller(m_wayPoint->GetCaller());
#endif // FindNearest

				}

#ifdef FindNearest
				FindNearestPath(0, m_path, outLen);
				for (Liar::Uint i = 0; i < m_nearstCount; ++i) m_nearstCells[i]->checkLinkCount = 0;
				free(m_nearstCells);
				m_nearstCells = nullptr;
				m_nearstCount = 0;
#endif // FindNearest

			}

			m_closeCount = 0;
			free(m_closeList);
			m_closeList = nullptr;

			free(pathCells);
			pathCells = nullptr;
		}
	}

#ifdef FindNearest

	void NavMesh::AddNearestCaller(Liar::Cell* cell)
	{
		++m_nearstCount;
		size_t blockSize = sizeof(Liar::Cell*)*m_nearstCount;
		if (m_nearstCells) m_nearstCells = (Liar::Cell**)realloc(m_nearstCells, blockSize);
		else m_nearstCells = (Liar::Cell**)malloc(blockSize);
		m_nearstCells[m_nearstCount - 1] = cell;
	}

	void NavMesh::FindNearestPath(Liar::Uint startIndex, Liar::Vector2f** path, Liar::Uint& pathCount)
	{
		if (startIndex >= pathCount)
		{
			return;
		}

		int revertIndex = pathCount - startIndex - 1;
		Liar::Vector2f* startPos = path[revertIndex];
		Liar::Cell* startCell = m_nearstCells[revertIndex];

		int farIndex = -1;

		for (int i = 0; i < revertIndex; ++i)
		{
			Liar::Vector2f* checkPos = path[i];
			Liar::Cell* checkCell = m_nearstCells[i];

			if (startCell == checkCell) continue;

			if (abs(i - revertIndex) == 2)
			{
				Liar::Vector2f* nextPos = path[i + 1];
				Liar::Cell* nextCell = m_nearstCells[i + 1];
				if (
					checkCell->IsSide(*checkPos, *startPos) || 
					checkCell->IsSide(*checkPos, *nextPos) || 
					nextCell->IsSide(*checkPos, *startPos) || 
					nextCell->IsSide(*nextPos, *startPos)
					)
				{
					break;
				}
			}

			if (TestOneLine2D(*startPos, *checkPos, startCell, checkCell))
			{
				if (TestOneLine2D(*checkPos, *startPos, checkCell, startCell))
				{
					farIndex = i;
					break;
				}
			}
		}

		Liar::Uint setStartIndex = farIndex + 1;
		Liar::Uint resetCount = revertIndex - setStartIndex;
		Liar::Uint resetIndex = 0;
		Liar::Uint i = 0;
		if (farIndex >= 0 && resetCount > 0)
		{
			for (i = setStartIndex; i < static_cast<Liar::Uint>(revertIndex); ++i)
			{
				/*path[i]->~Vector2f();
				free(path[i]);
				path[i] = nullptr;*/
				m_nearstCells[i]->checkLinkCount = 0;
				m_nearstCells[i] = nullptr;
			}

			resetIndex = setStartIndex;
			Liar::Vector2f* tmp = nullptr;
			for (i = revertIndex; i < pathCount; ++i)
			{
				tmp = path[resetIndex];
				path[resetIndex++] = path[i];
				path[i] = tmp;
			}

			resetIndex = setStartIndex;
			for (i = revertIndex; i < m_nearstCount; ++i)
			{
				m_nearstCells[resetIndex++] = m_nearstCells[i];
			}

			pathCount -= resetCount;
			m_nearstCount -= resetCount;
		}

		FindNearestPath(startIndex + 1, path, pathCount);
	}

	// 检测两点间是否能直达
	bool NavMesh::TestOneLine2D(const Vector2f& start, const Vector2f& end, Cell* startCell, Cell* endCell)
	{
		return TestOneLine2D(start.GetX(), start.GetY(), end.GetX(), end.GetY(), startCell, endCell);
	}

	bool NavMesh::TestOneLine2D(
		Liar::NAVDTYPE startX, Liar::NAVDTYPE startY,
		Liar::NAVDTYPE endX, Liar::NAVDTYPE endY,
		Liar::Cell* startCell, Liar::Cell* endCell
	)
	{
		// 记录已经找过的路径
		Liar::Cell** crossCells = (Liar::Cell**)malloc(sizeof(Liar::Cell*));
		int crossCount = 1;
		Liar::Cell* cell = endCell;
		crossCells[0] = cell;

		bool findCross = true;

		while (cell && cell != startCell)
		{
			cell = GetCrossCell(startX, startY, endX, endY, *cell, crossCells, crossCount, findCross);
			findCross = false;
			if (cell)
			{
				crossCount++;
				crossCells = (Liar::Cell**)realloc(crossCells, sizeof(Liar::Cell*)*crossCount);
				crossCells[crossCount - 1] = cell;
			}
			else
			{
				if (crossCount - 2 >= 0)
				{
					Liar::Cell* preCell = crossCells[crossCount - 2];
					if (!preCell->CheckAllLink())
					{
						cell = preCell;
					}
				}
			}
		}
		for (int i = 0; i < crossCount; ++i)
		{
			crossCells[i]->checkLinkCount = 0;
		}
		free(crossCells);

		return (cell && cell == startCell);
	}

	Cell* NavMesh::GetCrossCell(const Vector2f& start, const Vector2f& end, Liar::Cell& testCell, Liar::Cell** crossList, int count, bool findCross)
	{
		return GetCrossCell(start.GetX(), start.GetY(), end.GetX(), end.GetY(), testCell, crossList, count, findCross);
	}

	Liar::Cell* NavMesh::GetCrossCell(
		Liar::NAVDTYPE startX, Liar::NAVDTYPE startY,
		Liar::NAVDTYPE endX, Liar::NAVDTYPE endY,
		Liar::Cell& testCell, Liar::Cell** crossList, int count, bool findCross
	)
	{
		int needCount = 2;
		int interCount = 0;

		int adjacentId = 0;
		Liar::Cell* sourceAdjacentTmp = nullptr;
		int startCrossCount = 0;
		Liar::Cell* outTmp = nullptr;

		Liar::Cell** cells = m_map->GetCells();

		for (Liar::Uint i = 0; i < 3; ++i)
		{
			adjacentId = testCell.GetLink(i);
			if (adjacentId < 0)
			{
				continue;
			}
			else
			{
				sourceAdjacentTmp = cells[adjacentId];

				bool crossed = false;
				for (int f = 0; f < count; ++f)
				{
					if (crossList[f] == sourceAdjacentTmp)
					{
						crossed = true;
						break;
					}
				}

				testCell.checkLinkCount += 1;

				if (!crossed)
				{
					interCount = TestLineCell(*sourceAdjacentTmp, startX, startY, endX, endY);
					if (interCount >= needCount)
					{
						if (interCount > startCrossCount)
						{
							outTmp = sourceAdjacentTmp;
							startCrossCount = interCount;
							if (!findCross || interCount > needCount) break;
						}
					}
				}
			}
		}
		return outTmp;
	}

	int NavMesh::TestLineCell(const Cell& sourceAdjacentTmp, const Vector2f& start, const Vector2f& end)
	{
		return TestLineCell(sourceAdjacentTmp, start.GetX(), start.GetY(), end.GetX(), end.GetY());
	}

	int NavMesh::TestLineCell(
		const Liar::Cell& sourceAdjacentTmp,
		Liar::NAVDTYPE startX, Liar::NAVDTYPE startY,
		Liar::NAVDTYPE endX, Liar::NAVDTYPE endY)
	{
		int crossCount = 0;
		for (int j = 0; j < 3; ++j)
		{
			Liar::Line2d* testline = sourceAdjacentTmp.GetSide(j);
			Liar::Vector2f& pa = testline->GetPointA();
			Liar::Vector2f& pb = testline->GetPointB();
			int intersect1 = LineIntersectSide(endX, endY, startX, startY, pa, pb);
			int intersect2 = LineIntersectSide(pa, pb, endX, endY, startX, startY);
			int intersect = intersect1 < intersect2 ? intersect1 : intersect2;
			if (intersect == 0) // 如果是在共用边上的交点，也是可以忽略的
			{
				/*if (sourceAdjacentTmp.arrivalWall < 0)
				{
				++crossCount;
				}
				else
				{
				if (_crossCount <= 0) continue;
				NAVDTYPE tx = 0.0;
				NAVDTYPE ty = 0.0;
				GetCrossVector2f(end, start, *pa, *pb, tx, ty);
				if (pa->Equals(tx, ty, EPSILON) || pb->Equals(tx, ty, EPSILON))
				{
				++crossCount;
				}
				}*/
				++crossCount;
			}
			else if (intersect > 0)
			{
				if (sourceAdjacentTmp.GetLink(j) > 0)
				{
					crossCount += 2;
				}
				else
				{
					return 0;
				}
			}
		}

		return crossCount;
	}

	int NavMesh::LineIntersectSide(const Vector2f& A, const Vector2f& B, const Vector2f& C, const Vector2f& D)
	{
		// A(x1, y1), B(x2, y2)的直线方程为：  
		// f(x, y) =  (y - y1) * (x1 - x2) - (x - x1) * (y1 - y2) = 0  

		Liar::NAVDTYPE AX = A.GetX();
		Liar::NAVDTYPE AY = A.GetY();

		Liar::NAVDTYPE BX = B.GetX();
		Liar::NAVDTYPE BY = B.GetY();

		Liar::NAVDTYPE CX = C.GetX();
		Liar::NAVDTYPE CY = C.GetY();

		Liar::NAVDTYPE DX = D.GetX();
		Liar::NAVDTYPE DY = D.GetY();

		return LineIntersectSide(AX, AY, BX, BY, CX, CY, DX, DY);
	}

	int NavMesh::LineIntersectSide(
		const Liar::Vector2f& A,
		const Liar::Vector2f& B,
		Liar::NAVDTYPE CX, Liar::NAVDTYPE CY,
		Liar::NAVDTYPE DX, Liar::NAVDTYPE DY
	)
	{
		return LineIntersectSide(A.GetX(), A.GetY(), B.GetX(), B.GetY(), CX, CY, DX, DY);
	}

	int NavMesh::LineIntersectSide(
		Liar::NAVDTYPE AX, Liar::NAVDTYPE AY,
		Liar::NAVDTYPE BX, Liar::NAVDTYPE BY,
		const Liar::Vector2f& C,
		const Liar::Vector2f& D
	)
	{
		return LineIntersectSide(AX, AY, BX, BY, C.GetX(), C.GetY(), D.GetX(), D.GetY());
	}

	int NavMesh::LineIntersectSide(
		Liar::NAVDTYPE AX, Liar::NAVDTYPE AY,
		Liar::NAVDTYPE BX, Liar::NAVDTYPE BY,
		Liar::NAVDTYPE CX, Liar::NAVDTYPE CY,
		Liar::NAVDTYPE DX, Liar::NAVDTYPE DY)
	{
		NAVDTYPE fC = (CY - AY) * (AX - BX) - (CX - AX) * (AY - BY);
		NAVDTYPE fD = (DY - AY) * (AX - BX) - (DX - AX) * (AY - BY);

		double val = fC * fD;

		if (val > 0)
		{
			return -1;	// 没交点
		}
		else if (val == 0)
		{
			return 0;	// 在线上
		}
		else
		{
			return 1;	// 相交
		}
	}

	// 获得交点
	bool NavMesh::GetCrossVector2f(const Vector2f& ss, const Vector2f& sd, const Vector2f& ts, const Vector2f& td, NAVDTYPE& x, NAVDTYPE& y)
	{
		Liar::NAVDTYPE ssX = ss.GetX();
		Liar::NAVDTYPE ssY = ss.GetY();

		Liar::NAVDTYPE sdX = sd.GetX();
		Liar::NAVDTYPE sdY = sd.GetY();

		Liar::NAVDTYPE tsX = ts.GetX();
		Liar::NAVDTYPE tsY = ts.GetY();

		Liar::NAVDTYPE tdX = td.GetX();
		Liar::NAVDTYPE tdY = td.GetY();

		NAVDTYPE sa = ssY - sdY;
		NAVDTYPE sb = sdX - ssX;
		NAVDTYPE sc = ssX * sdY - sdX * ssY;

		NAVDTYPE ta = tsY - tdY;
		NAVDTYPE tb = tdX - tsX;
		NAVDTYPE tc = tsX * tdY - tdX * tsY;

		NAVDTYPE d = sa * tb - ta * sb;
		if (d != 0)
		{
			x = (sb*tc - tb * sc) / d;
			y = (sc*ta - tc * sa) / d;
			return true;
		}
		else
		{
			return false;
		}
	}

#endif // FindNearest

	/**
	* 路径经过的网格
	* @return
	*/
	Liar::Cell** NavMesh::GetCellPath(Liar::Uint& numPath)
	{
		Liar::Cell* st = m_closeList[m_closeCount - 1];

		++numPath;
		size_t blockSize = sizeof(Liar::Cell*)*numPath;
		Liar::Cell** pathCell = (Liar::Cell**)malloc(blockSize);
		pathCell[numPath - 1] = st;

		while (st->parent)
		{
#ifdef EditorMod
			AddCrossCell(st);
#endif // EditorMod
			st = st->parent;
			
			++numPath;
			blockSize = sizeof(Liar::Cell*)*numPath;
			pathCell = (Liar::Cell**)realloc(pathCell, blockSize);
			pathCell[numPath - 1] = st;
		}

#ifdef EditorMod
		AddCrossCell(st);
#endif // EditorMod

		return pathCell;
	}

#ifdef EditorMod
	void NavMesh::AddCrossCell(Cell* cell)
	{
		m_crossCount++;
		size_t blockSize = sizeof(Liar::Cell*)*m_crossCount;
		if (m_crossList) m_crossList = (Liar::Cell**)realloc(m_crossList, blockSize);
		else m_crossList = (Liar::Cell**)malloc(blockSize);
		m_crossList[m_crossCount - 1] = cell;
	}

	void NavMesh::DisposeCross()
	{
		if (m_crossList)
		{
			free(m_crossList);
			m_crossList = nullptr;
		}
		m_crossCount = 0;
	}
#endif // !EditorMod

	/*
	*	add point to path
	*/
	Liar::Vector2f** NavMesh::AddPathPoint(const Liar::Vector2f& vec, Liar::Uint& len)
	{
		return AddPathPoint(vec.GetX(), vec.GetY(), len);
	}

	Liar::Vector2f** NavMesh::AddPathPoint(Liar::NAVDTYPE x, Liar::NAVDTYPE y, Liar::Uint& len)
	{
		++len;
		Liar::Vector2f* addPoint = nullptr;
		if (len > m_numPath)
		{
			size_t blockSize = sizeof(Liar::Vector2f*)*len;
			if (m_path) m_path = (Liar::Vector2f**)realloc(m_path, blockSize);
			else m_path = (Liar::Vector2f**)malloc(blockSize);
			m_numPath = len;
			addPoint = (Liar::Vector2f*)malloc(sizeof(Liar::Vector2f));
		}
		else
		{
			addPoint = m_path[len - 1];
		}
		addPoint->Set(x, y);
		m_path[len - 1] = addPoint;
		return m_path;
	}

	/**
	* 找出给定点所在的三角型
	* @param Point
	* @return
	*/
	Liar::Cell* NavMesh::FindClosestCell(const Vector2f& pt, bool rw)
	{
		return FindClosestCell(pt.GetX(), pt.GetY(), rw);
	}

	Liar::Cell* NavMesh::FindClosestCell(Liar::NAVDTYPE x, Liar::NAVDTYPE y, bool rw)
	{

		Liar::Uint numCell = m_map->GetCellCount();
		Liar::Cell** cells = m_map->GetCells();
		for (Liar::Uint i = 0; i < numCell; ++i)
		{
			Liar::Cell* it = cells[i];
			if (it->IsPointIn(x, y, rw))
			{
				return it;
			}
		}
		return nullptr;
	}

	void NavMesh::DisposePath()
	{
		if (m_path)
		{
			for (Liar::Uint i = 0; i < m_numPath; ++i)
			{
				m_path[i]->~Vector2f();
				free(m_path[i]);
				m_path[i] = nullptr;
			}
			free(m_path);
			m_path = nullptr;
		}
		m_numPath = 0;
	}

#ifdef ShareFind
	int NavMesh::PATHSESSIONID = 0;
#endif // !ShareFind

	Liar::Uint NavMesh::PATHMAX = 50;
}
