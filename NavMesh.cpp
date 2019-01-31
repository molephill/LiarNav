#include "NavMesh.h"

namespace Liar
{
#ifdef ShareFind
#ifdef FindNearest
	NavMesh::NavMesh():
		m_cells(nullptr), m_numberCell(0),
		m_openList((Heap*)malloc(sizeof(Heap))), m_closeList(nullptr), m_closeCount(0),
		m_nearstCells(nullptr), m_nearstCount(0)
#else
	NavMesh::NavMesh() :
		m_cells(nullptr), m_numberCell(0),
		m_openList((Heap*)malloc(sizeof(Heap))), m_closeList(nullptr), m_closeCount(0),
#endif // FindNearest
#else
	NavMesh::NavMesh() :
		m_cells(nullptr), m_numberCell(0),
#endif // ShareFind
	{
	}


	NavMesh::~NavMesh()
	{
		m_cells = nullptr;
#if ShareFind
		m_openList->~Heap();
		free(m_openList);
		m_openList = nullptr;
		if (m_closeList)
		{
			free(m_closeList);
			m_closeList = nullptr;
		}

#ifdef EditorMod
		DisposeFindCtr();
#endif // EditorMod

#endif // ShareFind
	}

	void NavMesh::Set()
	{
		m_cells = nullptr;
		m_numberCell = 0;
#ifdef ShareFind
		m_openList = (Heap*)malloc(sizeof(Heap));
		m_closeList = nullptr;
		m_closeCount = 0;

#ifdef FindNearest
		m_nearstCells = nullptr;
		m_nearstCount = 0;
#endif // FindNearest

#ifdef EditorMod
		m_crossCount = 0;
		m_crossList = nullptr;
#endif // EdtorMod

#endif // ShareFind
	}

	// add cells
	void NavMesh::AddCell(Liar::Cell* cell)
	{
		cell->SetIndex(m_numberCell++);
		size_t blockSize = sizeof(Liar::Cell*)*m_numberCell;
		if (m_cells) m_cells = (Liar::Cell**)realloc(m_cells, blockSize);
		else m_cells = (Liar::Cell**)malloc(blockSize);
		m_cells[m_numberCell - 1] = cell;
	}

	// linkCells
	void NavMesh::LinkCells(bool isCW)
	{
		for (Liar::Uint i = 0; i < m_numberCell; ++i)
		{
			for (Liar::Uint j = 0; j < m_numberCell; ++j)
			{
				if(i != j) m_cells[i]->CheckAndLink(*(m_cells[j]));
			}
		}

#ifdef ShareFind
		m_openList->Set(m_numberCell);
#endif
		
	}

#ifdef ShareFind

#ifdef EditorMod
	void NavMesh::DisposeFindCtr()
	{
		if (m_crossList)
		{
			free(m_crossList);
			m_crossList = nullptr;
		}
		m_crossCount = 0;
	}
#endif // EditorMod

	Liar::Vector2f** NavMesh::FindPath(Liar::NAVDTYPE startX, Liar::NAVDTYPE startY, Liar::NAVDTYPE endX, Liar::NAVDTYPE endY, Liar::Uint& outLen, bool rw)
	{
		++NavMesh::PATHSESSIONID;

		Liar::Cell* startCell = FindClosestCell(startX, startY, rw);
#ifndef OutFind
		if (!startCell) return nullptr;
#endif // !OutFind
		Liar::Cell* endCell = FindClosestCell(endX, endY, rw);
		if (!endCell) return nullptr;

#ifdef OutFind
		if (!startCell)
		{
			Liar::Vector2f** out = (Liar::Vector2f**)malloc(sizeof(Vector2f*) * 2);
			out[0]->Set(startX, startY);
			out[1]->Set(endX, endY);
			return out;
		}
#endif // OutFind

		if (startCell == endCell || TestOneLine2D(startX, startY, endX, endY, startCell, endCell))
		{
			outLen = 2;
			Liar::Vector2f** out = (Liar::Vector2f**)malloc(sizeof(Liar::Vector2f*)*outLen);
			out[0]->Set(startX, startY);
			out[1]->Set(endX, endY);

			return out;
		}
		else
		{
			return BuildPath(startCell, startX, startY, endCell, endX, endY, outLen, rw);
		}
	}

	/**
	* 构建路径
	* @param startCell
	* @param startPos
	* @param endCell
	* @param endPos
	* @return Point路径数组
	*/
	Liar::Vector2f** NavMesh::BuildPath(
		Liar::Cell* startCell, const Liar::Vector2f& startPos,
		Liar::Cell* endCell, const Liar::Vector2f& endPos, 
		Liar::Uint& outLen, bool rw)
	{
		return BuildPath(startCell, startPos.GetX(), startPos.GetY(), endCell, endPos.GetX(), endPos.GetY(), outLen, rw);
	}

	Liar::Vector2f** NavMesh::BuildPath(
		Liar::Cell* startCell, Liar::NAVDTYPE startX, Liar::NAVDTYPE startY,
		Liar::Cell* endCell, Liar::NAVDTYPE endX, Liar::NAVDTYPE endY,
		Liar::Uint& outLen, bool rw)
	{
		m_openList->Clear();
		m_openList->Push(endCell);
		endCell->f = 0;
		endCell->h = 0;
		endCell->isOpen = false;
		endCell->parent = nullptr;
		endCell->sessionId = Liar::NavMesh::PATHSESSIONID;

		bool foundPath = false;			//是否找到路径
		Liar::Cell* currNode = nullptr;		//当前节点
		Liar::Cell* adjacentTmp = nullptr;	//当前节点的邻接三角型

		while (m_openList->Size())
		{
			// 1. 把当前节点从开放列表删除, 加入到封闭列表
			currNode = m_openList->Pop();
			++m_closeCount;
			size_t blockSize = sizeof(Liar::Cell*)*m_closeCount;
			if (m_closeList) m_closeList = (Liar::Cell**)realloc(m_closeList, blockSize);
			else m_closeList = (Liar::Cell**)malloc(blockSize);
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

				if (adjacentId < 0) continue; //不能通过
				else adjacentTmp = m_cells[adjacentId];

				if (adjacentTmp)
				{
					if (adjacentTmp->sessionId != NavMesh::PATHSESSIONID)
					{
						// 4. 如果该相邻节点不在开放列表中,则将该节点添加到开放列表中, 
						//    并将该相邻节点的父节点设为当前节点,同时保存该相邻节点的G和F值;
						adjacentTmp->sessionId = NavMesh::PATHSESSIONID;
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
							adjacentTmp = nullptr;
							continue;
						}
					}
				}
			}
		}

		//由网格路径生成Point数组路径
		if (foundPath)
		{
			return GetPath(startX, startY, endX, endY, outLen, rw);
		}
		else
		{
			return nullptr;
		}
	}

	/**
	* 根据经过的三角形返回路径点(下一个拐角点法)
	* @param start
	* @param end
	* @return Point数组
	*/
	Liar::Vector2f** NavMesh::GetPath(const Liar::Vector2f& start, const Liar::Vector2f& end, Liar::Uint& outLen, bool rw)
	{
		return GetPath(start.GetX(), start.GetY(), end.GetX(), end.GetY(), outLen, rw);
	}

	Liar::Vector2f** NavMesh::GetPath(Liar::NAVDTYPE startX, Liar::NAVDTYPE startY, Liar::NAVDTYPE endX, Liar::NAVDTYPE endY, Liar::Uint& outLen, bool rw)
	{
		//经过的三角形
		int cellPathSize = 0;
		Liar::Cell** cellPath = GetCellPath(cellPathSize);

		if (cellPathSize > 0)
		{
			//开始点
			Liar::Vector2f** pathArr = AddPathPoint(nullptr, startX, startY, outLen);
			//起点与终点在同一三角形中
			if (cellPathSize == 1)
			{
				pathArr = AddPathPoint(pathArr, endX, endY, outLen);
			}
			else
			{
				Line2d* lastLineA = (Line2d*)malloc(sizeof(Line2d));
				Line2d* lastLineB = (Line2d*)malloc(sizeof(Line2d));
				lastLineA->Init(*pathArr, *pathArr);
				lastLineB->Init(*pathArr, *pathArr);

				WayPoint* w = (WayPoint*)malloc(sizeof(WayPoint));
				w->Init(cellPath[0], *pathArr);

#ifdef FindNearest
				int preNCount = m_nearstCount++;
				m_nearstCells = (Liar::Cell**)malloc(sizeof(Liar::Cell*)*m_nearstCount);
				m_nearstCells[preNCount] = w->caller;
#endif // FindNearest

				while (!(w->GetPos()->Equals(*end, EPSILON)))
				{
					GetFurthestWayPoint(w, cellPath, cellPathSize, end, lastLineA, lastLineB, rw);
					pre = outLen;
					++outLen;
					pathArr = (Vector2f*)realloc(pathArr, sizeof(Vector2f)*outLen);
					pathArr[pre].SetVector2f(*(w->GetPos()));

#ifdef FindNearest
					preNCount = m_nearstCount++;
					m_nearstCells = (Liar::Cell**)realloc(m_nearstCells, sizeof(Liar::Cell*)*m_nearstCount);
					m_nearstCells[preNCount] = w->caller;
#endif // FindNearest

				}
				w->~WayPoint();
				free(w);
				w = nullptr;

				lastLineA->~Line2d();
				free(lastLineA);
				lastLineA = nullptr;
				lastLineB->~Line2d();
				free(lastLineB);
				lastLineB = nullptr;

#ifdef FindNearest
				FindNearestPath(0, pathArr, outLen);
				for (int i = 0; i < _nearstCount; ++i)
				{
					_nearstCells[i]->checkLinkCount = 0;
				}
				free(_nearstCells);
				_nearstCells = nullptr;
				_nearstCount = 0;
#endif // FindNearest

			}

			free(cellPath);
			free(m_closeList);
			m_closeList = nullptr;

			return pathArr;
			/*return pathArr;*/
		}

	}

	/**
	* 路径经过的网格
	* @return
	*/
	Liar::Cell** NavMesh::GetCellPath(int& len)
	{
		Liar::Cell* st = m_closeList[m_closeCount - 1];
		int pre = len;
		++len;
		Liar::Cell** out = (Liar::Cell**)malloc(sizeof(Liar::Cell*)*len);
		out[pre] = st;

		while (st->parent)
		{
			pre = len;
			++len;
			out = (Liar::Cell**)realloc(out, sizeof(Liar::Cell*)*len);
			out[pre] = st->parent;
			AddCrossCell(st);
			st = st->parent;
		}
		AddCrossCell(st);
		return out;
	}

	void NavMesh::AddCrossCell(Cell* cell)
	{
#ifdef EditorMod
		int preCross = m_crossCount++;
		size_t blockSize = sizeof(Liar::Cell*)*m_crossCount;
		if (m_crossList) m_crossList = (Liar::Cell**)realloc(m_crossList, blockSize);
		else m_crossList = (Liar::Cell**)malloc(blockSize);
		m_crossList[preCross] = cell;
#endif // EditorMod
	}

	/*
	*	add point to path
	*/
	Liar::Vector2f** NavMesh::AddPathPoint(Liar::Vector2f** path, const Liar::Vector2f& vec, Liar::Uint& len)
	{
		return AddPathPoint(path, vec.GetX(), vec.GetY(), len);
	}

	Liar::Vector2f** NavMesh::AddPathPoint(Liar::Vector2f** path, Liar::NAVDTYPE x, Liar::NAVDTYPE y, Liar::Uint& len)
	{
		++len;
		size_t blockSize = sizeof(Liar::Vector2f*)*blockSize;
		if (path) path = (Liar::Vector2f**)realloc(path, blockSize);
		else path = (Liar::Vector2f**)malloc(blockSize);

		Liar::Vector2f* addPoint = (Liar::Vector2f*)malloc(sizeof(Liar::Vector2f));
		addPoint->Set(x, y);
		path[len - 1] = addPoint;

		return path;
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
		for (Liar::Uint i = 0; i < m_numberCell; ++i)
		{
			Liar::Cell* it = m_cells[i];
			if (it->IsPointIn(x, y, rw)) return it;
		}
		return nullptr;
	}

	// 检测两点间是否能直达
	bool NavMesh::TestOneLine2D(const Vector2f& start, const Vector2f& end, Cell* startCell, Cell* endCell)
	{
		return TestOneLine2D(start.GetX(), start.GetY(), end.GetX(), end.GetY(), startCell, endCell);
	}

	bool NavMesh::TestOneLine2D(
		Liar::NAVDTYPE startX, Liar::NAVDTYPE startY,
		Liar::NAVDTYPE endX, Liar::NAVDTYPE endY,
		Cell* startCell, Cell* endCell
	)
	{
		// 记录已经找过的路径
		Cell** crossCells = (Cell**)malloc(sizeof(Cell*));
		int crossCount = 1;
		int pre = 0;
		Liar::Cell* cell = endCell;
		crossCells[0] = cell;

		bool findCross = true;
		while (cell && cell != startCell)
		{
			cell = GetCrossCell(startX, startY, endX, endY, *cell, crossCells, crossCount, findCross);
			findCross = false;
			if (cell)
			{
				pre = crossCount++;
				crossCells = (Liar::Cell**)realloc(crossCells, sizeof(Liar::Cell*)*crossCount);
				crossCells[pre] = cell;
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
		int tmpIndex = 0;

		int adjacentId = 0;
		Cell* sourceAdjacentTmp = nullptr;
		int startCrossCount = 0;
		Cell* outTmp = nullptr;
		for (Liar::Uint i = 0; i < 3; ++i)
		{
			adjacentId = testCell.GetLink(i);
			if (adjacentId < 0)
			{
				continue;
			}
			else
			{
				sourceAdjacentTmp = m_cells[adjacentId];

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
						//return sourceAdjacentTmp;
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
			Liar::Line2d& testline = sourceAdjacentTmp.GetSide(j);
			Liar::Vector2f& pa = testline.GetPointA();
			Liar::Vector2f& pb = testline.GetPointB();
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

	int NavMesh::PATHSESSIONID = 0;
#endif // ShareFind
}
