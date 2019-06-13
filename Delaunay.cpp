#include "Delaunay.h"

#ifdef EditorMod
#include <string>
#endif // EditorMod


namespace Liar
{
	void Delaunay::Dispose()
	{
		Liar::Uint i = 0;

		if (m_line2ds)
		{
			for (i = 0; i < m_totalLines; ++i)
			{
				m_line2ds[i]->~Line2d();
				free(m_line2ds[i]);
				m_line2ds[i] = nullptr;
			}
			free(m_line2ds);
			m_line2ds = nullptr;
			m_totalLines = 0;
		}

		if (m_circle)
		{
			m_circle->~Circle();
			free(m_circle);
			m_circle = nullptr;
		}

		if (m_tmpRange)
		{
			free(m_tmpRange);
			m_tmpRange = nullptr;
		}

		if (m_findDTPoints)
		{
			free(m_findDTPoints);
			m_findDTPoints = nullptr;
		}

		if (m_edge)
		{
			m_edge->~Line2d();
			free(m_edge);
			m_edge = nullptr;
		}

		if (m_interscetVector)
		{
			m_interscetVector->~Vector2f();
			free(m_interscetVector);
			m_interscetVector = nullptr;
		}

#ifdef UNION_POLYGON
		if (m_nodes1)
		{
			for (i = 0; i < m_numberNode1; ++i)
			{
				m_nodes1[i]->~Node();
				m_nodes1[i] = nullptr;
			}
			free(m_nodes1);
			m_nodes1 = nullptr;
			m_numberNode1 = 0;
		}

		if (m_nodes2)
		{
			for (i = 0; i < m_numberNode2; ++i)
			{
				m_nodes2[i]->~Node();
				m_nodes2[i] = nullptr;
			}

			free(m_nodes2);
			m_nodes2 = nullptr;
			m_numberNode2 = 0;
		}
#endif // UNION_POLYGON

#ifdef INFLATE
		if (m_inflates)
		{
			for (i = 0; i < m_allNumberInflate; ++i)
			{
				m_inflates[i]->~Vector2f();
				free(m_inflates[i]);
				m_inflates[i] = nullptr;
			}
			free(m_inflates);
			m_inflates = nullptr;
		}
#endif // INFLATE

#ifdef EditorMod
		PrintBuildErrorLogs();
#endif // EditorMod


	}

#ifdef INFLATE
	void Delaunay::AddVertex(const Liar::Vector2f& v)
	{
		AddVertex(v.GetX(), v.GetY());
	}

	void Delaunay::AddVertex(Liar::NAVDTYPE x, Liar::NAVDTYPE y)
	{
		++m_curNumberInflate;
		Liar::Vector2f* vec = nullptr;
		if (m_curNumberInflate >= m_allNumberInflate)
		{
			++m_allNumberInflate;
			size_t blockSize = sizeof(Liar::Vector2f*)*m_allNumberInflate;
			if (m_inflates) m_inflates = (Liar::Vector2f**)realloc(m_inflates, blockSize);
			else m_inflates = (Liar::Vector2f**)malloc(blockSize);
			vec = (Liar::Vector2f*)malloc(sizeof(Liar::Vector2f));
			m_inflates[m_curNumberInflate - 1] = vec;
		}
		else
		{
			vec = m_inflates[m_curNumberInflate - 1];
		}
		vec->Set(x, y);
	}

	void Delaunay::Inflate(Liar::Map& map, Liar::Polygon& polygon, bool wall, Liar::NAVDTYPE dis)
	{
		Liar::Vector2f ab;
		Liar::Vector2f ac;

		Liar::Uint nextIndex = 0;
		Liar::Uint previousIndex = 0;

		for (Liar::Uint i = 0; i < m_curNumberInflate; ++i)
		{
			Liar::Vector2f* current = m_inflates[i];
			nextIndex = (i + 1) % m_curNumberInflate;
			Liar::Vector2f* next = m_inflates[nextIndex];
			previousIndex = (i == 0) ? m_curNumberInflate - 1 : i - 1;
			Liar::Vector2f* previous = m_inflates[previousIndex];
			if (current->Equals(*next) || current->Equals(*previous))
			{
				Liar::Delaunay::AddVertex(map, polygon, *current);
			}
			else
			{
				ab.Set(*next);
				ab -= (*current);
				ab.Normalize();

				ac.Set(*previous);
				ac -= (*current);
				ac.Normalize();

				ab += ac;
				ab.Normalize();
				if(wall) ab *= (!Liar::Delaunay::PointIsConcave(i) ? -dis : dis);
				else ab *= (!Liar::Delaunay::PointIsConcave(i) ? dis : -dis);

				Liar::Delaunay::AddVertex(map, polygon, current->GetX() + ab.GetX(), current->GetY() + ab.GetY());
			}
		}
		Liar::Delaunay::m_curNumberInflate = 0;
	}

	void Delaunay::ResetInFlate()
	{
		Liar::Delaunay::m_curNumberInflate = 0;
	}

	bool Delaunay::PointIsConcave(Liar::Uint i)
	{
		Liar::Vector2f* current = m_inflates[i];
		Liar::Uint nextIndex = (i + 1) % m_curNumberInflate;
		Liar::Vector2f* next = m_inflates[nextIndex];
		Liar::Uint previousIndex = i == 0 ? m_curNumberInflate - 1 : i - 1;
		Liar::Vector2f* previous = m_inflates[previousIndex];

		Liar::NAVDTYPE lx = current->GetX() - previous->GetX();
		Liar::NAVDTYPE ly = current->GetY() - previous->GetY();

		Liar::NAVDTYPE rx = next->GetX() - current->GetX();
		Liar::NAVDTYPE ry = next->GetY() - current->GetY();

		Liar::NAVDTYPE cross = lx * ry - ly * rx;
		return cross > 0;
	}

	void Delaunay::AddVertexToMap(Liar::Map&, Liar::Polygon&, const Liar::Vector2f& v)
	{
		AddVertex(v);
	}

	void Delaunay::AddVertexToMap(Liar::Map&, Liar::Polygon&, Liar::NAVDTYPE x, Liar::NAVDTYPE y)
	{
		AddVertex(x, y);
	}
#else
	void Delaunay::AddVertexToMap(Liar::Map& map, Liar::Polygon& polygon, const Liar::Vector2f& v)
	{
		AddVertex(map, polygon, v);
	}

	void Delaunay::AddVertexToMap(Liar::Map& map, Liar::Polygon& polygon, Liar::NAVDTYPE x, Liar::NAVDTYPE y)
	{
		AddVertex(map, polygon, x, y);
	}
#endif // INFLATE

	void Delaunay::AddVertex(Liar::Map& map, Liar::Polygon& polygon, Liar::NAVDTYPE x, Liar::NAVDTYPE y)
	{
		Liar::Uint pointIndex = map.AddVertex(x, y);
		polygon.AddPointIndex(pointIndex);
	}

	void Delaunay::AddVertex(Liar::Map& map, Liar::Polygon& polygon, const Liar::Vector2f& vf)
	{
		Liar::Delaunay::AddVertex(map, polygon, vf.GetX(), vf.GetY());
	}

	Liar::Uint Delaunay::Set(Liar::Map& map, bool isCW, Liar::Uint boxIndex)
	{
		// ================== init ====================
		if (!m_circle)
		{
			m_circle = (Liar::Delaunay::Circle*)malloc(sizeof(Liar::Delaunay::Circle));
			m_circle->Init();
		}

		if (!m_tmpRange) m_tmpRange = (Liar::NAVDTYPE*)malloc(sizeof(Liar::NAVDTYPE) * 4);

		if (!m_edge)
		{
			m_edge = (Liar::Line2d*)malloc(sizeof(Liar::Line2d));
			m_edge->Set(nullptr);
		}

		if (!m_interscetVector)
		{
			m_interscetVector = (Liar::Vector2f*)malloc(sizeof(Liar::Vector2f));
			m_interscetVector->Set(Liar::ZERO, Liar::ZERO);
		}
		// ============================================

		m_curNumLines = 0;

#ifdef UNION_POLYGON
		map.UnionAll(isCW);
#endif // UNION_POLYGON

		BuildEdges(map);
		BuildTrianges(map, isCW, boxIndex);
		Liar::Uint cellCount = map.NavMeshLinkCells(isCW);

#ifdef EditorMod
		if (cellCount <= 0 && map.GetNumPoints() > 0) SetBuildErrClock(Liar::Delaunay::mapId);
#endif // EditorMod

		return cellCount;
	}

	Liar::Line2d** Delaunay::BuildTrianges(
		const Liar::Vector2f& p1, const Liar::Vector2f& p2, Liar::Line2d** lineV, 
		Liar::Int& tmpLineCount, Liar::Map& map, Liar::Uint paIndex, Liar::Uint pbIndex)
	{
		int index = FindLinePos(p1, p2, m_line2ds, m_curNumLines);
		if (index < 0)
		{
			index = FindLinePos(p1, p2, lineV, tmpLineCount);
			if (index > -1)
			{
				lineV = RemovePosLine(lineV, tmpLineCount, index);
			}
			else
			{
				++tmpLineCount;
				size_t blockSize = sizeof(Liar::Line2d*)*tmpLineCount;
				if (lineV) lineV = (Liar::Line2d**)realloc(lineV, blockSize);
				else lineV = (Liar::Line2d**)malloc(blockSize);
				Liar::Line2d* line13 = (Liar::Line2d*)malloc(sizeof(Liar::Line2d));
				line13->Set(&map, paIndex, pbIndex);
				lineV[tmpLineCount - 1] = line13;
			}
		}
		return lineV;
	}

	void Delaunay::BuildTrianges(Liar::Map& map, bool isCW, Liar::Uint boxIndex)
	{
		Liar::Line2d& initEdge = GetBoundEdage(map, isCW, boxIndex);

		int idx = 0;
		int tmpLineCount = 1;

		Liar::Line2d** lineV = (Liar::Line2d**)malloc(sizeof(Liar::Line2d*));
		Liar::Line2d* firstLine = (Liar::Line2d*)malloc(sizeof(Liar::Line2d));
		firstLine->Set(initEdge);
		lineV[0] = firstLine;

		m_edge->Set(initEdge);

		m_interscetVector->Set(Liar::ZERO, Liar::ZERO);

#ifdef PRINT
		int index = 0;
		const char* path = "C:/Users/Administrator/Desktop/new/build_trianges.txt";
		std::ofstream outfile(path, std::ios::ate);
#endif // PRINT

		do
		{
			idx = tmpLineCount - 1;

			m_edge->Set(*lineV[idx]);
			lineV = RemovePosLine(lineV, tmpLineCount, idx);

			Liar::Int p3Index = FindDT(map, *m_edge, *m_interscetVector, isCW);

			if (p3Index < 0) continue;

			Liar::Uint pointAIndex = m_edge->GetPointAIndex();
			Liar::Uint pointBIndex = m_edge->GetPointBIndex();

			Liar::Vector2f* p3 = map.GetVertex(p3Index);
			Liar::Vector2f& pa = m_edge->GetPointA();
			Liar::Vector2f& pb = m_edge->GetPointB();

#ifdef PRINT
			outfile << "index:" << index << ",tmplen:" << tmpLineCount << ",p3:{" << p3->GetX() << "," << p3->GetY() << "},pa:{" << pa.GetX() << "," << pa.GetY() << "},pb:{" << pb.GetX() << "," << pb.GetY() << "}\n";
			outfile << "	tmpv:\n";
			for (int tmpi = 0; tmpi < tmpLineCount; ++tmpi)
			{
				Line2d* tmpTT = lineV[tmpi];
				outfile << "	tmp_index:" << tmpi << ",tmp_pa:{" << tmpTT->GetPointA().GetX() << "," << tmpTT->GetPointA().GetY() << "},tmp_pb:{" << tmpTT->GetPointB().GetX() << "," << tmpTT->GetPointB().GetY() << "}\n";
			}

			index++;
#endif // PRINT

			Liar::Cell* addTri = (Liar::Cell*)malloc(sizeof(Liar::Cell));
			if(isCW) addTri->Set(&map, pointAIndex, pointBIndex, p3Index);
			else addTri->Set(&map, p3Index, pointBIndex, pointAIndex);
			Liar::Uint curTri = map.AddNavMeshCell(addTri);

			if (curTri > Liar::DEAD_LOOP_MAX)
			{

#ifdef EditorMod
				SetBuildErrLog(Liar::Delaunay::mapId, true, false);
#endif // EditorMod

				break;
			}

			lineV = BuildTrianges(pa, *p3, lineV, tmpLineCount, map, pointAIndex, p3Index);
			lineV = BuildTrianges(*p3, pb, lineV, tmpLineCount, map, p3Index, pointBIndex);

		} while (tmpLineCount > 0);

#ifdef PRINT
		outfile.close();
#endif // PRINT

		free(lineV);
		lineV = nullptr;
	}

	void Delaunay::BuildEdges(Liar::Map& map)
	{
		Liar::Uint numPolygons = map.GetNumPolygon();
		for (Liar::Uint i = 0; i < numPolygons; ++i)
		{
			Liar::Polygon* polygon = map.GetPolygon(i);
			if (polygon)
			{

#ifndef UNION_POLYGON
				polygon->Rectangle();
#endif // !UNION_POLYGON

				map.CalcBound(*polygon); // calcBound
				Liar::Uint numPoints = polygon->GetNumPoints();

				Liar::Uint pre = m_curNumLines;
				m_curNumLines += numPoints;

				Liar::Uint resetNum = m_curNumLines > m_totalLines ? m_totalLines : m_curNumLines;
				// reset old data
				for (Liar::Uint resetMap = pre; resetMap < resetNum; ++resetMap) m_line2ds[resetMap]->Set(&map);

				if (m_curNumLines > m_totalLines)
				{
					Liar::Uint preTotal = m_totalLines;
					m_totalLines = m_curNumLines;
					size_t blockSize = sizeof(Liar::Line2d*)*m_totalLines;
					if (m_line2ds) m_line2ds = (Liar::Line2d**)realloc(m_line2ds, blockSize);
					else m_line2ds = (Liar::Line2d**)malloc(blockSize);

					for (Liar::Uint initMap = preTotal; initMap < m_totalLines; ++initMap)
					{
						Liar::Line2d* tmpLine = (Liar::Line2d*)malloc(sizeof(Liar::Line2d));
						tmpLine->Set(&map);
						m_line2ds[initMap] = tmpLine;
					}
				}

				Liar::Uint p1 = polygon->GetPointIndex(0);
				Liar::Uint p2 = 0;
				for (Liar::Uint j = 1; j < numPoints; ++j)
				{
					p2 = polygon->GetPointIndex(j);
					m_line2ds[pre + j - 1]->Set(p1, p2);
					p1 = p2;
				}
				p1 = polygon->GetPointIndex(numPoints - 1);
				p2 = polygon->GetPointIndex(0);
				m_line2ds[pre + numPoints - 1]->Set(p1, p2);
			}
		}

#ifdef PRINT
		PrintEdges(map);
#endif // PRINT
	}

#ifdef EditorMod
	void Delaunay::PrintEdges(const Liar::Map& map)
	{
		const char* path = "C:/Users/Administrator/Desktop/new/polygon_edges.txt";
		std::ofstream outfile(path, std::ios::ate);
		for (Liar::Uint i = 0; i < m_curNumLines; ++i)
		{
			Liar::Line2d* tmpLine = m_line2ds[i];
			Liar::Vector2f& pa = tmpLine->GetPointA();
			Liar::Vector2f& pb = tmpLine->GetPointB();
			outfile << i << ":" << "A:{" << pa.GetX() << "," << pa.GetY() << "},B:{" << pb.GetX() << "," << pb.GetY() << "}\n";
		}
		outfile.close();
	}

	Liar::Delaunay::BuildErrorLog* Delaunay::GetBuildErrLog(Liar::Uint bid)
	{
		for (Liar::Uint i = 0; i < m_numberBuildErrLogs; ++i)
		{
			if (*(m_buildErrLogs[i]) == bid) return m_buildErrLogs[i];
		}
		return nullptr;
	}

	void Delaunay::SetBuildErrClock(Liar::Uint bid, bool clock)
	{
		Liar::Delaunay::BuildErrorLog* log = GetBuildErrLog(bid);
		if (log) log->clockErr = clock;
	}

	Liar::Delaunay::BuildErrorLog* Delaunay::SetBuildErrLog(Liar::Uint bid, bool loop, bool clock)
	{
		Liar::Delaunay::BuildErrorLog* log = GetBuildErrLog(bid);
		
		if (!log)
		{
			++m_numberBuildErrLogs;
			size_t blockSize = sizeof(Liar::Delaunay::BuildErrorLog*)*m_numberBuildErrLogs;
			if (m_buildErrLogs) m_buildErrLogs = (Liar::Delaunay::BuildErrorLog**)realloc(m_buildErrLogs, blockSize);
			else m_buildErrLogs = (Liar::Delaunay::BuildErrorLog**)malloc(blockSize);
			log = (Liar::Delaunay::BuildErrorLog*)malloc(sizeof(Liar::Delaunay::m_buildErrLogs));
			m_buildErrLogs[m_numberBuildErrLogs - 1] = log;
		}
		else
		{
			log->Set(bid, loop, clock);
		}
		return log;
	}

	void Delaunay::PrintBuildErrorLogs()
	{
		if (m_numberBuildErrLogs > 0)
		{
			const char* path = "BuildErrorLogs.txt";
			std::ofstream outfile(path, std::ios::ate);
			for (Liar::Uint i = 0; i < m_numberBuildErrLogs; ++i)
			{
				Liar::Delaunay::BuildErrorLog* log = m_buildErrLogs[i];
				std::cout << "map_bid: " << log->bid;
				if (log->loopDead) std::cout << " has overflower";
				if (log->clockErr) std::cout << " has diffrent clock wise";
				std::cout << "\n\n";
			}
		}
	}
#endif // EditorMod

	Liar::Line2d& Delaunay::GetBoundEdage(Liar::Map& map, bool isCW, Liar::Uint boundIndex)
	{
		Liar::Uint numPolygons = map.GetNumPolygon();
		Liar::Uint numPoints = 0;
		Liar::Uint boundEdgeNum = map.GetPolygon(boundIndex)->GetNumPoints();

		Liar::Line2d* initEdge = m_line2ds[0];

		bool loopSign = false;
		Liar::Uint loopIdx = 0;
		Liar::Vector2f* cit = nullptr;
		Liar::Uint pointIndex = 0;

		do
		{
			loopSign = false;
			++loopIdx;

			for (Liar::Uint i = 0; i < numPolygons; ++i)
			{
				Liar::Polygon* polygon = map.GetPolygon(i);
				numPoints = polygon->GetNumPoints();
				for (Liar::Uint j = 0; j < numPoints; ++j)
				{
					pointIndex = polygon->GetPointIndex(j);
					cit = polygon->GetVertex(pointIndex);
					Liar::Vector2f& pa = initEdge->GetPointA();
					Liar::Vector2f& pb = initEdge->GetPointB();
					if (cit->Equals(pa) || cit->Equals(pb)) continue;
					if (initEdge->ClassifyPoint(*cit, isCW) == Liar::PointClassification::ON_LINE)
					{
						loopSign = true;
						initEdge = m_line2ds[loopIdx];
						break;
					}
				}
			}
		} while (loopSign && loopIdx<boundEdgeNum - 1);

		return *initEdge;
	}

	Liar::Int Delaunay::FindDT(Liar::Map& map, const Liar::Line2d& line, Liar::Vector2f& interscetVector, bool isCW)
	{
		Liar::Uint allPointCount = 0;

		Liar::Uint numPoints = map.GetNumPoints();
		size_t blockSize = 0;
		for (Liar::Uint i = 0; i < numPoints; ++i)
		{
			Liar::Vector2f* it = map.GetVertex(i);
			if (IsVisiblePointOfLine(*it, line, interscetVector, isCW))
			{
				++allPointCount;
				if (allPointCount >= m_numFindDTPoints)
				{
					blockSize = sizeof(Liar::Uint)*allPointCount;
					if (m_findDTPoints) m_findDTPoints = (Liar::Uint*)realloc(m_findDTPoints, blockSize);
					else m_findDTPoints = (Liar::Uint*)malloc(blockSize);
					m_numFindDTPoints = allPointCount;
				}
				m_findDTPoints[allPointCount - 1] = i;
			}
		}

		if (allPointCount <= 0) return -1;

		Liar::Vector2f& p1 = line.GetPointA();
		Liar::Vector2f& p2 = line.GetPointB();

		Liar::Uint p3Index = m_findDTPoints[0];
		Liar::Vector2f* p3 = map.GetVertex(p3Index);

		bool loopSign = false;
		do
		{
			loopSign = false;
			CircumCircle(p1, p2, *p3, *m_circle);
			CircleBounds(*m_circle, m_tmpRange);

			NAVDTYPE angle132 = abs(LineAngle(p1, *p3, p2));
			for (Liar::Uint i = 0; i < allPointCount; ++i)
			{
				Liar::Uint tmpP3Index = m_findDTPoints[i];
				Liar::Vector2f* vec = map.GetVertex(tmpP3Index);
				if (vec->Equals(p1) || vec->Equals(p2) || vec->Equals(*p3)) continue;

				Liar::NAVDTYPE x = vec->GetX();
				Liar::NAVDTYPE y = vec->GetY();
				Liar::NAVDTYPE minx = m_tmpRange[0];
				Liar::NAVDTYPE maxx = m_tmpRange[2];
				Liar::NAVDTYPE miny = m_tmpRange[1];
				Liar::NAVDTYPE maxy = m_tmpRange[3];


				if (x < minx || x > maxx || y < miny || y > maxy) continue;


				Liar::NAVDTYPE a1 = abs(LineAngle(p1, *vec, p2));
				if (a1 > angle132)
				{
					p3Index = tmpP3Index;
					p3 = vec;
					loopSign = true;
					break;
				}
			}
		} while (loopSign);

		return p3Index;
	}

	bool Delaunay::IsVisiblePointOfLine(const Vector2f& vec, const Line2d& line, Liar::Vector2f& interscetVector, bool rw)
	{
		Liar::Vector2f& pa = line.GetPointA();
		Liar::Vector2f& pb = line.GetPointB();

		if (vec.Equals(pa) || vec.Equals(pb)) return false;
		if (line.ClassifyPoint(vec,rw) != Liar::PointClassification::RIGHT_SIDE) return false;
		if (IsVisibleIn2Point(pa, vec, interscetVector) == false) return false;
		if (IsVisibleIn2Point(pb, vec, interscetVector) == false) return false;

		return true;
	}

	bool Delaunay::IsVisibleIn2Point(const Vector2f& pa, const Vector2f& pb, Liar::Vector2f& interscetVector)
	{
		for (Liar::Uint i = 0; i < m_curNumLines; ++i)
		{
			Line2d* limeTmp = m_line2ds[i];
			if (Liar::Delaunay::Intersection(pa, pb, *limeTmp, &interscetVector) == Liar::LineClassification::SEGMENTS_INTERSECT)
			{
				if (!pa.Equals(interscetVector) && !pb.Equals(interscetVector))
				{
					return false;
				}
			}
		}
		return true;
	}

	Liar::LineClassification Delaunay::Intersection(
		const Liar::Line2d& line0,
		const Liar::Line2d& line, Liar::Vector2f* pIntersectPoint)
	{
		return Liar::Delaunay::Intersection(line0.GetPointA(), line0.GetPointB(), line.GetPointA(), line.GetPointB(), pIntersectPoint);
	}

	Liar::LineClassification Delaunay::Intersection(
		const Liar::Vector2f& pointA, const Liar::Vector2f& pointB,
		const Liar::Line2d& line, Liar::Vector2f* pIntersectPoint)
	{
		return Liar::Delaunay::Intersection(pointA, pointB, line.GetPointA(), line.GetPointB(), pIntersectPoint);
	}

	Liar::LineClassification Delaunay::Intersection(
		const Liar::Vector2f& pointA, const Liar::Vector2f& pointB,
		const Liar::Vector2f& otherPointA, const Liar::Vector2f& otherPointB,
		Liar::Vector2f* pIntersectPoint)
	{
		Liar::EPSILONTYPE pointAX = pointA.GetX();
		Liar::EPSILONTYPE pointAY = pointA.GetY();
		Liar::EPSILONTYPE pointBX = pointB.GetX();
		Liar::EPSILONTYPE pointBY = pointB.GetY();
		Liar::EPSILONTYPE otherPointAX = otherPointA.GetX();
		Liar::EPSILONTYPE otherPointAY = otherPointA.GetY();
		Liar::EPSILONTYPE otherPointBX = otherPointB.GetX();
		Liar::EPSILONTYPE ohterPointBY = otherPointB.GetY();

		Liar::EPSILONTYPE denom =
			(ohterPointBY - otherPointAY)*(pointBX - pointAX)
			-
			(otherPointBX - otherPointAX)*(pointBY - pointAY);

		Liar::EPSILONTYPE u0 =
			(otherPointBX - otherPointAX)*(pointAY - otherPointAY)
			-
			(ohterPointBY - otherPointAY)*(pointAX - otherPointAX);
		Liar::EPSILONTYPE u1 =
			(otherPointAX - pointAX)*(pointBY - pointAY)
			-
			(otherPointAY - pointAY)*(pointBX - pointAX);

		if (denom == 0.0)
		{
			if (u0 == 0.0 && u1 == 0.0) return Liar::LineClassification::COLLINEAR;
			else return Liar::LineClassification::PARALELL;
		}
		else
		{
			//check if they intersect
			u0 = u0 / denom;
			u1 = u1 / denom;

			Liar::EPSILONTYPE x = pointAX + u0 * (pointBX - pointAX);
			Liar::EPSILONTYPE y = pointAY + u0 * (pointBY - pointAY);

			if (pIntersectPoint) pIntersectPoint->Set(x, y);

			if ((u0 >= 0.0) && (u0 <= 1.0) && (u1 >= 0.0) && (u1 <= 1.0))
			{
				return Liar::LineClassification::SEGMENTS_INTERSECT;
			}
			else if (u1 >= 0.0 && u1 <= 1.0)
			{
				return Liar::LineClassification::A_BISECTS_B;
			}
			else if (u0 >= 0.0 && u0 <= 1.0)
			{
				return Liar::LineClassification::B_BISECTS_A;
			}
			return Liar::LineClassification::LINES_INTERSECT;
		}
	}

	Liar::Line2d** Delaunay::RemovePosLine(Liar::Line2d** lv, int& lvlen, int pos)
	{
		int tmpLen = lvlen;
		for (int i = 0; i < tmpLen; ++i)
		{
			if (i > pos)
			{
				lv[i - 1] = lv[i];
			}
			else
			{
				if (i == pos)
				{
					lv[i]->~Line2d();
					free(lv[i]);
					lv[i] = nullptr;
					--lvlen;
				}
			}
		}
		return lv;
	}

	void Delaunay::CircumCircle(const Liar::Vector2f& p1, const Liar::Vector2f& p2, const Liar::Vector2f& p3, Liar::Delaunay::Circle& cir)
	{
		Liar::NAVDTYPE p1y = p1.GetY();
		Liar::NAVDTYPE p2y = p2.GetY();
		Liar::NAVDTYPE p3y = p3.GetY();

		if (abs(p1y - p2y) < Liar::EPSILON && abs(p2y - p3y) < Liar::EPSILON)
		{
			return;
		}

		Liar::NAVDTYPE m1, m2, mx1, mx2, my1, my2;
		Liar::NAVDTYPE dx, dy, rsqr;
		Liar::NAVDTYPE xc, yc, r;

		Liar::NAVDTYPE p1x = p1.GetX();
		Liar::NAVDTYPE p2x = p2.GetX();
		Liar::NAVDTYPE p3x = p3.GetX();

		m1 = -(p2x - p1x) / (p2y - p1y);
		m2 = -(p3x - p2x) / (p3y - p2y);
		mx1 = (p1x + p2x) / 2.0f;
		mx2 = (p2x + p3x) / 2.0f;
		my1 = (p1y + p2y) / 2.0f;
		my2 = (p2y + p3y) / 2.0f;

		if (abs(p2y - p1y) < EPSILON) {
			xc = (p2x + p1x) / 2.0f;
			yc = m2 * (xc - mx2) + my2;
		}
		else if (abs(p3y - p2y) < EPSILON) {
			xc = (p3x + p2x) / 2.0f;
			yc = m1 * (xc - mx1) + my1;
		}
		else {
			xc = (m1 * mx1 - m2 * mx2 + my2 - my1) / (m1 - m2);
			yc = m1 * (xc - mx1) + my1;
		}

		dx = p2x - xc;
		dy = p2y - yc;
		rsqr = dx * dx + dy * dy;
		r = sqrtf(rsqr);

		cir.center->Set(xc, yc);
		cir.r = r;
	}

	void Delaunay::CircleBounds(const Liar::Delaunay::Circle& cir, Liar::NAVDTYPE* fv)
	{
		NAVDTYPE lx = cir.center->GetX() - cir.r;
		NAVDTYPE ty = cir.center->GetY() - cir.r;
		NAVDTYPE rx = cir.center->GetX() + cir.r;
		NAVDTYPE by = cir.center->GetY() + cir.r;

		fv[0] = lx;
		fv[1] = ty;
		fv[2] = rx;
		fv[3] = by;
	}

	Liar::NAVDTYPE Delaunay::LineAngle(const Vector2f& s, const Vector2f& o, const Vector2f& e)
	{
		Liar::NAVDTYPE sx = s.GetX();
		Liar::NAVDTYPE sy = s.GetY();
		Liar::NAVDTYPE ox = o.GetX();
		Liar::NAVDTYPE oy = o.GetY();
		Liar::NAVDTYPE ex = e.GetX();
		Liar::NAVDTYPE ey = e.GetY();

		NAVDTYPE cosfi = Liar::ZERO, fi = 0.0f, norm = Liar::ZERO;
		NAVDTYPE dsx = sx - ox;
		NAVDTYPE dsy = sy - oy;
		NAVDTYPE dex = ex - ox;
		NAVDTYPE dey = ey - oy;

		cosfi = dsx * dex + dsy * dey;
		norm = (dsx*dsx + dsy * dsy) * (dex*dex + dey * dey);
		cosfi /= sqrt(norm);

		if (cosfi >= 1.0f) return 0.0;
		if (cosfi <= -1.0f) return -3.1415926f;

		fi = acosf(cosfi);
		if (dsx*dey - dsy * dex > 0) return fi;
		return -fi;
	}

	/*
	* find line pos in edges
	*/
	Liar::Int Delaunay::FindLinePos(const Liar::Vector2f& pa, const Liar::Vector2f& pb, Liar::Line2d** lines, Liar::Uint size)
	{
		for (Liar::Uint i = 0; i < size; ++i)
		{
			if (lines[i]->Equals(pa, pb)) return i;
		}
		return -1;
	}

	Liar::Line2d** Liar::Delaunay::m_line2ds = nullptr;
	Liar::Uint Liar::Delaunay::m_totalLines = 0;
	Liar::Uint Liar::Delaunay::m_curNumLines = 0;
	Liar::Delaunay::Circle* Liar::Delaunay::m_circle = nullptr;
	Liar::NAVDTYPE* Liar::Delaunay::m_tmpRange = nullptr;
	Liar::Uint* Liar::Delaunay::m_findDTPoints = nullptr;
	Liar::Uint Liar::Delaunay::m_numFindDTPoints = 0;
	Liar::Line2d* Liar::Delaunay::m_edge = nullptr;
	Liar::Vector2f* Liar::Delaunay::m_interscetVector = nullptr;
#ifdef EditorMod
	Liar::Int Liar::Delaunay::mapId = 0;
	Liar::Delaunay::BuildErrorLog** Liar::Delaunay::m_buildErrLogs = nullptr;
	Liar::Uint Liar::Delaunay::m_numberBuildErrLogs = 0;
#endif // EditorMod

#ifdef INFLATE
	Liar::Vector2f** Liar::Delaunay::m_inflates = nullptr;
	Liar::Uint Liar::Delaunay::m_allNumberInflate = 0;
	Liar::Uint Liar::Delaunay::m_curNumberInflate = 0;
#endif // INFLATE


#ifdef UNION_POLYGON
	void Delaunay::ExpandNodes(Liar::Uint type, Liar::Uint num)
	{
		Liar::Uint curNum = type == 1 ? Liar::Delaunay::m_numberNode1 : Liar::Delaunay::m_numberNode2;
		if (curNum < num)
		{
			size_t blockSize = sizeof(Liar::Delaunay::Node*)*num;
			Liar::Delaunay::Node** nodes = nullptr;
			if (type == 1)
			{
				if (Liar::Delaunay::m_nodes1) Liar::Delaunay::m_nodes1 = (Liar::Delaunay::Node**)realloc(Liar::Delaunay::m_nodes2, blockSize);
				else Liar::Delaunay::m_nodes1 = (Liar::Delaunay::Node**)malloc(blockSize);
				nodes = Liar::Delaunay::m_nodes1;
			}
			else
			{
				if (Liar::Delaunay::m_nodes2) Liar::Delaunay::m_nodes2 = (Liar::Delaunay::Node**)realloc(Liar::Delaunay::m_nodes2, blockSize);
				else Liar::Delaunay::m_nodes2 = (Liar::Delaunay::Node**)malloc(blockSize);
				nodes = Liar::Delaunay::m_nodes2;
			}

			for (Liar::Uint i = curNum; i < num; ++i)
			{
				Liar::Delaunay::Node* node = (Liar::Delaunay::Node*)malloc(sizeof(Liar::Delaunay::Node));
				node->Set(0, false, false);
				nodes[i] = node;
			}

			if (type == 1) Liar::Delaunay::m_numberNode1 = num;
			else Liar::Delaunay::m_numberNode2 = num;
		}
	}

	bool Delaunay::Expand(Liar::Uint num1, Liar::Uint num2)
	{
		if ((num1 >= num2) && (Liar::Delaunay::m_numberNode1 >= Liar::Delaunay::m_numberNode2))
		{
			ExpandNodes(1, num1);
			ExpandNodes(2, num2);
			return false;
		}
		else
		{
			ExpandNodes(2, num1);
			ExpandNodes(1, num2);
			return true;
		}
	}

	Liar::Uint Delaunay::UnionPolygons(Liar::Map& map, const Liar::Polygon& p0, const Liar::Polygon& p, bool rw)
	{
		Liar::NAVDTYPE* sr = p0.GetRect();
		Liar::NAVDTYPE* pr = p.GetRect();

		Liar::Uint unionNum = 0;

		if (CheckCross(sr, pr))
		{
			Liar::Uint num = p0.GetNumPoints();
			Liar::Uint pnum = p.GetNumPoints();

			bool revert = Expand(num, pnum);

			Liar::Delaunay::Node** cv0 = Liar::Delaunay::m_nodes1;
			Liar::Delaunay::Node** cv1 = Liar::Delaunay::m_nodes2;
			if (revert)
			{
				cv0 = Liar::Delaunay::m_nodes2;
				cv1 = Liar::Delaunay::m_nodes1;
			}

			Liar::Uint i = 0;
			for (i = 0; i < num; ++i)
			{
				cv0[i]->Set(i, false, true);
				if (i > 0) cv0[i - 1]->next = cv0[i];
			}

			for (i = 0; i < pnum; ++i)
			{
				cv1[i]->Set(i, false, false);
				if (i > 0) cv1[i - 1]->next = cv1[i];
			}

			if (IntersectPoint(map, num, pnum, revert, rw) > 0)
			{
				unionNum = LinkToPolygon(map, num, pnum, revert);
			}
		}

		return unionNum;
	}

	int Delaunay::IntersectPoint(Liar::Map& map, Liar::Uint& cv0c, Liar::Uint& cv1c, bool revert, bool rw)
	{
		Liar::Delaunay::Node** cv0 = Liar::Delaunay::m_nodes1;
		Liar::Delaunay::Node** cv1 = Liar::Delaunay::m_nodes2;
		if (revert)
		{
			cv0 = Liar::Delaunay::m_nodes2;
			cv1 = Liar::Delaunay::m_nodes1;
		}

		Liar::Int insCnt = 0;
		bool hasIns = false;
		Liar::Delaunay::Node* startNode0 = cv0[0];
		Liar::Delaunay::Node* startNode1 = nullptr;
		Liar::Delaunay::Node* test = nullptr;

		Liar::Vector2f* ins = (Liar::Vector2f*)malloc(sizeof(Liar::Vector2f));
		ins->Set(Liar::ZERO, Liar::ZERO);

		size_t size = sizeof(Liar::Line2d);
		Liar::Line2d* line0 = (Liar::Line2d*)malloc(size);
		Liar::Line2d* line1 = (Liar::Line2d*)malloc(size);
		line0->Set(&map);
		line1->Set(&map);

		Liar::Int prec0 = 0;
		Liar::Int prec1 = 0;

		while (startNode0)
		{
			test = startNode0->next ? startNode0->next : cv0[0];
			line0->Set(startNode0->v, test->v);

			startNode1 = cv1[0];
			hasIns = false;

			while (startNode1)
			{
				test = startNode1->next ? startNode1->next : cv1[0];
				line1->Set(startNode1->v, test->v);

				ins->Set(Liar::ZERO, Liar::ZERO);

				if (line0->Intersection(*line1, ins) == Liar::LineClassification::SEGMENTS_INTERSECT)
				{
					if (GetNodeIndex(cv0, cv0c, *ins) == -1)
					{
						++insCnt;

						prec0 = cv0c;
						prec1 = cv1c;

						// expand

						Liar::Uint addPointIndex = map.AddVertex(*ins);

						Liar::Delaunay::Node* Node0 = cv0[prec0];
						Liar::Delaunay::Node* Node1 = cv1[prec1];

						Node0->Set(addPointIndex, true, true);
						Node1->Set(addPointIndex, true, false);

						Node0->other = Node1;
						Node1->other = Node0;


						Node0->next = startNode0->next;
						startNode0->next = Node0;
						Node1->next = startNode1->next;
						startNode1->next = Node1;

						Liar::Vector2f& line1PointB = line1->GetPointB();
						if (line0->ClassifyPoint(line1PointB, rw) == Liar::PointClassification::RIGHT_SIDE)
						{
							Node0->o = true;
							Node1->o = true;
						}

						hasIns = true;
						break;
					}
				}

				startNode1 = startNode1->next;
			}

			if (hasIns == false)
			{
				startNode0 = startNode0->next;
			}
		}

		ins->~Vector2f();
		free(ins);
		ins = nullptr;

		line0->~Line2d();
		free(line0);
		line0 = nullptr;

		line1->~Line2d();
		free(line1);
		line1 = nullptr;

		return insCnt;
	}

	Liar::Uint Delaunay::LinkToPolygon(Liar::Map& map, Liar::Uint cv0c, Liar::Uint cv1c, bool revert)
	{
		Liar::Uint linkNum = 0;

		Liar::Delaunay::Node** cv0 = Liar::Delaunay::m_nodes1;
		Liar::Delaunay::Node** cv1 = Liar::Delaunay::m_nodes2;
		if (revert)
		{
			cv0 = Liar::Delaunay::m_nodes2;
			cv1 = Liar::Delaunay::m_nodes1;
		}

		for (Liar::Uint i = 0; i < cv0c; ++i)
		{
			Liar::Delaunay::Node* testNode = cv0[i];
			if (testNode->i == true && testNode->p == false)
			{
				Vector2f* rcNodes = nullptr;
				int rcNum = 0;
				int test = 0;
				while (testNode)
				{
					testNode->p = true;

					if (testNode->i == true)
					{
						testNode->other->p = true;

						if (testNode->o == false)
						{
							if (testNode->isMain == true) testNode = testNode->other;
						}
						else
						{
							if (testNode->isMain == false) testNode = testNode->other;
						}
					}

					++rcNum;
					if (rcNodes) rcNodes = (Vector2f*)realloc(rcNodes, sizeof(Vector2f)*rcNum);
					else rcNodes = (Vector2f*)malloc(sizeof(Vector2f)*rcNum);

					Liar::Vector2f* testNodePoint = testNode->GetVertex(testNode->v);
					rcNodes[rcNum - 1].Set(*testNodePoint);
					++test;
					if (!testNode->next) testNode = testNode->isMain ? cv0[0] : cv1[0];
					else testNode = testNode->next;

					if (testNodePoint->Equals(rcNodes[0])) break;
				}

				if (rcNum > 0)
				{
					++linkNum;
					map.AddPolygon(rcNodes, rcNum);
				}

			}
		}
		return linkNum;
	}

	int Delaunay::GetNodeIndex(Liar::Delaunay::Node** cv, int ncount, const Liar::Vector2f& Node)
	{
		for (int i = 0; i < ncount; ++i)
		{
			Liar::Delaunay::Node& node = *(cv[i]);
			Liar::Vector2f* vec = node.GetVertex(node.v);
			if (vec->Equals(Node)) return i;
		}
		return -1;
	}

	bool Delaunay::CheckCross(Liar::NAVDTYPE r1[], Liar::NAVDTYPE r2[])
	{
		NAVDTYPE minx1 = r1[0];
		NAVDTYPE maxx1 = r1[1];
		NAVDTYPE miny1 = r1[2];
		NAVDTYPE maxy1 = r1[3];

		NAVDTYPE minx2 = r2[0];
		NAVDTYPE maxx2 = r2[1];
		NAVDTYPE miny2 = r2[2];
		NAVDTYPE maxy2 = r2[3];

		NAVDTYPE minx = minx1 > minx2 ? minx1 : minx2;
		NAVDTYPE miny = miny1 > miny2 ? miny1 : miny2;
		NAVDTYPE maxx = maxx1 > maxx2 ? maxx2 : maxx1;
		NAVDTYPE maxy = maxy1 > maxy2 ? maxy2 : maxy1;

		if (minx > maxx || miny > maxy)
		{
			return false;
		}
		else
		{
			return true;
		}
	}

	Liar::Delaunay::Node** Liar::Delaunay::m_nodes1 = nullptr;
	Liar::Delaunay::Node** Liar::Delaunay::m_nodes2 = nullptr;
	Liar::Uint Liar::Delaunay::m_numberNode1 = 0;
	Liar::Uint Liar::Delaunay::m_numberNode2 = 0;
#endif // UNION_POLYGON

	// ================================================ circle =======================================
	Delaunay::Circle::Circle() :
		center((Liar::Vector2f*)malloc(sizeof(Liar::Vector2f))),
		r(Liar::ZERO)
	{}

	Delaunay::Circle::~Circle()
	{
		center->~Vector2f();
		free(center);
		center = nullptr;
	}

	void Delaunay::Circle::Init()
	{
		center = (Liar::Vector2f*)malloc(sizeof(Liar::Vector2f));
		center->Set(Liar::ZERO, Liar::ZERO);
		r = Liar::ZERO;
	}

	void Delaunay::Circle::Set(const Liar::Vector2f& v, Liar::NAVDTYPE rr)
	{
		Set(v.GetX(), v.GetY(), rr);
	}

	void Delaunay::Circle::Set(Liar::NAVDTYPE x, Liar::NAVDTYPE y, Liar::NAVDTYPE rr)
	{
		center->Set(x, y);
		r = rr;
	}
	// ================================================ circle =======================================

	// ============================== Node =========================
	Liar::Delaunay::Node::Node(Liar::Map const* map) :
		Liar::MapSource(map),
		v(0), i(false), p(false), o(false),
		other(nullptr), isMain(false), next(nullptr)
	{}

	Liar::Delaunay::Node::~Node()
	{
		m_map = nullptr;
		other = nullptr;
		next = nullptr;
	}

	void Liar::Delaunay::Node::Init(Liar::Map const* map)
	{
		Liar::MapSource::Set(map);

		v = 0;
		i = p = o = isMain = false;
		other = next = nullptr;
	}

	void Liar::Delaunay::Node::Set(Liar::Uint v0, bool isInters, bool main)
	{
		v = v0;
		i = isInters;
		isMain = main;
	}
	// ============================== Node =========================

#ifdef EditorMod
	// ============================== BuildErrorLog =========================
	Liar::Delaunay::BuildErrorLog::BuildErrorLog(Liar::Uint id, bool l, bool c) :
		bid(id), loopDead(l), clockErr(c)
	{}

	void Liar::Delaunay::BuildErrorLog::Set(Liar::Uint id, bool l, bool c)
	{
		bid = id;
		loopDead = l;
		clockErr = c;
	}

	bool Liar::Delaunay::BuildErrorLog::operator==(const Liar::Delaunay::BuildErrorLog& rhs) const
	{
		return rhs.bid == bid;
	}

	bool Liar::Delaunay::BuildErrorLog::operator==(Liar::Uint rid) const
	{
		return rid == bid;
	}
	// ============================== BuildErrorLog =========================
#endif // EditorMod

}
