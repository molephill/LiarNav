#include "Delaunay.h"

namespace Liar
{
	Delaunay::Delaunay():
		m_line2ds(nullptr), m_totalLines(0),
		m_curNumLines(0)
	{
	}


	Delaunay::~Delaunay()
	{
		Liar::Uint i = 0;

		if (m_line2ds)
		{
			for (i = 0; i < m_totalLines; ++i)
			{
				m_line2ds[i]->~Line2d();
				m_line2ds[i] = nullptr;
			}
			free(m_line2ds);
			m_line2ds = nullptr;
			m_totalLines = 0;
		}
	}

	Liar::Uint Delaunay::Set(Liar::Map& map, bool isCW, Liar::Uint boxIndex)
	{
		m_curNumLines = 0;

#ifdef UNION_POLYGON
		map.UnionAll(isCW);
#endif // UNION_POLYGON

		BuildEdges(map);
		BuildTrianges(map, isCW, boxIndex);
		return map.NavMeshLinkCells(isCW);
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

		Line2d* edge = (Line2d*)malloc(sizeof(Line2d));
		edge->Set(initEdge);

		size_t blockSize = 0;

		do
		{
			idx = tmpLineCount - 1;

			edge->Set(*lineV[idx]);
			RemovePosLine(lineV, tmpLineCount, idx);

			Liar::Int p3Index = FindDT(map, *edge, isCW);

			if (p3Index < 0) continue;

			Liar::Cell* addTri = (Liar::Cell*)malloc(sizeof(Liar::Cell));
			addTri->Set(&map, edge->GetPointAIndex(), edge->GetPointBIndex(), p3Index);
			map.AddNavMeshCell(addTri);

			int index = 0;
			Liar::Vector2f* p3 = map.GetVertex(p3Index);
			if (FindLinePos(edge->GetPointA(), *p3, m_line2ds, m_curNumLines) < 0)
			{
				index = FindLinePos(edge->GetPointA(), *p3, lineV, tmpLineCount);
				if (index > -1)
				{
					RemovePosLine(lineV, tmpLineCount, index);
				}
				else
				{
					++tmpLineCount;
					blockSize = sizeof(Liar::Line2d*)*tmpLineCount;
					if (lineV) lineV = (Liar::Line2d**)realloc(lineV, blockSize);
					else lineV = (Liar::Line2d**)malloc(blockSize);
					Liar::Line2d* line13 = (Liar::Line2d*)malloc(sizeof(Liar::Line2d));
					line13->Set(&map, edge->GetPointAIndex(), p3Index);
					lineV[tmpLineCount - 1] = line13;
				}
			}

			if (FindLinePos(*p3, edge->GetPointB(), m_line2ds, m_curNumLines) < 0)
			{
				index = FindLinePos(*p3, edge->GetPointB(), lineV, tmpLineCount);

				if (index > -1)
				{
					RemovePosLine(lineV, tmpLineCount, index);
				}
				else
				{
					++tmpLineCount;
					blockSize = sizeof(Liar::Line2d*)*tmpLineCount;
					if (lineV) lineV = (Liar::Line2d**)realloc(lineV, blockSize);
					else lineV = (Liar::Line2d**)malloc(blockSize);
					Liar::Line2d* line32 = (Liar::Line2d*)malloc(sizeof(Liar::Line2d));
					line32->Set(&map, p3Index, edge->GetPointBIndex());
					lineV[tmpLineCount - 1] = line32;
				}
			}

		} while (tmpLineCount > 0);

		edge->~Line2d();
		free(edge);
		edge = nullptr;

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
				map.CalcBound(*polygon); // calcBound
				Liar::Uint numPoints = polygon->GetNumPoints();

				Liar::Uint pre = m_curNumLines;
				if (m_curNumLines + numPoints > m_totalLines)
				{
					Liar::Uint preTotal = m_totalLines;
					m_totalLines = m_curNumLines + numPoints;
					size_t blockSize = sizeof(Liar::Line2d*)*m_totalLines;
					if (m_line2ds) m_line2ds = (Liar::Line2d**)realloc(m_line2ds, blockSize);
					else m_line2ds = (Liar::Line2d**)malloc(blockSize);

					for (Liar::Uint initMap = preTotal; initMap < m_totalLines; ++initMap)
					{
						Liar::Line2d* tmpLine = (Liar::Line2d*)malloc(sizeof(Liar::Line2d));
						tmpLine->Set(&map);
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
			}
		}
	}

	Liar::Line2d& Delaunay::GetBoundEdage(Liar::Map& map, bool isCW, Liar::Uint boundIndex)
	{
		Liar::Uint numPolygons = map.GetNumPolygon();
		Liar::Uint numPoints = 0;
		Liar::Uint boundEdgeNum = map.GetPolygon(boundIndex)->GetNumPoints();

		Line2d* initEdge = m_line2ds[0];

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
					if (cit->Equals(initEdge->GetPointA()) || cit->Equals(initEdge->GetPointB())) continue;
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

	Liar::Int Delaunay::FindDT(Liar::Map& map, const Liar::Line2d& line, bool isCW)
	{
		Liar::Uint* allPoint = nullptr;
		int allPointCount = 0;

		Liar::Uint numPolygons = map.GetNumPolygon();
		Liar::Uint numPoints = 0;
		for (Liar::Uint i = 0; i < numPolygons; ++i)
		{
			Liar::Polygon* polygon = map.GetPolygon(i);
			numPoints = polygon->GetNumPoints();
			for (Liar::Uint j = 0; j < numPoints; ++j)
			{
				Liar::Uint pointIndex = polygon->GetPointIndex(j);
				if (IsVisiblePointOfLine(map, pointIndex, line, isCW))
				{
					++allPointCount;
					size_t blockSize = sizeof(Liar::Uint)*allPointCount;
					if (allPoint) allPoint = (Liar::Uint*)realloc(allPoint, blockSize);
					else allPoint = (Liar::Uint*)malloc(blockSize);
					allPoint[allPointCount - 1] = pointIndex;
				}
			}
		}

		if (allPointCount <= 0) return -1;

		Liar::Vector2f& p1 = line.GetPointA();
		Liar::Vector2f& p2 = line.GetPointB();

		Liar::Uint p3Index = allPoint[0];
		Liar::Vector2f* p3 = map.GetVertex(p3Index);

		bool loopSign = false;
		NAVDTYPE* fv = (NAVDTYPE*)malloc(sizeof(NAVDTYPE) * 4);
		Delaunay::Circle* cir = (Delaunay::Circle*)malloc(sizeof(Delaunay::Circle));
		cir->Set(0.0f, 0.0f, 0.0f);
		do
		{
			loopSign = false;
			CircumCircle(p1, p2, *p3, *cir);
			CircleBounds(*cir, fv);

			NAVDTYPE angle132 = abs(LineAngle(p1, *p3, p2));
			for (int i = 0; i < allPointCount; ++i)
			{
				Liar::Uint tmpP3Index = allPoint[i];
				Vector2f* vec = map.GetVertex(tmpP3Index);
				if (vec->Equals(p1) || vec->Equals(p2) || vec->Equals(*p3)) continue;

				NAVDTYPE x = vec->GetX();
				NAVDTYPE y = vec->GetY();
				NAVDTYPE minx = fv[0];
				NAVDTYPE maxx = fv[2];
				NAVDTYPE miny = fv[1];
				NAVDTYPE maxy = fv[3];


				if (x < minx || x > maxx || y < miny || y > maxy) continue;


				NAVDTYPE a1 = abs(LineAngle(p1, *vec, p2));
				if (a1 > angle132)
				{
					p3Index = tmpP3Index;
					p3 = vec;
					loopSign = true;
					break;
				}
			}
		} while (loopSign);

		cir->~Circle();
		free(cir);
		free(allPoint);
		free(fv);
		fv = nullptr;
		cir = nullptr;
		allPoint = nullptr;

		return p3Index;
	}

	bool Delaunay::IsVisiblePointOfLine(Liar::Map& map, Liar::Uint vecIndex, const Line2d& line, bool rw)
	{
		Liar::Vector2f& vec = *(map.GetVertex(vecIndex));

		if (vec.Equals(line.GetPointA()) || vec.Equals(line.GetPointB()))
		{
			return false;
		}


		if (line.ClassifyPoint(vec, rw) != PointClassification::RIGHT_SIDE) {
			return false;
		}

		if (IsVisibleIn2Point(map, line.GetPointAIndex(), vecIndex) == false)
		{
			return false;
		}


		if (IsVisibleIn2Point(map, line.GetPointBIndex(), vecIndex) == false)
		{
			return false;
		}

		return true;
	}

	bool Delaunay::IsVisibleIn2Point(Liar::Map& map, Liar::Uint pa, Liar::Uint pb)
	{
		Line2d* linepapb = (Line2d*)malloc(sizeof(Line2d));
		linepapb->Set(&map);
		linepapb->Set(pa, pb);

		Vector2f* interscetVector = (Vector2f*)malloc(sizeof(Vector2f));
		interscetVector->Set(0.0f, 0.0f);

		for (Liar::Uint i = 0; i < m_curNumLines; ++i)
		{
			Line2d* limeTmp = m_line2ds[i];
			if (linepapb->Intersection(*limeTmp, interscetVector) == LineClassification::SEGMENTS_INTERSECT)
			{
				Liar::Vector2f& pa = linepapb->GetPointA();
				Liar::Vector2f& pb = linepapb->GetPointB();
				if (!pa.Equals(*interscetVector) && !pb.Equals(*interscetVector))
				{
					linepapb->~Line2d();
					free(linepapb);
					interscetVector->~Vector2f();
					free(interscetVector);
					linepapb = nullptr;
					interscetVector = nullptr;
					return false;
				}
			}
		}

		linepapb->~Line2d();
		free(linepapb);
		interscetVector->~Vector2f();
		free(interscetVector);
		linepapb = nullptr;
		interscetVector = nullptr;
		return true;
	}

	void Delaunay::RemovePosLine(Liar::Line2d** lv, int& lvlen, int pos)
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
					lv[i] = nullptr;
					--lvlen;
				}
			}
		}
		
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

		NAVDTYPE cosfi = 0.0f, fi = 0.0f, norm = 0.0f;
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

	// ================================================ circle =======================================
	Delaunay::Circle::Circle() :
		center((Liar::Vector2f*)malloc(sizeof(Liar::Vector2f))),
		r(0.0f)
	{}

	Delaunay::Circle::~Circle()
	{
		center->~Vector2f();
		free(center);
		center = nullptr;
	}

	void Delaunay::Circle::Set(const Liar::Vector2f& v, Liar::NAVDTYPE rr)
	{
		Set(v.GetX(), v.GetY(), rr);
	}

	void Delaunay::Circle::Set(Liar::NAVDTYPE x, Liar::NAVDTYPE y, Liar::NAVDTYPE rr)
	{
		if (!center) center = (Liar::Vector2f*)malloc(sizeof(Liar::Vector2f));
		center->Set(x, y);
		r = rr;
	}
	// ================================================ circle =======================================
}
