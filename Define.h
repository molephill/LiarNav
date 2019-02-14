#ifndef __DEFINE_H__
#define __DEFINE_H__

namespace Liar
{
//#define _DEBUG 1
//#define ShareFind 1
//#define EditorMod 1
//#define DEBUG_NIF 1
//#define UNION_POLYGON 1
//#define UNIQUE_POINT 1
#define FindNearest 1
#define OutFind 1
#define PASS_FIRST 1
#define FIND_IN_SAME_MAP 1

	// data type
	typedef float NAVDTYPE;
	typedef unsigned int Uint;
	typedef int Int;

	//
	const static NAVDTYPE EPSILON = 0.0001f;
	const static NAVDTYPE ZERO = 0.0f;

	// define version
	const static int NAV_VERSION = 1;

	enum LineClassification
	{
		COLLINEAR,				// both lines are parallel and overlap each other
		LINES_INTERSECT,		// lines intersect, but their segments do not
		SEGMENTS_INTERSECT,		// both line segments bisect each other
		A_BISECTS_B,			// line segment B is crossed by line A
		B_BISECTS_A,			// line segment A is crossed by line B
		PARALELL				// the lines are paralell
	};

	enum PointClassification
	{
		ON_LINE,		// The point is on, or very near, the line
		LEFT_SIDE,		// looking from endpoint A to B, the test point is on the left
		RIGHT_SIDE		// looking from endpoint A to B, the test point is on the right
	};

	enum TriangleSide
	{
		SIDE_AB,
		SIDE_BC,
		SIDE_CA
	};

	enum ClockWiseDefine
	{
		CLOCK_NO_DEFINE,
		CLOCK_WISE_DEFINE,
		COUNT_CLOCK_WISE_DEFINE
	};

}

#endif // !__DEFINE_H__