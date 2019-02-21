
#ifndef __LINE2D_H__
#define __LINE2D_H__

#include "MapSource.h"

#ifdef  EditorMod  
#define  LINE2D_API _declspec(dllexport)  
#else  
#define  LINE2D_API _declspec(dllimport)  
#endif 


namespace Liar
{
#ifdef EditorMod
	class LINE2D_API Line2d :public Liar::MapSource
#else
	class Line2d:public Liar::MapSource
#endif // EditorMod
	{
	public:
		Line2d(Liar::Map const*);
		~Line2d();

	private:
		Liar::Uint m_pointIndexA;
		Liar::Uint m_pointIndexB;

	public:
		void Set(Liar::Map const*);
		void Set(Liar::Uint, Liar::Uint);
		void Set(const Liar::Line2d&);
		void Set(Liar::Map const*, Liar::Uint, Liar::Uint);
		Liar::Vector2f& GetPointA() const;
		Liar::Vector2f& GetPointB() const;
		Liar::Uint GetPointAIndex() const { return m_pointIndexA; };
		Liar::Uint GetPointBIndex() const { return m_pointIndexB; };
		Liar::LineClassification Intersection(const Liar::Line2d&, Liar::Vector2f*);
		Liar::PointClassification ClassifyPoint(const Liar::Vector2f&, bool = true, Liar::NAVDTYPE = Liar::EPSILON) const;
		Liar::PointClassification ClassifyPoint(Liar::NAVDTYPE, Liar::NAVDTYPE, bool = true, Liar::NAVDTYPE = Liar::EPSILON) const;
		Liar::NAVDTYPE Length() const;
		bool Equals(const Liar::Line2d&, bool = true, Liar::NAVDTYPE = Liar::EPSILON) const;
		bool Equals(const Liar::Vector2f&, const Liar::Vector2f&, bool = true, Liar::NAVDTYPE = Liar::EPSILON) const;
		bool Equals(Liar::NAVDTYPE, Liar::NAVDTYPE, Liar::NAVDTYPE, Liar::NAVDTYPE, bool = true, Liar::NAVDTYPE = Liar::EPSILON) const;
		void Dispose();

		// static function
		static Liar::PointClassification ClassifyPoint(const Liar::Line2d&, const Liar::Vector2f&, bool = true, Liar::NAVDTYPE = Liar::EPSILON);
		static Liar::PointClassification ClassifyPoint(const Liar::Line2d&, Liar::NAVDTYPE, Liar::NAVDTYPE, bool = true, Liar::NAVDTYPE = Liar::EPSILON);
		static Liar::PointClassification ClassifyPoint(const Liar::Vector2f&, const Liar::Vector2f&, const Liar::Vector2f&, bool = true, Liar::NAVDTYPE = Liar::EPSILON);
		static Liar::PointClassification ClassifyPoint(const Liar::Vector2f&, const Liar::Vector2f&, Liar::NAVDTYPE, Liar::NAVDTYPE, bool = true, Liar::NAVDTYPE = Liar::EPSILON);
		static Liar::NAVDTYPE SignedDistance(const Liar::Line2d&, Liar::NAVDTYPE, Liar::NAVDTYPE);
		static Liar::NAVDTYPE SignedDistance(const Liar::Vector2f&, const Liar::Vector2f&, Liar::NAVDTYPE, Liar::NAVDTYPE);
		static Liar::NAVDTYPE Length(const Liar::Line2d&);
		static Liar::NAVDTYPE Length(const Liar::Vector2f&, const Liar::Vector2f&);

	private:
		Liar::NAVDTYPE SignedDistance(const Liar::Vector2f& v) const;
		Liar::NAVDTYPE SignedDistance(Liar::NAVDTYPE, Liar::NAVDTYPE) const;
	};
}


#endif // ! __LINE2D_H__