// ===================================================================================  //
//    This program is free software: you can redistribute it and/or modify              //
//    it under the terms of the GNU General Public License as published by              //
//    the Free Software Foundation, either version 3 of the License, or                 //
//    (at your option) any later version.                                               //
//                                                                                      //
//    This program is distributed in the hope that it will be useful,                   //
//    but WITHOUT ANY WARRANTY; without even the implied warranty of                    //
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the                     //
//    GNU General Public License for more details.                                      //
//                                                                                      //
//    You should have received a copy of the GNU General Public License                 //
//    along with this program.  If not, see <https://www.gnu.org/licenses/>.            //
//                                                                                      //
//    Copyright: Luiz Gustavo Pfitscher e Feldmann, 2020                                //
// ===================================================================================  //

#ifndef _SLOAN_DELAUNAY_H_
#define _SLOAN_DELAUNAY_H_

#include <glm/glm.hpp>
#include <list>
#include <tuple>

#define INLINE inline __attribute__((always_inline))

namespace Sloan {
    using namespace std;
    using namespace glm;

    // Coordinates (float/double)
    using Coord  = float;


    INLINE bool almost_equal(const Coord& a, const Coord& b) // are two coordinates close enough to be considered identical?
    {
        const static Coord epsilon = 2.0f * std::numeric_limits<Coord>::epsilon();
        return (glm::abs(a - b) < epsilon);
    }

    //Vertex
    using Vertex = vec2;

    INLINE bool almost_equal(const Vertex& a, const Vertex& b) // test if two vertices are numerically equal
    {
        if (!almost_equal(a.x, b.x))
            return false;

        if (!almost_equal(a.y, b.y))
            return false;

        return true;
    }

    struct VertexList
    {
        private:
        list<Vertex> vertices;                              // list is pointer-stable: iterators and pointers remain constant after insertion/deletion
        VertexList(const VertexList&) = delete;             // cannot copy
        VertexList operator=(const VertexList&) = delete;   // cannot assign

        public:
        INLINE VertexList()
        {

        }

        INLINE VertexList(initializer_list<Vertex> data)
        {
            vertices.assign(data);
        }

        INLINE const list<Vertex>& Get() const // read-only access to the list...
        {
            return vertices;
        }

        INLINE void clear()
        {
            vertices.clear();
        }

        const Vertex* Add(const Vertex& newVertex);
        const Vertex* FindOrAdd(const Vertex& newVertex);
        void remove(const Vertex* rem);
        tuple<const Vertex, const Vertex, const Vertex> FindSupertriangle() const;
    };

    // Circumcircle in format <center, radius>
    using Circle = pair<Vertex, Coord>;

    struct Triangle
    {
        private:
        Triangle(const Triangle&) = delete;             // cannot copy
        Triangle operator=(const Triangle&) = delete;   // cannot assign
        bool operator==(const Triangle&) = delete;      // please dont compare directly

        INLINE Triangle(const Vertex* vA, const Vertex* vB,const Vertex* vC, Triangle* adjA, Triangle* adjB, Triangle* adjC) :
            vertices{vA, vB, vC},
            adjacents{adjA, adjB, adjC}
        {

        }

        friend class TriangleList;                      // only the list may create/store the triangle

        public:
        Triangle(Triangle&&) = default;
        list<Triangle>::const_iterator iter;            // points to self in the list holding it

        // VERTEX
        const Vertex* vertices[3];
        INLINE const Vertex* A() const
        {
            return vertices[0];
        }

        INLINE const Vertex* B() const
        {
            return vertices[1];
        }

        INLINE const Vertex* C() const
        {
            return vertices[2];
        }

        // ADJACENCY
        Triangle* adjacents[3];
        char GetAdjacencyIndex(const Triangle* other) const;
        void ReplaceAdjacency(const Triangle* original, Triangle* newTri);

        // TESTS
        bool IsVertex(const Vertex* v) const;

        bool ContainsPoint(const Vertex& point) const;
        INLINE bool ContainsPoint(const Vertex* point) const
        {
            return ContainsPoint(*point);
        }

        bool CircumcircleContains(const Vertex& point) const;
        INLINE bool CircumcircleContains(const Vertex* point) const
        {
            return CircumcircleContains(*point);
        }

        Circle FindCircumcircle() const;
    };

    struct TriangleList
    {
        private:
        TriangleList(const TriangleList&) = delete;             // cannot copy
        TriangleList operator=(const TriangleList&) = delete;   // cannot assign

        list<Triangle> triangles;

        public:
        INLINE TriangleList()
        {

        }

        INLINE const list<Triangle>& Get() const
        {
            return triangles;
        }

        INLINE void remove(list<Triangle>::const_iterator& rem)
        {
            triangles.erase(rem);
        }

        INLINE void remove(const Triangle* rem)
        {
            triangles.erase(rem->iter);
        }

        INLINE void clear()
        {
            triangles.clear();
        }

        INLINE Triangle* Add(const Vertex* vA, const Vertex* vB,const Vertex* vC, Triangle* adjA, Triangle* adjB, Triangle* adjC)
        {
            triangles.emplace_back( Triangle{vA, vB, vC, adjA, adjB, adjC} );
            decltype(triangles)::iterator it = std::prev(triangles.end(), 1);
            it->iter = it;

            return &*it;
        }
    };


    void SimpleDelaunay(VertexList& vertices, TriangleList& triangles);

    struct Edge
    {
        public:
        const Vertex* u;
        const Vertex* w;

        INLINE Edge(const Vertex* uu, const Vertex* ww) :
            u(uu),
            w(ww)
        {

        }

        INLINE Edge(const Triangle* t, unsigned char which) :
            u( (which == 0) ? t->B() : ((which == 1) ? t->C() : t->A()) ),
            w( (which == 0) ? t->C() : ((which == 1) ? t->A() : t->B()) )
        {
            //       /\ C2      EDGE 0: B->C (1->2)
            //      /  \        EDGE 1: C->A (2->0)
            //     /1  0\       EDGE 2: A->B (0->1)
            //    /   2  \      Edge number is same as opposite vertex number
            //A0 /--------\ B1  Edge is oriented counter-clockwise
        }


        INLINE bool operator==(const Edge& other) const
        {
            return ((u == other.u) && (w == other.w)) || ((u == other.w) && (w == other.u));
        }

        bool InTriangle(const Triangle* tri, unsigned char& which) const;
        bool Intersect(const Edge* other, Vertex& where) const;
    };

    using EdgeList = list<Edge>;

    void ConstrainedDelaunay(VertexList& vertices, TriangleList& triangles, const EdgeList& forceEdges);
};

#endif // _SLOAN_DELAUNAY_H_
