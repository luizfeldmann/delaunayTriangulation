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

#include "sloan_delaunay.h"
#include <glm/gtx/norm.hpp>
#include <algorithm>
#include <stack>
#include <set>
#include <iostream>
#include <map>

namespace Sloan {

// VERTEX AND LIST
// ================================================================================


void VertexList::remove(const Vertex* rem)
{
    auto it = std::find_if(vertices.cbegin(), vertices.cend(), [&rem](const Vertex& test) -> bool {
        return (rem == &test);
    });

    if (it != vertices.cend())
        vertices.erase( it );
}

// adds a vertex and returns (stable) pointer to it
const Vertex* VertexList::Add(const Vertex& newVertex)
{
    vertices.emplace_back( Vertex(newVertex) );
    return &vertices.back();
}

// returns stable pointer to a vertex
// if not in the list already, it's added first
const Vertex* VertexList::FindOrAdd(const Vertex& newVertex)
{
    auto it = std::find_if(vertices.cbegin(), vertices.cend(), [&newVertex](const Vertex& v) -> bool
    {
        return almost_equal(v, newVertex);
    });

    if (it != vertices.cend())
        return &(*it);

    // did not find in the list - add new
    return Add(newVertex);
}
// Select three dummy points to form a supertriangle that completely encompasses all of the points to be triangulated.
//                  Keeping the angle at 45deg will minimize the total area of the super triangle
//         /| 2     (check by deriving total area with respect to alpha and equating to zero)
//        / |       Each side of the triangle must measure Width+Height of the rectangle containing all points
//       /  |
//      /___|
//     /|   |
//  0 /_|___| 1
//
tuple<const Vertex, const Vertex, const Vertex> VertexList::FindSupertriangle() const
{
        // Find minimum and maximum coordinates
    Coord minx = vertices.front().x;
    Coord maxx = minx;
    Coord miny = vertices.front().y;
    Coord maxy = miny;

    for (const Vertex& v : vertices)
    {
        minx = glm::min(minx, v.x);
        maxx = glm::max(maxx, v.x);

        miny = glm::min(miny, v.y);
        maxy = glm::max(maxy, v.y);
    }

    // scale up the triangle
    Coord width = maxx - minx;
    Coord height = maxy - miny;

    static const Coord upScale = 0.50f; // push edges out 50%
    const Coord margin = glm::max(width, height) * upScale;

    minx -= margin;
    miny -= margin;

    maxx += margin;
    maxy += margin;

    width = maxx - minx;
    height = maxy - miny;

    // make up the points
    return tuple<const Vertex, const Vertex, const Vertex>(
                        Vertex(minx - height, miny        ),
                        Vertex(maxx         , miny        ),
                        Vertex(maxx         , maxy + width)
                    );
}

// EDGE
// ==========================================================================================================
bool Edge::InTriangle(const Triangle* tri, unsigned char& which) const
{
      if (tri == NULL)
        return false;

      if (operator==(Edge(tri, 0)))
        which = 0;

      else if (operator==(Edge(tri, 1)))
        which = 1;

      else if (operator==(Edge(tri, 2)))
        which = 2;
      else
        return false;

      return true;
}

bool Edge::Intersect(const Edge* other, Vertex& where) const
{
    if (u == other->u || u == other->w || w == other->u || w == other->w)
        return false; // shared vertex != intersection

    const Vertex A = *u;
    const Vertex B = *w;
    const Vertex C = *(other->u);
    const Vertex D = *(other->w);

    const glm::tmat2x2<Coord> M( (B - A), -(D - C) );

    if (glm::abs( glm::determinant(M) ) == 0.0f) // cannot be inverted
        return false;

    const Vertex parameters = glm::inverse(M) * (C - A);

    if (parameters.x <= 0.0f || parameters.x >= 1.0f || parameters.y <= 0.0f || parameters.y > 1.0f) // returns false of intersections exactly on the endpoint vertices
        return false; // extended infinite lines do intersect, intersection occurs outside the bounds of the segment

    where = A + parameters.x*(B-A);

    return true;
}

// TRIANGLE
// ==========================================================================================================
void Triangle::ReplaceAdjacency(const Triangle* original, Triangle* newTri) // pointers to 'original' will be replaced by pointers to 'new'
{
    if (adjacents[0] == original)
        adjacents[0] = newTri;

    if (adjacents[1] == original)
        adjacents[1] = newTri;

    if (adjacents[2] == original)
        adjacents[2] = newTri;
}

char Triangle::GetAdjacencyIndex(const Triangle* other) const // gets the index of the adjacency entry where 'other' is set, if present
{
    if (other == NULL) // sanity check
    {
        std::cerr << "GetAdjacencyIndex received NULL Triangle*" << std::endl;
        throw;
    }

    #ifdef COMPARE_ADJACENCY_USING_VERTEX
    // useful if you don't trust adjacency list for some reason...
    const Vertex* _A = A();
    const Vertex* _B = B();
    const Vertex* _C = C();

    const Vertex* a = other->A();
    const Vertex* b = other->B();
    const Vertex* c = other->C();

    if ( ((_A == a) || (_A == b) || (_A == c)) && ((_B == a) || (_B == b) || (_B == c)) )
        return 2;

    if ( ((_B == a) || (_B == b) || (_B == c)) && ((_C == a) || (_C == b) || (_C == c)) )
        return 0;

    if ( ((_C == a) || (_C == b) || (_C == c)) && ((_A == a) || (_A == b) || (_A == c)) )
        return 1;

    return -1;
    #else
    if (adjacents[0] == other)
        return 0;

    else if (adjacents[1] == other)
        return 1;

    else if (adjacents[2] == other)
        return 2;

    else
        return -1;
    #endif
}

template <typename T> char almost_signum(T val)
{
    static const T epsilon = std::numeric_limits<T>::epsilon();

    if (val > epsilon)
        return 1;
    else if (val < -epsilon)
        return -1;
    else
        return 0;
}

/*bool Triangle::ContainsPoint(const Vertex& point) const
{
    // the triangle vertices
    const Vertex& A = *this->A();
    const Vertex& B = *this->B();
    const Vertex& C = *this->C();

    // the edges/segments
    const Vertex BA = B - A;
    const Vertex CB = C - B;
    const Vertex AC = A - C;

    // the segments from test point to triangle vertices
    const Vertex PA = point - A;
    const Vertex PB = point - B;
    const Vertex PC = point - C;


    // calculate cross product of triangle segments and connection from vertex to testing point
    // if both lines are parallel, then cross product returns zero
    // or else, it returns +/- depending on the order/side of the lines
    const Coord testBA = (BA.x * PA.y) - (BA.y * PA.x);
    const Coord testCB = (CB.x * PB.y) - (CB.y * PB.x);
    const Coord testAC = (AC.x * PC.y) - (AC.y * PC.x);

    #ifdef POINT_TEST_ASSUME_COUNTERCLOCKWISE
    // this simple test only works if vertices are counter-clockwise
    if ((testBA >=0) && (testCB >= 0) && (testAC >= 0))
        return true;
    return false;
    #else
    // test which side of the segment the triangle is on...
    const char sigBA = almost_signum(testBA);
    const char sigCB = almost_signum(testCB);
    const char sigAC = almost_signum(testAC);

    // if point lies on vertex: two signs are zero
    if ( ((sigBA == 0) && (sigCB == 0)) || ((sigCB == 0) && (sigAC == 0)) || ((sigAC == 0) && (sigBA == 0)) )
        return true;

    // if point lies on edge of triangle: one sign is zero, two other signs are equal
    if (sigBA == 0) // exactly co-linear with BA
        return (sigCB == sigAC);

    if (sigCB == 0) // exactly co-linear with CB
        return (sigBA == sigAC);

    if (sigAC == 0) // exactly co-linear with AC
        return (sigBA == sigCB);

    // if point lies inside triangle area: three signs are equal
    return (sigBA == sigCB) && (sigCB == sigAC);
    #endif
}*/

bool Triangle::IsVertex(const Vertex* v) const
{
    if (vertices[0] == v)
        return true;
    else if (vertices[1] == v)
        return true;
    else if (vertices[2] == v)
        return true;

    else
        return false;
}

Circle Triangle::FindCircumcircle() const
{
    const Vertex& A = *this->A();
    const Vertex& B = *this->B();
    const Vertex& C = *this->C();

    const Vertex CA = C - A;                    // segment from point C to point A
    const Vertex Nca = Vertex(CA.y, -CA.x);     // vector normal to segment CA
    const Vertex Mca = (C + A) / 2.0f;          // the midpoint from segment CA

    const Vertex CB = C - B;                    // segment from point C to point B
    const Vertex Ncb = Vertex(CB.y, -CB.x);     // vector normal to segment CB

    // solve linear system: find parameters T and S that map where in the lines the intersection occurs
    const Vertex ts = glm::inverse( mat2(Nca, -Ncb) ) * Vertex( (B - A )/2.0f );
    const Vertex intersection = (ts.x * Nca) + Mca;

    // radius is distance from center to any of the vertices
    const Coord radius = glm::distance(intersection, C );

    return Circle(intersection, radius);
}

bool Triangle::CircumcircleContains(const Vertex& point) const
{
    const Circle circle = FindCircumcircle();
    return glm::length2(circle.first - point) < ( circle.second * circle.second);
}

// BASE DELAUNAY TRIANGULATION
// =========================================================================================================
static INLINE void triangleGetAngles(const Triangle* t, Coord angles[3])
{
    const Coord ab2 = glm::distance2(*t->A(), *t->B());
    const Coord bc2 = glm::distance2(*t->B(), *t->C());
    const Coord ca2 = glm::distance2(*t->C(), *t->A());

    const Coord arg0 = (ab2 + ca2 - bc2) / glm::sqrt( 4* ab2 * ca2 );
    const Coord arg1 = (ab2 + bc2 - ca2) / glm::sqrt( 4* ab2 * bc2 );
    const Coord arg2 = (bc2 + ca2 - ab2) / glm::sqrt( 4* bc2 * ca2 );

    if (arg0 < -1 || arg1 < -1 || arg2 < -1 || arg0 > 1 || arg1 > 1 || arg2 > 1 )
    {
        std::cerr << "triangleGetAngles invalid cosine (triangle is degenerate): " << arg0 << "  " << arg1 << "  " << arg2 << std::endl;
        throw;
    }

    angles[0] = glm::acos(  arg0 );
    angles[1] = glm::acos(  arg1 );
    angles[2] = glm::acos(  arg2 );
}

static INLINE bool isQuadConvex(const Triangle* t1, unsigned char k, const Triangle* t2, unsigned char l)
{
    float angles1[3];
    triangleGetAngles(t1, angles1);

    float angles2[3];
    triangleGetAngles(t2, angles2);

    float quadAngle1 = angles1[(k + 2) % 3] + angles2[(l + 1) % 3];
    float quadAngle2 = angles1[(k + 1) % 3] + angles2[(l + 2) % 3];

    static const float maxAngle = glm::pi<Coord>();

    if ((quadAngle1 >= maxAngle) || (quadAngle2 >= maxAngle))
        return false;

    return true;
}

/*static INLINE Vertex Barycenter(const Triangle* t)
{
    return ( *t->vertices[0] + *t->vertices[1] + *t->vertices[2] ) / 3.0f;
}*/

static INLINE list<Triangle>::const_iterator OrientedWalkingSearch(const list<Triangle>& triangles, const Triangle* const start, const Vertex& P)
{
    const Triangle* search = start;

    while (1) {
        // get all vertices
        const Vertex& A = *search->A();
        const Vertex& B = *search->B();
        const Vertex& C = *search->C();

        const Vertex BA = B - A;
        const Vertex PA = P - A;

        if ((BA.x * PA.y) - (BA.y * PA.x) < 0) // is P outside current triangle, on negative side of edge BA?
            search = search->adjacents[2];     // yes, search triangle neighboring current on edge BA
        else
        {
            const Vertex CB = C - B;
            const Vertex PB = P - B;

            if ((CB.x * PB.y) - (CB.y * PB.x) < 0) // is P outside current triangle, on negative side of edge CB?
                search = search->adjacents[0];     // yes, search triangle neighboring current on edge CB
            else
            {
                const Vertex AC = A - C;
                const Vertex PC = P - C;

                if ((AC.x * PC.y) - (AC.y * PC.x) < 0)  // is P outside current triangle, on negative side of edge CA?
                    search = search->adjacents[1];      // yes, search triangle neighboring current on edge CA
                else
                    return search->iter;                // point must be *inside* current triangle...
            }
        }
    }
}

// t1 is the triangle with P for a vertex
// t2 is the triangle opposite to P
// both triangles are 'const' because the are not modified, but are simply purged from the list and two new triangles are inserted
pair<Triangle*, Triangle*> SwapDiagonal(TriangleList& list, const Triangle* t1, const Triangle* t2)
{
    if (t1 == NULL || t2 == NULL)
    {
        std::cerr << "SwapDiagonal: received NULL" << std::endl;
        throw; // sanity check
    }

    char p = t1->GetAdjacencyIndex(t2); // which edge index in the first triangle is shared with second?
    char q = t2->GetAdjacencyIndex(t1); // which edge in second triangle is shared with first?

    if (p < 0 || q < 0)
    {
        std::cerr << "SwapDiagonal: triangles don't share an edge" << std::endl;
        throw;
    }

    const Vertex* p0 = t1->vertices[(unsigned char) p];
    const Vertex* p1 = t1->vertices[(unsigned char) (p+1) % 3];
    const Vertex* p2 = t1->vertices[(unsigned char) (p+2) % 3];

    const Vertex* q0 = t2->vertices[(unsigned char) q];
    const Vertex* q1 = t2->vertices[(unsigned char)(q+1) % 3];
    const Vertex* q2 = t2->vertices[(unsigned char)(q+2) % 3];

    if ((p1 != q2) || (p2 != q1))
    {
        std::cerr << "SwapDiagonal: vertex relation not asserted" << std::endl;
        throw;
    }

    Triangle* P1 = t1->adjacents[(p + 1) % 3];
    Triangle* P2 = t1->adjacents[(p + 2) % 3];

    Triangle* Q1 = t2->adjacents[(q + 1) % 3];
    Triangle* Q2 = t2->adjacents[(q + 2) % 3];

    Triangle* tA = list.Add( p0, p1, q0,
                            Q1, NULL, P2 );

    Triangle* tB = list.Add( p0, q0, p2,
                            Q2, P1,  tA);

    tA->adjacents[1] = tB;

    if (P1 != NULL)
        P1->ReplaceAdjacency(t1, tB);

    if (P2 != NULL)
        P2->ReplaceAdjacency(t1, tA);

    if (Q1 != NULL)
        Q1->ReplaceAdjacency(t2, tA);

    if (Q2 != NULL)
        Q2->ReplaceAdjacency(t2, tB);

    list.remove(t1);
    list.remove(t2);

    return {tA, tB};
}

static tuple<const Vertex*, const Vertex*, const Vertex*> DelaunayBase(VertexList& vertices, TriangleList& triangles)
{
    // 3. Establish the supertriangle
    // Select three dummy points to form a supertriangle that completely encompasses all of the points to be triangulated.
    // This supertriangle initially defines a Delaunay triangulation which is comprised of a single triangle.
    const tuple<const Vertex, const Vertex, const Vertex> super_vertices = vertices.FindSupertriangle();
    const Vertex* super1 = vertices.Add(get<0>(super_vertices));
    const Vertex* super2 = vertices.Add(get<1>(super_vertices));
    const Vertex* super3 = vertices.Add(get<2>(super_vertices));
    triangles.Add( super1, super2, super3, NULL, NULL, NULL); // the supertriangle has no adjacents (it's the boundary)

    // 4. Loop over each point
    // For each point P in the list of sorted points, do steps 5-7.
    for (const Vertex& P : vertices.Get())
    {
        const Vertex* PP = &P;
        // ignore the points of the supertriangle
        if (PP == super1 || PP == super2 || PP == super3)
            continue;

        // 5. Insert new point in triangulation The net gain in the total number of triangles after this stage is two.
        auto old_triangle_iterator = OrientedWalkingSearch(triangles.Get(), &triangles.Get().back(), P);

        if (old_triangle_iterator == triangles.Get().end()) // sanity check
            throw;                                          // this test can never fail because supertriangle is guaranteed to enclose all points


        // 5.2 Delete this triangle and form three new triangles by connecting P to each of its vertices.
        Triangle* new_tri_A = triangles.Add(old_triangle_iterator->A(),
                                            old_triangle_iterator->B(),
                                            PP,

                                            NULL, // new_tri_B
                                            NULL, // new_tri_C
                                            old_triangle_iterator->adjacents[2]);

        Triangle* new_tri_B = triangles.Add(PP,
                                            old_triangle_iterator->B(),
                                            old_triangle_iterator->C(),

                                            old_triangle_iterator->adjacents[0],
                                            NULL, // new_tri_C
                                            new_tri_A );
        new_tri_A->adjacents[0] = new_tri_B;

        Triangle* new_tri_C = triangles.Add(old_triangle_iterator->A(),
                                            PP,
                                            old_triangle_iterator->C(),

                                            new_tri_B,
                                            old_triangle_iterator->adjacents[1],
                                            new_tri_A);

        new_tri_A->adjacents[1] = new_tri_C;
        new_tri_B->adjacents[1] = new_tri_C;

        // 5.3 Make sure to update the adjacency list
        Triangle* temp;
        if ((temp = old_triangle_iterator->adjacents[0]) != NULL) {
            temp->ReplaceAdjacency(&(*old_triangle_iterator), new_tri_B);
        }

        if ((temp = old_triangle_iterator->adjacents[1]) != NULL) {
            temp->ReplaceAdjacency(&(*old_triangle_iterator), new_tri_C);
        }

        if ((temp = old_triangle_iterator->adjacents[2]) != NULL) {
            temp->ReplaceAdjacency(&(*old_triangle_iterator), new_tri_A);
        }

        triangles.remove(old_triangle_iterator);

        // 6. Initialize stack
        // Place all triangles which are adjacent to the edges opposite P
        // on a last-in-first-out stack. There is a maximum of three such triangles (if triangle is boundary there will be less than 3)
        // PAIR: newly create triangle (P is vertex); second: forms quadrilateral with first (opposite P)
        stack<pair<Triangle*, Triangle*>> lifo;

        if ((temp = new_tri_A->adjacents[2] ) != NULL) {
            lifo.emplace(  pair<Triangle*, Triangle*>{new_tri_A, temp} );
        }

        if ((temp = new_tri_B->adjacents[0] ) != NULL) {
            lifo.emplace( pair<Triangle*, Triangle*>{new_tri_B, temp} );
        }

        if ((temp = new_tri_C->adjacents[1] ) != NULL) {
            lifo.emplace( pair<Triangle*, Triangle*>{new_tri_C, temp} );
        }

        // 7. Restore Delaunay triangulation
        // While the stack of triangles is not empty, execute Lawson’s swapping scheme
        while (!lifo.empty())
        {
            // 7.1. Remove a triangle which is opposite P from the top of the stack.
            pair<Triangle*, Triangle*> quad = lifo.top();
            lifo.pop();

            // 7.2. If P is outside (or on) the circumcircle for this triangle, return to step 7.1.
            if (!quad.second->CircumcircleContains(P))
                continue;

            // Else, the triangle containing P as a vertex(first) and the unstacked triangle(second) form a convex quadrilateral whose diagonal is drawn in the wrong direction.
            // Swap this diagonal so that two old triangles are replaced by two new triangles and the structure of the Delaunay triangulation is locally restored.
            auto new_triangles = SwapDiagonal(triangles, quad.first, quad.second);

            // 7.3. Place any triangles which are now oppositeP on the stack
            Triangle* opposite1 = new_triangles.first->adjacents[0];
            Triangle* opposite2 = new_triangles.second->adjacents[0];

            if (opposite1)
                lifo.emplace( pair<Triangle*, Triangle*>{new_triangles.first, opposite1} );

            if (opposite2)
                lifo.emplace( pair<Triangle*, Triangle*>{new_triangles.second, opposite2} );
        }
    }

    // a post-processing stage may require the supertriangle is still present...
    // pass a reference to the vertices, allowing it to be removed later
    return {super1, super2, super3};
}

static void RemoveSupertriangle(VertexList& vertices, TriangleList& triangles, tuple<const Vertex*, const Vertex*, const Vertex*>& super)
{
    // remove the supertriangle
    for (auto it = triangles.Get().cbegin(); it != triangles.Get().cend(); ++it)
    {
        if ( !(it->IsVertex(get<0>(super)) || it->IsVertex(get<1>(super)) || it->IsVertex(get<2>(super))) )
            continue;

        triangles.remove(it);
    }

    // remove the vertices we created for the supertriangle...
    vertices.remove(get<0>(super));
    vertices.remove(get<1>(super));
    vertices.remove(get<2>(super));
}

// SPECIFIC DELAUNAY TRIANGULATION
// =========================================================================================================
void SimpleDelaunay(VertexList& vertices, TriangleList& triangles)
{
    // perform base triangulation
    auto super = DelaunayBase(vertices, triangles);

    // no post-processing required
    RemoveSupertriangle(vertices, triangles, super);
}

void ConstrainedDelaunay(VertexList& vertices, TriangleList& triangles, const EdgeList& forceEdges)
{
    // perform base triangulation
    auto super = DelaunayBase(vertices, triangles);
    // <POST-PROCESSING>

    // 0. Find all edges in triangulation (without repeating)
    EdgeList edges;
    // 0.1 For each triangle, get it's 3 edges
    for (const Triangle& t : triangles.Get())
    {
        Edge triEdges[3] = {
            Edge(&t, 0),
            Edge(&t, 1),
            Edge(&t, 2),
        };

        // For each edge, add to the list of edges if (and only if) an equivalent edge is not already present
        for (const Edge& e : triEdges)
        {
            auto it = std::find(edges.cbegin(), edges.cend(), e);

            if (it == edges.cend())
                edges.push_back(e);
        }
    }

    // 1. Loop over each constrained edge (Vi-Vj). For each of these edges, do steps 2-4
    for (const Edge& ViVj : forceEdges)
    {
        // 2. Find intersecting edges
        // 2.1 If the constrained edge Vi-Vj is already present in the triangulation, then go to step 1.
        if (std::find(edges.cbegin(), edges.cend(), ViVj) != edges.cend())
            continue;

        // 2.2 Else, search the triangulation and store all of the edges that cross Vi-Vj.
        deque<decltype(edges)::const_iterator> intersectors;  // contains a reference to element in the 'edges' list
        deque<decltype(edges)::const_iterator> newly_created; // contains a reference to element in the 'edges' list

        Vertex intersectionPoint;
        for (auto it = edges.cbegin(); it != edges.cend(); ++it)
            if (it->Intersect(&ViVj, intersectionPoint))
                intersectors.push_back(it);

        // 3. Remove intersecting edges
        // While some edges still cross the constrained edge, do steps 3.1 and 3.2.
        while (!intersectors.empty())
        {
            // 3.1 Remove an edge from the list of edges that intersect Vi-Vj
            // Let this edge be defined by the vertices Vk and Vl.
            decltype(edges)::const_iterator VkVl = intersectors.back(); // this is an iterator to 'edges', not an iterator to 'intersectors' (it's just stored in intersectors)
            intersectors.pop_back();

            // 3.1a Find the (1 or 2) triangles that share this edge
            unsigned char k, l;
            list<Triangle>::const_iterator tri1 = std::find_if(triangles.Get().cbegin(), triangles.Get().cend(), [&VkVl, &k](const Triangle& test) -> bool {
                return VkVl->InTriangle(&test, k);
            });

            if (tri1 == triangles.Get().cend()) // sanity check
            {
                cerr << "Edge is not part of any triangle" << endl;
                throw; // edge does not belong to any triangle? how so?
            }

            Triangle* tri2 = tri1->adjacents[k];
            if (!VkVl->InTriangle(tri2, l))
            {
                cerr << "Delaunay constrain: triangles do not share edge" << endl;
                throw;
            }

            if ( !((tri1->vertices[(k + 2) % 3] == tri2->vertices[(l + 1) % 3]) && (tri1->vertices[(k + 1) % 3] == tri2->vertices[(l + 2) % 3])) )
            {
                cerr << "Delaunay constrain: expected vertices are not equal" << endl;
                throw;
            }

            // 3.2a If the two triangles that share the edge Vk-Vl do not form a quadrilateral
            // which is strictly convex (no angle>= 180), then place Vk-Vl back on the list of intersecting edges and go step 3.1
            if (!isQuadConvex(&*tri1, k, tri2, l))
            {
                intersectors.push_front(VkVl);
                continue;
            }

            // 3.2b Else, swap the diagonal of this strictly convex quadrilateral so that two old triangles are substituted for two new triangles.
            // Let the new diagonal be defined by the vertices Vm-Vn
            auto new_triangles = SwapDiagonal(triangles, &*tri1, tri2);

            edges.erase(VkVl); // old edge is no more...
            edges.emplace_back( Edge( new_triangles.first->A(), new_triangles.first->C() ) ); // new edge gets listed
            decltype(edges)::const_iterator VmVn = std::prev(edges.cend(), 1); // new edge get's referenced

            // 3.2c If Vm-Vn, still intersects the constrained edge Vi-Vj, then place it on the list of intersecting edges.
            // If Vm-Vn does not intersect Vi-Vj, then place Vm-Vn on a list of newly created edges
            if ( VmVn->Intersect(&ViVj, intersectionPoint) )
            {
                intersectors.push_front(VmVn);
            }
            else
                newly_created.push_back(VmVn);
        }

        // 4. (Optimization) Restore Delaunay triangulation
        // Repeat steps 4.1-4.3 until no further swaps take place.

        // 4.1 Loop over each edge in the list of newly created edges.
        // in loop below VkVl is an iterator to 'class Edge' stored in 'list<Edge> edges' which has been saved inside another list
        // the intention is to have an iterator readily available to facilitate removing from 'list<Edge> edges' when a swap takes place
        bool bSwap;
        do
        {
            bSwap = false;

            for (auto it = newly_created.cbegin(); it != newly_created.cend(); ++it)
            {
                decltype(edges)::const_iterator VkVl = *it;

                // 4.2 Let the newly created edge be defined by the vertices Vk and Vl.
                // If the edge Vk-Vl is equal to the constrained edge Vi-Vj, then skip (back) to step 4.1
                if (VkVl->operator==(ViVj))
                    continue;

                // 4.3 If the two triangles that share the edge Vk-Vl do not satisfy the Delaunay criterion
                // so that a vertex of one of the triangles is inside the circumcircle of the other triangle
                // then these triangles form a quadrilateral with the diagonal drawn in the wrong direction.
                unsigned char k, l;
                list<Triangle>::const_iterator tri1 = std::find_if(triangles.Get().cbegin(), triangles.Get().cend(), [&VkVl, &k](const Triangle& test) -> bool {
                    return VkVl->InTriangle(&test, k);
                });

                if (tri1 == triangles.Get().cend()) // sanity check
                {
                    cerr << "Optimization: Edge is not part of any triangle" << endl;
                    throw;
                }

                Triangle* tri2 = tri1->adjacents[k];
                if (!VkVl->InTriangle(tri2, l))
                {
                    cerr << "Optimization: Mismatch - Triangle on other side does not share edge" << endl;
                    throw;
                }

                // tri1 has vertices k, k+1, k+2
                // tri2 has vertices l, l+1, l+2
                // where (l+1 == k+2) and (l+2 == k+1) because those vertices are shared on the edge VkVl
                if ( !(tri1->CircumcircleContains( tri2->vertices[l] )
                    || tri2->CircumcircleContains( tri1->vertices[k] )) ) // check Delaunay criterion
                    continue;

                // In this case, the edge Vk-Vl, is swapped with the other diagonal (say) Vm-Vn,
                // thus substituting two old triangles for two new triangles, and Vk-Vl is replaced by Vm-Vn, in the list of newly created edges.
                auto new_triangles = SwapDiagonal(triangles, &*tri1, tri2);

                // after the swap takes placem tri1 and tri2 no longer exists
                // edge VkVl also no longer exists and has been replace by edge VmVn
                // old edge VkVl needs to be deleted from 2 places: global list 'list<Edge> edges' and local 'deque<newly_created>'

                edges.erase(VkVl);                                                                  // old edge is no more...
                edges.emplace_back( Edge( new_triangles.first->A(), new_triangles.second->B() ) );  // new edge gets listed

                // new edge get's referenced
                decltype(edges)::const_iterator VmVn = std::prev(edges.cend(), 1);
                newly_created.erase(it);
                newly_created.push_back(VmVn);

                bSwap = true; // the optimization continues as long as swaps take place...
            }
        } while (bSwap);
    }

    // 5 Remove superfluous triangles
    // Remove all triangles that contain a supertriangle vertex or lie outside the domain boundary.
    // </POST-PROCESSING>
    RemoveSupertriangle(vertices, triangles, super);
}

}
