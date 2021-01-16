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
//    Copyright: Luiz Gustavo Pfitscher e Feldmann, 2021                                //
// ===================================================================================  //
#include <chrono>
#include <random>
#include <wx/wx.h>

#include <wx/wxprec.h>

#ifndef WX_PRECOMP
    #include <wx/wx.h>
#endif

#include <wx/propgrid/propgrid.h>


using namespace std;
class mainApp: public wxApp
{
	public:
	virtual bool OnInit();
};

wxIMPLEMENT_APP(mainApp);

#include "gui.h"
bool mainApp::OnInit()
{
    mainFrame*mf = new mainFrame(NULL);
    mf->Show();

	return true;
}

#include "sloan_delaunay.h"

Sloan::VertexList list({
        Sloan::Vertex(0.1f, 0.1f),
        Sloan::Vertex(0.9f, 0.1f),
        Sloan::Vertex(0.9f, 0.9f),
});

const Sloan::Vertex* v1 = &::list.Get().front();
const Sloan::Vertex* v2 = &*std::next(::list.Get().begin(), 1);
const Sloan::Vertex* v3 = &*std::next(::list.Get().begin(), 2);

Sloan::EdgeList constraint({
    Sloan::Edge( v1, v2 ),
    Sloan::Edge( v2, v3 ),
    Sloan::Edge( v3, v1 ),
});


void mainFrame::panelClick( wxMouseEvent& event )
{
    wxCoord x, y;
    event.GetPosition(&x, &y);

    wxCoord w, h;
    m_panel->GetSize(&w, &h);

    ::list.Add( Sloan::Vertex(
                              (float)x/(float)w,
                              1.0f - ((float)y/(float)h)
                              ) );

    auto ev2 = wxShowEvent(0, true);
    onShowWindow( ev2 );
}

void mainFrame::randomClick( wxCommandEvent& event )
{
    ::list.clear();
    constraint.clear();

    static std::default_random_engine eng(std::random_device{}());
	static std::uniform_real_distribution<float> point_distrib(0, 1);

    std::cout << "randomizing ...";
	// add random points
	for (unsigned int i = 0; i < (unsigned int)m_spin->GetValue(); i++)
        ::list.FindOrAdd( Sloan::Vertex( point_distrib(eng), point_distrib(eng) ) ); // make sure no repetitions

    // pick N/10 points from list to form edges
    std::uniform_int_distribution<unsigned int> constraint_distrib(0, m_spin->GetValue() - 1);
    for (unsigned int i = 0; i < (unsigned int)m_spin->GetValue() / 10; i++)
    {
        Sloan::Edge newEdge((const Sloan::Vertex*)NULL, NULL);
        decltype(constraint)::const_iterator it;

        // get a random edge and check if it intersects the other edges
        do {
            auto iter1 = ::list.Get().cbegin();
            std::advance(iter1, constraint_distrib(eng));
            const Sloan::Vertex* v1 = &*iter1;

            auto iter2 = ::list.Get().cbegin();
            std::advance(iter1, constraint_distrib(eng));
            const Sloan::Vertex* v2 = &*iter2;

            newEdge = Sloan::Edge( v1,  v2 );

            // make sure the edge we add doesnt cross the other edges
            it = std::find_if(constraint.cbegin(), constraint.cend(), [&newEdge](const Sloan::Edge& test) -> bool {
                            Sloan::Vertex v;
                            return test.Intersect(&newEdge, v);
                         });
        }
        while (it != constraint.cend()); // if we found intersection, then try again... :(

        constraint.push_back(newEdge);
    }
    std::cout << "done!" << std::endl;

    // show plot
    auto ev2 = wxShowEvent(0, true);
    onShowWindow( ev2 );
}

void mainFrame::onShowWindow( wxShowEvent& event )
{
    m_panel->Clear();

    Sloan::TriangleList tris;

    auto t1 = std::chrono::high_resolution_clock::now();

    if (m_checkBox->GetValue())
        Sloan::ConstrainedDelaunay(::list, tris, constraint);
    else
        Sloan::SimpleDelaunay(::list, tris);

    auto t2 = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>( t2 - t1 ).count();
    std::cout << "Points: " << ::list.Get().size() << " Triangles: " << tris.Get().size() << " Time: " << duration << " ms " << std::endl;

    for (const Sloan::Triangle& t : tris.Get())
    {
        m_panel->Triangle(*t.A(), *t.B(), *t.C());

        const Sloan::Circle c = t.FindCircumcircle();
        m_panel->Circle(c.first, c.second);
    }

    m_panel->Refresh();
}
