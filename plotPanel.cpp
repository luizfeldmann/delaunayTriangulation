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
#include "plotPanel.h"

BEGIN_EVENT_TABLE(plotPanel, wxPanel)
EVT_PAINT(plotPanel::OnPaint)
END_EVENT_TABLE()

plotPanel::plotPanel(wxWindow *parent, wxWindowID winid, const wxPoint& pos, const wxSize& size, long style, const wxString& name) :
    wxPanel(parent, winid, pos, size, style | wxFULL_REPAINT_ON_RESIZE, name)
{
    SetBackgroundStyle(wxBG_STYLE_PAINT);
}

void plotPanel::Clear()
{
    lines.clear();
    circles.clear();
}

void plotPanel::Circle(const glm::vec2& a, float radius)
{
    circles.emplace_back( std::pair<glm::vec2, float>{a, radius} );
}

void plotPanel::Line(const glm::vec2& a, const glm::vec2& b)
{
    lines.emplace_back( std::pair<glm::vec2,glm::vec2>(a, b) );
}

void plotPanel::Triangle(const glm::vec2& a, const glm::vec2& b, const glm::vec2& c)
{
    Line(a,b);
    Line(b,c);
    Line(c,a);
}

#include <wx/dcbuffer.h>
void plotPanel::OnPaint(wxPaintEvent& evt)
{
    wxAutoBufferedPaintDC dc(this);

    wxCoord width, height;
    GetSize(&width, &height);

    auto coord = [&width, &height](float x, float y) -> wxPoint
    {
      return wxPoint(x * (float)width, (1.0f - y) * (float)height);
    };

    // background
    dc.SetBrush(*wxTRANSPARENT_BRUSH);
    dc.SetBackground(*wxWHITE);
    dc.Clear();

    // lines
    dc.SetPen(*wxRED_PEN);
    for (const std::pair<glm::vec2, glm::vec2>& line : lines)
        dc.DrawLine(coord(line.first.x, line.first.y), coord(line.second.x, line.second.y));

    // circles
    dc.SetPen(*wxGREEN_PEN);
    for (const std::pair<glm::vec2, float>& circle : circles)
    {
        wxPoint center = coord(circle.first.x, circle.first.y);
        wxSize size = wxSize(circle.second * width, circle.second*height);

        dc.DrawEllipse(center.x - size.x, center.y - size.y, size.x * 2, size.y * 2);
    }
}
