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

#ifndef _PLOT_PANEL_H_
#define _PLOT_PANEL_H_

#include <glm/glm.hpp>
#include <wx/panel.h>

class plotPanel : public wxPanel
{
public:
    plotPanel(wxWindow *parent,
            wxWindowID winid = wxID_ANY,
            const wxPoint& pos = wxDefaultPosition,
            const wxSize& size = wxDefaultSize,
            long style = wxTAB_TRAVERSAL | wxNO_BORDER,
            const wxString& name = wxPanelNameStr);

    void OnPaint(wxPaintEvent& evt);
    DECLARE_EVENT_TABLE()

public:
    std::vector<std::pair<glm::vec2, glm::vec2>> lines;
    std::vector<std::pair<glm::vec2, float>> circles;

    void Clear();
    void Line(const glm::vec2& a, const glm::vec2& b);
    void Triangle(const glm::vec2& a, const glm::vec2& b, const glm::vec2& c);
    void Circle(const glm::vec2& a, float radius);
};

#endif // _PLOT_PANEL_H_
