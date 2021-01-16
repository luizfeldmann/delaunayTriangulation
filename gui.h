///////////////////////////////////////////////////////////////////////////
// C++ code generated with wxFormBuilder (version Oct 26 2018)
// http://www.wxformbuilder.org/
//
// PLEASE DO *NOT* EDIT THIS FILE!
///////////////////////////////////////////////////////////////////////////

#pragma once

#include <wx/artprov.h>
#include <wx/xrc/xmlres.h>
#include "plotPanel.h"
#include <wx/string.h>
#include <wx/bitmap.h>
#include <wx/image.h>
#include <wx/icon.h>
#include <wx/gdicmn.h>
#include <wx/button.h>
#include <wx/font.h>
#include <wx/colour.h>
#include <wx/settings.h>
#include <wx/spinctrl.h>
#include <wx/checkbox.h>
#include <wx/statline.h>
#include <wx/stattext.h>
#include <wx/sizer.h>
#include <wx/panel.h>
#include <wx/frame.h>

///////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////
/// Class mainFrame
///////////////////////////////////////////////////////////////////////////////
class mainFrame : public wxFrame
{
	private:

	protected:
		wxButton* btnRandom;
		wxSpinCtrl* m_spin;
		wxCheckBox* m_checkBox;
		wxStaticLine* m_staticline1;
		wxStaticText* m_staticText1;
		plotPanel* m_panel;

		// Virtual event handlers, overide them in your derived class
		void onShowWindow( wxShowEvent& event );
		void randomClick( wxCommandEvent& event );
		void panelClick( wxMouseEvent& event );


	public:

		mainFrame( wxWindow* parent, wxWindowID id = wxID_ANY, const wxString& title = wxEmptyString, const wxPoint& pos = wxDefaultPosition, const wxSize& size = wxSize( 926,527 ), long style = wxDEFAULT_FRAME_STYLE|wxTAB_TRAVERSAL );

		~mainFrame();

};

