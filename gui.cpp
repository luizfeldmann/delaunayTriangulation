///////////////////////////////////////////////////////////////////////////
// C++ code generated with wxFormBuilder (version Oct 26 2018)
// http://www.wxformbuilder.org/
//
// PLEASE DO *NOT* EDIT THIS FILE!
///////////////////////////////////////////////////////////////////////////

#include "gui.h"

///////////////////////////////////////////////////////////////////////////

mainFrame::mainFrame( wxWindow* parent, wxWindowID id, const wxString& title, const wxPoint& pos, const wxSize& size, long style ) : wxFrame( parent, id, title, pos, size, style )
{
	this->SetSizeHints( wxDefaultSize, wxDefaultSize );

	wxBoxSizer* bSizer1;
	bSizer1 = new wxBoxSizer( wxVERTICAL );

	wxBoxSizer* bSizer2;
	bSizer2 = new wxBoxSizer( wxHORIZONTAL );

	btnRandom = new wxButton( this, wxID_ANY, wxT("Random points"), wxDefaultPosition, wxDefaultSize, 0 );
	bSizer2->Add( btnRandom, 0, wxALL, 5 );

	m_spin = new wxSpinCtrl( this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, 3, 100000, 50 );
	bSizer2->Add( m_spin, 0, wxALL, 5 );

	m_checkBox = new wxCheckBox( this, wxID_ANY, wxT("Constrained"), wxDefaultPosition, wxDefaultSize, 0 );
	m_checkBox->SetValue(true);
	bSizer2->Add( m_checkBox, 0, wxALL, 5 );

	m_staticline1 = new wxStaticLine( this, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxLI_VERTICAL );
	bSizer2->Add( m_staticline1, 0, wxEXPAND | wxALL, 5 );

	m_staticText1 = new wxStaticText( this, wxID_ANY, wxT("Notice: measured time may vary grotesquely in some runs due to system preempting the thread"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText1->Wrap( -1 );
	bSizer2->Add( m_staticText1, 0, wxALL, 5 );


	bSizer1->Add( bSizer2, 0, wxEXPAND, 5 );

	m_panel = new plotPanel( this, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL );
	bSizer1->Add( m_panel, 1, wxEXPAND | wxALL, 5 );


	this->SetSizer( bSizer1 );
	this->Layout();

	this->Centre( wxBOTH );

	// Connect Events
	this->Connect( wxEVT_SHOW, wxShowEventHandler( mainFrame::onShowWindow ) );
	btnRandom->Connect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( mainFrame::randomClick ), NULL, this );
	m_panel->Connect( wxEVT_LEFT_DOWN, wxMouseEventHandler( mainFrame::panelClick ), NULL, this );
}

mainFrame::~mainFrame()
{
	// Disconnect Events
	this->Disconnect( wxEVT_SHOW, wxShowEventHandler( mainFrame::onShowWindow ) );
	btnRandom->Disconnect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( mainFrame::randomClick ), NULL, this );
	m_panel->Disconnect( wxEVT_LEFT_DOWN, wxMouseEventHandler( mainFrame::panelClick ), NULL, this );

}
