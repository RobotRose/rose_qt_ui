//===============================================================================
// $Copyright: Copyright © 1996-2011 Sioux Embedded Systems B.V. $
// All rights reserved.
//===============================================================================

#ifndef QTDEFINITIONS_H
#define QTDEFINITIONS_H

#include <QtGui/QShortcut>

#define NEW_SHORTCUT( sequence, description, activated_handler, activatedambiguously_handler )\
{\
  QShortcut* shortcut;\
  shortcut = new QShortcut( QKeySequence(tr(sequence, description)), this );\
  if( shortcut )\
  {\
    connect( shortcut, SIGNAL( activated() ), this, SLOT( activated_handler() ) );\
    connect( shortcut, SIGNAL( activatedAmbiguously() ), this, SLOT( activatedambiguously_handler() ) );\
  }\
}

class TsrDefinitions
{
public:
	static TsrDefinitions* Instance()
	{
		if( !OnlyOne )
			OnlyOne = new TsrDefinitions();
		return OnlyOne;
	}
	//
	std::string GetRedButtonStyleSheet();
	std::string GetDefaultButtonStyleSheet();
	std::string GetStyleSheet();
	//
protected:

private:
	//
	static TsrDefinitions* OnlyOne;

};

#endif // QTDEFINITIONS
