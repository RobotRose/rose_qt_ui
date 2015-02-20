//===============================================================================
// $Copyright: Copyright © 1996-2011 Sioux Embedded Systems B.V. $
// All rights reserved.
//===============================================================================

#include "GuiDefinitions.h"

#include <string>

TsrDefinitions* TsrDefinitions::OnlyOne = NULL;

const char* TSR_REDBUTTON =		
								"QPushButton"
								"{"
								"  background-color: rgb(255, 255, 255);"
								"  border-style: outset;"
								"  border-width: 2px;"
								"  border-radius: 6px;"
								"  border-color: black;"
								"  font: 12px;"
								"  padding: 6px;"
								"}"
								"QPushButton:hover"
								"{"
								"  background-color: rgb(255, 0, 0);"
								"  border-style: outset;"
								"  border-width: 2px;"
								"  border-radius: 6px;"
								"  border-color: white;"
								"  font: 12px;"
								"}"
								"QPushButton:hover:pressed"
								"{"
								"  background-color: rgb(200, 0, 0);"
								"  border-style: outset;"
								"  border-width: 2px;"
								"  border-radius: 6px;"
								"  border-color: black;"
								"  font: 12px;"
								"}"
"";

const char* TSR_DEFAULTBUTTON =	
								"QPushButton"
								"{"
								"  background-color: rgb(238, 237, 237);"
								"  border-style: outset;"
								"  border-width: 2px;"
								"  border-radius: 6px;"
								"  border-color: black;"
								"  font: 12px;"
								"}"
								"QPushButton:hover"
								"{"
								"  background-color: rgb(238, 237, 237);"
								"  border-style: outset;"
								"  border-width: 2px;"
								"  border-radius: 6px;"
								"  border-color: rgb(0, 142, 211);"
								"  font: 12px;"
								"}"
								"QPushButton:hover:pressed"
								"{"
								"  background-color: rgb(130, 130, 130);"
								"  border-style: outset;"
							    "  border-width: 2px;"
								"  border-radius: 6px;"
								"  border-color: black;"
								"  font: 4px"
								"}"
"";

 const char* TSR_DEFAULT_STYLE_SHEET =	
// 								"QPushButton"
// 								"{"
// 								"  background-color: rgb(238, 237, 237);"
// 								"  border-style: outset;"
// 								"  border-width: 2px;"
// 								"  border-radius: 6px;"
// 								"  border-color: black;"
// 								"  font: 12px;"
// 								"}"
// 								"QPushButton:hover"
// 								"{"
// 								"  background-color: rgb(238, 237, 237);"
// 								"  border-style: outset;"
// 								"  border-width: 2px;"
// 								"  border-radius: 6px;"
// 								"  border-color: rgb(0, 142, 211);"
// 								"  font: 12px;"
// 								"}"
// 								"QPushButton:hover:pressed"
// 								"{"
// 								"  background-color: rgb(130, 130, 130);"
// 								"  border-style: outset;"
// 							    "  border-width: 2px;"
// 								"  border-radius: 6px;"
// 								"  border-color: black;"
// 								"  font: 4px"
// 								"}"
								"QTableWidget" 
								"{"
								"  background: rgb(204, 229, 255);"
     							"  selection-background-color: rgb(204, 229, 255);"
     							"}"
     							"QTableWidgetItem" 
								"{"
								"  background: rgb(204, 229, 255);"
     							"  selection-background-color: (204, 229, 255);"
     							"}"
"";

std::string TsrDefinitions::GetRedButtonStyleSheet()
{
	return std::string(TSR_REDBUTTON);
}

std::string TsrDefinitions::GetDefaultButtonStyleSheet()
{
	return std::string(TSR_DEFAULTBUTTON);
}

std::string TsrDefinitions::GetStyleSheet()
{
	return std::string(TSR_DEFAULT_STYLE_SHEET);
}