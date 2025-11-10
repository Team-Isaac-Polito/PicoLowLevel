#line 1 "C:\\Users\\Utente\\Desktop\\RCR-master\\dxl_get_position\\lib\\WebManagement\\src\\html.h"
#ifndef html_h
#define html_h

inline const char* serverIndex PROGMEM = "<form method='POST' action='/update' enctype='multipart/form-data'>"
                            "<input type='file' name='update'>"
                            "<input type='submit' value='Update'>"
                          "</form>"
                          "<form method='POST' action='/config' enctype='multipart/form-data'>"
                            "<input type='file' name='config' accept=\".json\">"
                            "<input type='submit' value='Set config'>"
                          "</form>";

#endif 
