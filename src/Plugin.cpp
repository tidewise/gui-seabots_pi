#include "Plugin.hpp"
#include "Images.hpp"
#include <iostream>

using namespace seabots_pi;

const char* Plugin::NAME = "Seabots";
const char* Plugin::DESCRIPTION_SHORT = "Seabots interface to OpenCPN";
const char* Plugin::DESCRIPTION_LONG = "OpenCPN/Seabots interface, to "
    "use OpenCPN for route planning in a Seabots system";

Plugin::Plugin(void* pptr)
    : opencpn_plugin_116(pptr)
{
    initialize_images();
}

int Plugin::Init() {
    return 0;
}
bool Plugin::DeInit() {
    return true;
}

wxBitmap* Plugin::GetPlugInBitmap() {
    return _img_seabots;
}

extern "C" DECL_EXP opencpn_plugin* create_pi(void *ppimgr)
{
    return new seabots_pi::Plugin(ppimgr);
}

extern "C" DECL_EXP void destroy_pi(opencpn_plugin* p)
{
    delete p;
}
