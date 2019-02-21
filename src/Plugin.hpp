#ifndef SEABOTS_PI_PLUGIN_HPP
#define SEABOTS_PI_PLUGIN_HPP

#include <wx/wx.h>
#include "ocpn_plugin.h"

namespace seabots_pi {
    class Plugin : public opencpn_plugin_116 {
        static const int REQUIRED_API_VERSION_MAJOR = 1;
        static const int REQUIRED_API_VERSION_MINOR = 16;
        static const int PLUGIN_VERSION_MAJOR = 0;
        static const int PLUGIN_VERSION_MINOR = 1;

        static const char* NAME;
        static const char* DESCRIPTION_SHORT;
        static const char* DESCRIPTION_LONG;
    public:
        Plugin(void* pptr);

        int Init();
        bool DeInit();

        int GetAPIVersionMajor() { return REQUIRED_API_VERSION_MAJOR; }
        int GetAPIVersionMinor() { return REQUIRED_API_VERSION_MINOR; }
        int GetPlugInVersionMajor() { return PLUGIN_VERSION_MAJOR; }
        int GetPlugInVersionMinor() { return PLUGIN_VERSION_MINOR; }
        wxBitmap *GetPlugInBitmap();

        wxString GetCommonName() { return NAME; }
        wxString GetShortDescription() { return DESCRIPTION_SHORT; }
        wxString GetLongDescription() { return DESCRIPTION_LONG; }
    };
}

#endif