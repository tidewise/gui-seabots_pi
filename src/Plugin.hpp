#ifndef SEABOTS_PI_PLUGIN_HPP
#define SEABOTS_PI_PLUGIN_HPP

#include <wx/wx.h>
#include "ocpn_plugin.h"

namespace RTT {
    class TaskContext;
    namespace base {
        class ActivityInterface;
    }
}

namespace seabots_pi {
    class OCPNInterfaceImpl;

    class Plugin : public opencpn_plugin_116 {
        friend class orogenTaskTimer;

        struct orogenTaskTimer : public wxTimer
        {
            Plugin& plugin;
            orogenTaskTimer(Plugin& plugin);
            void Notify();
        };

        static const int REQUIRED_API_VERSION_MAJOR = 1;
        static const int REQUIRED_API_VERSION_MINOR = 16;
        static const int PLUGIN_VERSION_MAJOR = 0;
        static const int PLUGIN_VERSION_MINOR = 1;

        static const int UPDATE_PERIOD_MS = 100;

        static const char* NAME;
        static const char* DESCRIPTION_SHORT;
        static const char* DESCRIPTION_LONG;

        static const int TOOL_EXECUTE_ROOT_POSITION = -1;

        typedef std::vector<RTT::TaskContext*> Tasks;
        Tasks tasks;
        typedef std::vector<RTT::base::ActivityInterface*> Activities;
        Activities activities;

        void setupTaskActivity(
            RTT::TaskContext* task,
            RTT::base::ActivityInterface* activity = nullptr
        );

        OCPNInterfaceImpl* mInterface;

        orogenTaskTimer mTimer;
        void executeTasks();

        void loadSVGs();
        void setupToolbar();

        wxString mSeabotsSVG;
        wxBitmap mSeabotsBitmap;

        wxString mExecuteRouteSVG;
        wxString mExecuteRouteSVGToggled;
        int mExecuteRoutePosition = -1;
        int mExecuteRouteTool = -1;
        int GetToolbarToolCount(void);
        void OnToolbarToolCallback(int id);

    public:
        Plugin(void* pptr);

        int Init();
        bool DeInit();

        int GetAPIVersionMajor() { return REQUIRED_API_VERSION_MAJOR; }
        int GetAPIVersionMinor() { return REQUIRED_API_VERSION_MINOR; }
        int GetPlugInVersionMajor() { return PLUGIN_VERSION_MAJOR; }
        int GetPlugInVersionMinor() { return PLUGIN_VERSION_MINOR; }
        wxBitmap *GetPlugInBitmap();

        bool executeCurrentRoute();

        wxString GetCommonName() { return NAME; }
        wxString GetShortDescription() { return DESCRIPTION_SHORT; }
        wxString GetLongDescription() { return DESCRIPTION_LONG; }
    };
}

#endif