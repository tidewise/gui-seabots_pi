#ifndef SEABOTS_PI_OCPNInterfaceImpl_HPP
#define SEABOTS_PI_OCPNInterfaceImpl_HPP

#include <wx/wx.h>
#include "ocpn_plugin.h"

#include <string>
#include <seabots_pi/OCPNInterface.hpp> // Provided by gui/orogen/seabots_pi
#include <base/samples/RigidBodyState.hpp>
#include <gps_base/UTMConverter.hpp>

namespace seabots_pi {
    /**
     *
     */
    class OCPNInterfaceImpl : public OCPNInterface {
    public:
        using OCPNInterface::OCPNInterface;

        /** Configure the UTM-to-LatLon converter */
        void setUTMConversionParameters(
            gps_base::UTMConversionParameters const& parameters
        );

        /** Send the system pose to OpenCPN */
        void updateSystemPose(base::samples::RigidBodyState const& rbs);

        /** Send an OpenCPN route to the Rock system */
        void pushRoute(PlugIn_Route const& route);

    private:
        void pushNMEA(std::string nmea);

        gps_base::UTMConverter mLatLonConverter;
    };
}

#endif