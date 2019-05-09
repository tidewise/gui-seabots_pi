#ifndef SEABOTS_PI_OCPNInterfaceImpl_HPP
#define SEABOTS_PI_OCPNInterfaceImpl_HPP

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
        void setUTMConversionParameters(
            gps_base::LocalCartesianConversionParameters const& parameters
        );
        void pushSystemPose(base::samples::RigidBodyState const& rbs);

    private:
        void pushNMEA(std::string nmea);

        gps_base::UTMConverter mLatLonConverter;
    };
}

#endif