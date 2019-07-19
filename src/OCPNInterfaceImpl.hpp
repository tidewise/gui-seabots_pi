#ifndef SEABOTS_PI_OCPNInterfaceImpl_HPP
#define SEABOTS_PI_OCPNInterfaceImpl_HPP

#include <wx/wx.h>
#include "ocpn_plugin.h"

#include <string>
#include <seabots_pi/OCPNInterface.hpp> // Provided by gui/orogen/seabots_pi
#include <base/samples/RigidBodyState.hpp>
#include <gps_base/UTMConverter.hpp>
#include <usv_control/Trajectory.hpp>

namespace seabots_pi {
    /**
     *
     */
    class OCPNInterfaceImpl : public OCPNInterface {
    public:
        using OCPNInterface::OCPNInterface;

        struct TrajectoryPoint
        {
            float latitude_deg = base::unknown<double>();
            float longitude_deg = base::unknown<double>();
            float velocity = base::unknown<double>();
        };

        struct SampledTrajectory
        {
            base::Time start_time;
            base::Time dt;
            std::vector<TrajectoryPoint> points;
        };

        /** Configure the UTM-to-LatLon converter */
        void setUTMConversionParameters(
            gps_base::UTMConversionParameters const& parameters
        );

        /** Send the system pose to OpenCPN */
        void updateSystemPose(base::samples::RigidBodyState const& rbs);

        /** Send an OpenCPN route to the Rock system */
        void pushRoute(PlugIn_Route const& route);

        /** Send an AIS position message to OpenCPN */
        void updateAIS(seabots_pi::AISPosition const& position);

        /** Send an AIS vessel message to OpenCPN */
        void updateAIS(seabots_pi::AISVesselInformation const& vessel);

        /** Visualize the trajectory planned by the seabots system */
        virtual void updatePlannedTrajectory(
            std::vector<usv_control::Trajectory> const& trajectories,
            base::Time dt = base::Time::fromSeconds(5)
        );

        std::pair<int, std::vector<SampledTrajectory>> const&
            getCurrentPlannedTrajectory() const;

        SampledTrajectory sampleTrajectory(
            usv_control::Trajectory const& trajectory,
            base::Time dt = base::Time::fromSeconds(5)
        );

    private:
        template<typename T> void pushAIS(T const& msg);
        void pushNMEA(std::string nmea);

        gps_base::UTMConverter mLatLonConverter;

        std::pair<int, std::vector<SampledTrajectory>> currentPlannedTrajectory;
    };
}

#endif