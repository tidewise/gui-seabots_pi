#include "OCPNInterfaceImpl.hpp"
#include <wx/wx.h>
#include "ocpn_plugin.h"
#include "NMEA.hpp"

using namespace std;
using base::Angle;
using namespace seabots_pi;

void OCPNInterfaceImpl::setUTMConversionParameters(
    gps_base::UTMConversionParameters const& parameters
)
{
    mLatLonConverter.setParameters(parameters);
}

void OCPNInterfaceImpl::updateSystemPose(base::samples::RigidBodyState const& rbs)
{
    auto gps = mLatLonConverter.convertNWUToGPS(rbs);
    float velocity_over_ground = rbs.velocity.norm();

    base::Angle track;
    if (velocity_over_ground > 0.1)
        track = Angle::fromRad(atan2(rbs.velocity.y(), rbs.velocity.x()));
    else
        track = Angle::fromRad(base::getYaw(rbs.orientation));

    string rmc = nmea::createRMC(
        rbs.time,
        Angle::fromDeg(gps.latitude), Angle::fromDeg(gps.longitude),
        velocity_over_ground, track,
        base::Angle::fromRad(0));
    PushNMEABuffer(rmc);
}

/** Send an OpenCPN route to the Rock system */
void OCPNInterfaceImpl::pushRoute(PlugIn_Route const& route)
{
    std::vector<Waypoint> rock_wps;
    for (auto const& waypoint_ptr : *route.pWaypointList) {
        auto const& wp = *waypoint_ptr;

        gps_base::Solution solution;
        solution.positionType = gps_base::AUTONOMOUS;
        solution.latitude  = wp.m_lat;
        solution.longitude = wp.m_lon;
        solution.altitude  = 0;

        Waypoint rock_wp;
        rock_wp.position = mLatLonConverter.convertToNWU(solution).position;
        rock_wp.speed = wp.m_speed;
        rock_wp.course = base::Angle::fromDeg(- wp.m_course);
        rock_wps.push_back(rock_wp);
    }
    pushWaypoints(rock_wps);
}

void OCPNInterfaceImpl::pushNMEA(string nmea)
{
    wxString wxnmea(nmea);
    PushNMEABuffer(wxnmea);
}