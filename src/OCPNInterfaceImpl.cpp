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
    mLatLonConverter.setParameters(UTMConversionParameters const& parameters);
}

void OCPNInterfaceImpl::setLocalOrigin(base::Position const& origin)
{
    mLatLonConverter.setNWUOrigin(origin);
}

void OCPNInterfaceImpl::pushSystemPose(base::samples::RigidBodyState const& rbs)
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

void OCPNInterfaceImpl::pushNMEA(string nmea)
{
    wxString wxnmea(nmea);
    PushNMEABuffer(wxnmea);
}