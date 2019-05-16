#include <marnav/ais/message_01.hpp>
#include <marnav/ais/message_05.hpp>
#include <marnav/ais/ais.hpp>
#include <marnav/nmea/ais_helper.hpp>

#include "OCPNInterfaceImpl.hpp"
#include <wx/wx.h>
#include "ocpn_plugin.h"
#include "NMEA.hpp"

using namespace std;
using base::Angle;
using namespace seabots_pi;
using namespace marnav;

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

template <typename T>
utils::optional<T> marnav_from_rock(T value) {
    if (base::isUnknown(value)) {
        return utils::make_optional<T>();
    }
    else {
        return utils::make_optional<T>(value);
    }
}

static const int RAD2DEG = 180.0 / M_PI;
static const double SI2KNOTS = 1.94384;

ais::rate_of_turn rot_marnav_from_rock(double value) {
    if (base::isUnknown(value)) {
        return ais::rate_of_turn();
    }
    else {
        return ais::rate_of_turn(value * RAD2DEG * 60);
    }
}

template<typename Ret>
utils::optional<Ret> marnav_from_rock(base::Angle value) {
    double deg = value.getDeg();
    if (base::isUnknown(deg)) {
        return utils::optional<Ret>();
    }
    else if (deg < 0) {
        return utils::make_optional<Ret>(-deg);
    }
    else {
        return utils::make_optional<Ret>(360 - deg);
    }
}

template<typename Ret>
utils::optional<Ret> latlon_marnav_from_rock(base::Angle value) {
    double deg = value.getDeg();
    if (base::isUnknown(deg)) {
        return utils::optional<Ret>();
    }
    else {
        return utils::make_optional<Ret>(deg);
    }
}

/** Send an AIS position message to OpenCPN */
void OCPNInterfaceImpl::updateAIS(seabots_pi::AISPosition const& position)
{
    ais::message_01 ais_position;
    ais_position.set_mmsi(utils::mmsi(position.mmsi));
    ais_position.set_nav_status(
        static_cast<ais::navigation_status>(position.status)
    );
    ais_position.set_rot(rot_marnav_from_rock(position.yaw_velocity));
    ais_position.set_sog(marnav_from_rock(position.speed_over_ground * SI2KNOTS));
    ais_position.set_position_accuracy(position.high_accuracy_position);
    ais_position.set_cog(marnav_from_rock<double>(position.course_over_ground));
    ais_position.set_hdg(marnav_from_rock<uint32_t>(position.yaw));
    ais_position.set_longitude(
        latlon_marnav_from_rock<geo::longitude>(position.longitude)
    );
    ais_position.set_latitude(
        latlon_marnav_from_rock<geo::latitude>(position.latitude)
    );
    pushAIS(ais_position);
}

/** Send an AIS vessel message to OpenCPN */
void OCPNInterfaceImpl::updateAIS(seabots_pi::AISVesselInformation const& vessel)
{
    ais::message_05 ais_vessel;
    ais_vessel.set_mmsi(utils::mmsi(vessel.mmsi));
    ais_vessel.set_imo_number(vessel.imo);
    ais_vessel.set_callsign(vessel.call_sign);
    ais_vessel.set_shipname(vessel.name);
    ais_vessel.set_shiptype(ais::ship_type(vessel.ship_type));
    ais_vessel.set_to_bow(vessel.length / 2);
    ais_vessel.set_to_stern(vessel.length / 2);
    ais_vessel.set_to_port(vessel.width / 2);
    ais_vessel.set_to_starboard(vessel.width / 2);
    ais_vessel.set_draught(vessel.draught);
    pushAIS(ais_vessel);
}

template <typename T>
void OCPNInterfaceImpl::pushAIS(T const& message)
{
    auto payload = ais::encode_message(message);
    auto sentences = marnav::nmea::make_vdms(payload);
    for (auto const& s : sentences) {
        pushNMEA(marnav::nmea::to_string(*s));
    }
}
