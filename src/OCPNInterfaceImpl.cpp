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

static const int RAD2DEG = 180.0 / M_PI;
static const double SI2KNOTS = 1.94384;

void OCPNInterfaceImpl::setUTMConversionParameters(
    gps_base::UTMConversionParameters const& parameters
)
{
    currentPlanningResult = SampledPlanningResult();
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
        if (rock_wp.speed == 0) {
            rock_wp.speed = route.m_PlannedSpeed;
        }
        rock_wp.course = base::Angle::fromDeg(- wp.m_course);
        rock_wps.push_back(rock_wp);
    }
    rock_wps[0].speed = 0;
    rock_wps[rock_wps.size() - 1].speed = 0;

    for (auto& wps : rock_wps) {
        wps.speed /= SI2KNOTS;
    }

    PlanningRequest request;
    request.id = ++lastPlanningRequestID;
    request.waypoints = std::move(rock_wps);
    lastPlannedRouteGUID = route.m_GUID;
    pushPlanningRequest(request);
}

void OCPNInterfaceImpl::updatePlanningResult(PlanningResult const& result, base::Time dt)
{
    std::vector<SampledTrajectory> sampled;
    sampled.reserve(result.trajectories.size());
    for (auto const& rockTrajectory : result.trajectories) {
        sampled.push_back(sampleTrajectory(rockTrajectory, dt));
    }
    currentPlanningResult.id = result.id;
    currentPlanningResult.success = result.success;
    currentPlanningResult.invalid_waypoint = result.invalid_waypoint;
    currentPlanningResult.error_message = result.error_message;
    currentPlanningResult.trajectories = result.trajectories;
    currentPlanningResult.sampled = std::move(sampled);

    if (!currentPlanningResult.success) {
        wxMessageBox("Planning route failed: " + currentPlanningResult.error_message);
    }
}

bool OCPNInterfaceImpl::hasValidPlanningResultForRoute(std::string guid) const {
    return lastPlannedRouteGUID == guid &&
           lastPlanningRequestID == currentPlanningResult.id;
}

bool OCPNInterfaceImpl::executeCurrentTrajectories(std::string guid) {
    if (!hasValidPlanningResultForRoute(guid))
        return false;
    pushTrajectoriesForExecution(currentPlanningResult.trajectories);
    return true;
}

OCPNInterfaceImpl::SampledPlanningResult const&
    OCPNInterfaceImpl::getCurrentPlanningResult() const
{
    return currentPlanningResult;
}


OCPNInterfaceImpl::SampledTrajectory OCPNInterfaceImpl::sampleTrajectory(
    usv_control::Trajectory const& trajectory, base::Time dt)
{
    auto startTime = trajectory.getStartTime();
    auto endTime   = trajectory.getEndTime();

    SampledTrajectory sampledTrajectory;
    auto& sampledPoints = sampledTrajectory.points;
    int pointCount = ceil((endTime - startTime).toSeconds() / dt.toSeconds());
    sampledPoints.reserve(pointCount);
    for (base::Time t = startTime; t < endTime; t = t + dt) {
        Eigen::Vector2d p;
        Eigen::Vector2d v;
        tie(p, v) = trajectory.getLinearAndTangent(t);

        base::samples::RigidBodyState rbs;
        rbs.position = Eigen::Vector3d(p.x(), p.y(), 0);
        auto latlon = mLatLonConverter.convertNWUToGPS(rbs);

        TrajectoryPoint ocpnPoint;
        ocpnPoint.latitude_deg  = latlon.latitude;
        ocpnPoint.longitude_deg = latlon.longitude;
        ocpnPoint.velocity = v.norm();
        sampledPoints.push_back(ocpnPoint);
    }

    sampledTrajectory.start_time = startTime;
    sampledTrajectory.dt = dt;
    return sampledTrajectory;
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
void OCPNInterfaceImpl::updateAIS(ais_base::Position const& position)
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
void OCPNInterfaceImpl::updateAIS(ais_base::VesselInformation const& vessel)
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
    ais_vessel.set_draught(vessel.draft);
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
