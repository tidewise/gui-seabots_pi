#include <gtest/gtest.h>
#include "../src/NMEA.hpp"

using namespace std;
using namespace seabots_pi;

struct NMEATest : public ::testing::Test {
};

TEST_F(NMEATest, it_computes_a_valid_checksum_with_uppercase_letters) {
    ASSERT_EQ(string("2D"), nmea::computeChecksum("PFEC,GPint,RMC05"));
}

TEST_F(NMEATest, it_builds_a_valid_NMEA_package_from_its_checksum) {
    ASSERT_EQ(string("$PFEC,GPint,RMC05*2D"), nmea::createPackage("PFEC,GPint,RMC05"));
}

TEST_F(NMEATest, it_builds_a_VTG_message_with_all_required_info) {
    base::Angle geographic = base::Angle::fromDeg(10.42);
    base::Angle magnetic = base::Angle::fromDeg(15.21);
    double speed = 1.4;

    string msg = nmea::createVTG(geographic, magnetic, speed);
    ASSERT_EQ("$SBVTG,10.42,T,15.21,M,2.72,N,5.04,K*4E", msg);
}

TEST_F(NMEATest, it_adds_360_degrees_to_negative_angles) {
    base::Angle geographic = base::Angle::fromDeg(-10.42);
    base::Angle magnetic = base::Angle::fromDeg(-15.21);
    double speed = 1.4;

    string msg = nmea::createVTG(geographic, magnetic, speed);
    ASSERT_EQ(string("$SBVTG,349.58,T,344.79,M,2.72,N,5.04,K*40"), msg);
}

TEST_F(NMEATest, it_flips_the_course_if_the_velocity_is_negative) {
    base::Angle geographic = base::Angle::fromDeg(10.42);
    base::Angle magnetic = base::Angle::fromDeg(15.21);
    double speed = -1.4;

    string msg = nmea::createVTG(geographic, magnetic, speed);
    ASSERT_EQ(string("$SBVTG,190.42,T,195.21,M,2.72,N,5.04,K*4E"), msg);
}

TEST_F(NMEATest, it_converts_a_time_into_the_UTC_format_required_by_NMEA) {
    base::Time time = base::Time::fromMilliseconds(1556222665123);
    auto converted = nmea::getTimeAndDate(time);

    ASSERT_EQ("200425.12", converted.first);
    ASSERT_EQ("250419", converted.second);
}

TEST_F(NMEATest, it_converts_an_angle_in_DMS_format) {
    base::Angle angle = base::Angle::fromDeg(43.2135634);
    ASSERT_EQ("4312.813", nmea::getAngleDMS(angle, 2));
}

TEST_F(NMEATest, it_zero_pads_the_degree_field_if_needed) {
    base::Angle angle = base::Angle::fromDeg(43.2135634);
    ASSERT_EQ("04312.813", nmea::getAngleDMS(angle, 3));
}

TEST_F(NMEATest, it_zero_pads_the_dms_fields) {
    base::Angle angle = base::Angle::fromDeg(43.018);
    ASSERT_EQ("4301.079", nmea::getAngleDMS(angle, 2));
}

TEST_F(NMEATest, it_formats_the_RMC_message) {
    // See UTC format test above
    base::Time time = base::Time::fromMilliseconds(1556222665123);
    // See DMS format test above
    base::Angle latitude = base::Angle::fromDeg(43.2135634);
    base::Angle longitude = base::Angle::fromDeg(43.018);

    base::Angle track = base::Angle::fromDeg(10.42);

    string msg = nmea::createRMC(time, latitude, longitude, 1.4,
        track, base::Angle::fromDeg(2));

    ASSERT_EQ("$SBRMC,200425.12,A,4312.813,N,04301.079,E,2.7,10.4,250419,2.0,E*63",
        msg);
}

TEST_F(NMEATest, it_goes_south_for_a_negative_latitude) {
    // See UTC format test above
    base::Time time = base::Time::fromMilliseconds(1556222665123);
    // See DMS format test above
    base::Angle latitude = base::Angle::fromDeg(-43.2135634);
    base::Angle longitude = base::Angle::fromDeg(43.018);

    base::Angle track = base::Angle::fromDeg(10.42);

    string msg = nmea::createRMC(time, latitude, longitude, 1.4,
        track, base::Angle::fromDeg(2));

    ASSERT_EQ("$SBRMC,200425.12,A,4312.813,S,04301.079,E,2.7,10.4,250419,2.0,E*7E",
        msg);
}

TEST_F(NMEATest, it_goes_west_for_a_negative_longitude) {
    // See UTC format test above
    base::Time time = base::Time::fromMilliseconds(1556222665123);
    // See DMS format test above
    base::Angle latitude = base::Angle::fromDeg(43.2135634);
    base::Angle longitude = base::Angle::fromDeg(-43.018);

    base::Angle track = base::Angle::fromDeg(10.42);

    string msg = nmea::createRMC(time, latitude, longitude, 1.4,
        track, base::Angle::fromDeg(2));

    ASSERT_EQ("$SBRMC,200425.12,A,4312.813,N,04301.079,W,2.7,10.4,250419,2.0,E*71",
        msg);
}

TEST_F(NMEATest, it_creates_a_heading_message) {
    base::Angle heading = base::Angle::fromDeg(10.42);
    string msg = nmea::createHDT(heading);
    ASSERT_EQ("$SBHDT,10.42,T*34", msg);
}
