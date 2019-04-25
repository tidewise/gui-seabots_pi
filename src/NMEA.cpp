#include "NMEA.hpp"
#include <iomanip>
#include <sstream>

using namespace std;
using namespace seabots_pi;

static double headingToNMEA(base::Angle angle)
{
    double deg = angle.getDeg();
    if (deg < 0)
        deg += 360;

    return deg;
}

string nmea::createVTG(
    base::Angle track_true_north,
    base::Angle track_magnetic,
    double speed)
{
    if (speed < 0) {
        track_true_north.flip();
        track_magnetic.flip();
        speed = -speed;
    }

    double track_true_north_nmea = headingToNMEA(track_true_north);
    double track_magnetic_nmea = headingToNMEA(track_magnetic);
    double speed_kmh = speed * 3.6;

    ostringstream str;
    str << "SBVTG,"
        << fixed << setprecision(2) << track_true_north_nmea << ",T,"
        << fixed << setprecision(2) << track_magnetic_nmea << ",M,"
        << fixed << setprecision(2) << (speed_kmh / KNOT_TO_KMH) << ",N,"
        << fixed << setprecision(2) << speed_kmh << ",K";

    return nmea::createPackage(str.str());
}

string nmea::createHDT(base::Angle heading)
{
    ostringstream str;
    str << "SBHDT," << headingToNMEA(heading) << ",T";
    return createPackage(str.str());
}

string nmea::createRMC(
    base::Time const& time, base::Angle latitude, base::Angle longitude,
    double speed_over_ground, base::Angle track, base::Angle magnetic_variation)
{
    auto time_and_date = getTimeAndDate(time);

    ostringstream str;
    str << "SBRMC,"
        << time_and_date.first << ",A,"
        << getAngleDMS(latitude, 2)  << "," << (latitude.getRad() > 0 ? "N," : "S,")
        << getAngleDMS(longitude, 3) << "," << (longitude.getRad() > 0 ? "E," : "W,")
        << fixed << setprecision(1)
        << speed_over_ground * MS_TO_KNOT << ","
        << headingToNMEA(track) << ","
        << time_and_date.second << ","
        << fabs(magnetic_variation.getDeg()) << "," << (magnetic_variation.getRad() > 0 ? "E" : "W");

    return createPackage(str.str());
}

string nmea::createPackage(string payload)
{
    string checksum = nmea::computeChecksum(payload);
    return "$" + payload + "*" + checksum;
}

string nmea::computeChecksum(string payload)
{
    uint8_t checksum = 0;
    for (unsigned int i = 0; i < payload.length(); ++i)
    {
        checksum = checksum ^ payload[i];
    }

    ostringstream str;
    str << uppercase << hex << static_cast<unsigned int>(checksum);
    return str.str();
}

pair<string, string> nmea::getTimeAndDate(base::Time const& time)
{
    time_t unix_time = time.toMicroseconds() / 1000000;
    struct tm* utc_time = gmtime(&unix_time);

    char buffer[32];
    strftime(buffer, 32, "%H%M%S.", utc_time);
    int centisecs = (time.toMilliseconds() / 10) % 100;
    snprintf(buffer + 7, 32 - 7, "%02i", centisecs);
    string hhmmss(buffer);

    strftime(buffer, 32, "%d%m%y", utc_time);
    string ddmmyy(buffer);
    return make_pair(hhmmss, ddmmyy);
}

string nmea::getAngleDMS(base::Angle const& angle, int dwidth)
{
    double i_as_d;
    double f = modf(fabs(angle.getDeg() * 60), &i_as_d);
    int i = round(i_as_d);

    int d = i / 60;
    int m = i % 60;

    ostringstream str;
    str << setfill('0')
        << setw(dwidth) << d
        << setw(2) << m << "."
        << setw(3) << static_cast<int>(f * 1000);
    return str.str();
}
