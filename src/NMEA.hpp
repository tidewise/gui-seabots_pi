#ifndef SEABOTS_PI_NMEA_HPP
#define SEABOTS_PI_NMEA_HPP

#include <string>
#include <base/Angle.hpp>
#include <base/Time.hpp>

namespace seabots_pi {
    namespace nmea {
        /** Convert a knot to km/h by multiplying with this. This is an exact
         * value (the know is defined as 1.852 km/h)
         */
        static const double KNOT_TO_KMH = 1.852;
        static const double MS_TO_KNOT = 3.6 / KNOT_TO_KMH;

        /**
         * Recommended minimum navigation information
         */
        std::string createRMC(
            base::Time const& time, base::Angle latitude, base::Angle longitude,
            double speed_over_ground, base::Angle track, base::Angle magnetic_variation);

        /**
         * Velocities
         */
        std::string createVTG(
            base::Angle track_true_north,
            base::Angle track_magnetic, double speed);

        /**
         * Heading
         */
        std::string createHDT(base::Angle heading);

        /** Add delimiters and checksum to a NMEA payload */
        std::string createPackage(std::string payload);

        /** Compute the checksum of a NMEA payload */
        std::string computeChecksum(std::string payload);

        /** Compute the checksum of a NMEA payload */
        std::pair<std::string, std::string> getTimeAndDate(base::Time const& time);

        /** Convert an angle in ddmm.ss format */
        std::string getAngleDMS(base::Angle const& angle, int dwidth);
    }
}

#endif