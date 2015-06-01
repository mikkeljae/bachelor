/****************************************************************************
 *
 *   Copyright (c) 2012-2015 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file rtklib.cpp
 *
 * Driver for use with RTLIB outputting data in NMEA 0183 protocol.
 * Following the protocol specification of http://fort21.ru/download/NMEAdescription.pdf.
 * Only accepts $--GGA messages.
 *
 * Based upon the ashtech driver in ashtech.cpp and ashtech.h.
 *
 * @author Mikkel Skaarup Jaedicke <mijae12@student.sdu.dk>
 *
 */



#include "rtklib.h"

#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <poll.h>
#include <math.h>
#include <string.h>
#include <assert.h>
#include <systemlib/err.h>
#include <uORB/uORB.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/satellite_info.h>
#include <drivers/drv_hrt.h>

#include <fcntl.h>
#include <math.h>


#define HEXDIGIT_CHAR(d) ((char)((d) + (((d) < 0xA) ? '0' : 'A'-0xA)))

RTKLIB::RTKLIB(const int &fd, struct vehicle_gps_position_s *gps_position):
	_fd(fd),
	_gps_position(gps_position)
{
        decode_init();
}

RTKLIB::~RTKLIB()
{
}


int
RTKLIB::handle_message(int len)
{
    char *end;
    // If the message is to short
    if (len < 7) { return 0; }

    // Number of commas
    int no_comma = 0;

    // Count the number of commas in the message
    for (int i = 0 ; i < len; i++) {
        if (_rx_buffer[i] == ',') { no_comma++; }
    }

    char *buffer_ptr;

    if ((memcmp(_rx_buffer+3, "GGA,", 3) == 0) && (no_comma == 14)) {        // If it is a $--GGA message
            /* Below is a description of a NMEA 0182 GGA message from RTKLIB
            $GPGGA,141452.00,5522.4094077,N,01023.8631003,E,1,08,1.0,40.809,M,39.170,M,0.0,*73
            Where:
            GGA             Global Positioning System Fix Data
            141452.00       Fix taken at 12:35:19:00 UTC
            5522.4094077    Latitude 55 deg 22.4094077' N
            01023.8631003,E Longitude 10 deg 023.8631003' E
            Fix quality:    0 = invalid
                            1 = Single
                            2 = DGPS
                            3 =
                            4 = Float Real Time Kinematic
                            5 = Fix RTK
            08              Number of satellites being tracked
            1.0             Horizontal dilution of position
            40.809,M        Antenna Altitude above/below mean-sea-level (geoid)
            39.170,M        Units of antenna altitude, meters
            0.0 time in seconds since last DGPS update
            (empty field) DGPS station ID number
            *73          the checksum data, always begins with *
            **/

        int fix,sat;
        double lat_full, lon_full,alt_sea;

        buffer_ptr = (char *)(_rx_buffer + 7);  //Make buffer pointer point to first information field

        // Get information from message
        strtod(buffer_ptr,&end);                //UTC time information is here - but is not used
        buffer_ptr = end+1;

        lat_full = strtod(buffer_ptr ,&end);    //Latitude information
        buffer_ptr = end+3;

        lon_full = strtod(buffer_ptr ,&end);    //Longitude information
        buffer_ptr = end+3;

        fix = strtol(buffer_ptr ,&end,10);      //Fix information
        end++;
        buffer_ptr = end;

        sat = strtol(buffer_ptr ,&end,10);      //Satellites information
        buffer_ptr = end+1;

        strtod(buffer_ptr,&end);                // hdop (relative) information is here - but is not used
        buffer_ptr = end+1;

        alt_sea = strtod(buffer_ptr,&end);      //Antenna altitude


        // Update _gps_position with information
        _gps_position->timestamp_position = hrt_absolute_time();        //Timestamp for position information

        // Convert lat_full and lon_full to latitude and longitude in degrees
        double lat_deg,lon_deg,lat_min,lon_min;

        lat_deg = floor(lat_full/100);
        lon_deg = floor(lon_full/100);

        lat_min = lat_full-lat_deg*100;
        lon_min = lon_full-lon_deg*100;

        lat_deg += lat_min/60;
        lon_deg += lon_min/60;

        // Update _gps_position
        _gps_position->lat = int32_t(lat_deg*1e7);                          //Latitude  in 1E-7 degrees
        _gps_position->lon = int32_t(lon_deg*1e7);                          //Longitude in 1E-7 degrees

        // Update _gps_position
        _gps_position->alt = int32_t(alt_sea*1e3);                          //Altitude in 1E-3 meters (millimeters) above MSL
        _gps_position->timestamp_variance = hrt_absolute_time();            //Ashtech driver does this....

        // Convert fix to _gps_position format
        if(fix==1){
            fix = 2;
        }
        else if(fix == 4){
            fix = 6;
        }


        // Update _gps_position
        _gps_position->fix_type = fix ;                                     //< 0-1: no fix, 2: 2D fix, 3: 3D fix, 4: RTCM code differential, 5: Real-Time Kinematic, float,
                                                                            //6: Real-Time Kinematic, fixed, 8: Extrapolated
        _gps_position->satellites_used = sat;

        // Update _gps_position like Ashtech driver does it...
        _gps_position->vel_m_s = 0;                                  /**< GPS ground speed (m/s) */
        _gps_position->vel_n_m_s = 0;                                /**< GPS ground speed in m/s */
        _gps_position->vel_e_m_s = 0;                                /**< GPS ground speed in m/s */
        _gps_position->vel_d_m_s = 0;                                /**< GPS ground speed in m/s */
        _gps_position->cog_rad = 0;                                  /**< Course over ground (NOT heading, but direction of movement) in rad, -PI..PI */
        _gps_position->vel_ned_valid = true;                         /**< Flag to indicate if NED speed is valid */
        _gps_position->c_variance_rad = 0.1f;
        _gps_position->timestamp_velocity = hrt_absolute_time();

        return 1;
    }


    return 0;
}



int RTKLIB::receive(unsigned timeout)
{
        {
		/* poll descriptor */
		pollfd fds[1];
		fds[0].fd = _fd;
		fds[0].events = POLLIN;

		uint8_t buf[32];

		/* timeout additional to poll */
		uint64_t time_started = hrt_absolute_time();

		int j = 0;
		ssize_t bytes_count = 0;

		while (true) {

			/* pass received bytes to the packet decoder */
			while (j < bytes_count) {
				int l = 0;

				if ((l = parse_char(buf[j])) > 0) {
					/* return to configure during configuration or to the gps driver during normal work
					 * if a packet has arrived */
                                        if (handle_message(l) > 0) {
						return 1;
					}
				}

				/* in case we keep trying but only get crap from GPS */
				if (time_started + timeout * 1000 * 2 < hrt_absolute_time()) {
					return -1;
				}

				j++;
			}

			/* everything is read */
			j = bytes_count = 0;

			/* then poll for new data */
			int ret = ::poll(fds, sizeof(fds) / sizeof(fds[0]), timeout * 2);

			if (ret < 0) {
				/* something went wrong when polling */
				return -1;

			} else if (ret == 0) {
				/* Timeout */
				return -1;

			} else if (ret > 0) {
				/* if we have new data from GPS, go handle it */
				if (fds[0].revents & POLLIN) {
					/*
					 * We are here because poll says there is some data, so this
					 * won't block even on a blocking device.  If more bytes are
					 * available, we'll go back to poll() again...
					 */
					bytes_count = ::read(_fd, buf, sizeof(buf));
				}
			}
		}
	}

}

int RTKLIB::parse_char(uint8_t b){
        int iRet = 0;

        switch (_decode_state) {
                /* First, look for sync1 */
        case NMEA_DECODE_UNINIT:
                if (b == '$') {
                        _decode_state = NMEA_DECODE_GOT_SYNC1;
                        _rx_buffer_bytes = 0;
                        _rx_buffer[_rx_buffer_bytes++] = b;
                }

                break;

        case NMEA_DECODE_GOT_SYNC1:
                if (b == '$') {
                        _decode_state = NMEA_DECODE_GOT_SYNC1;
                        _rx_buffer_bytes = 0;

                } else if (b == '*') {
                        _decode_state = NMEA_DECODE_GOT_ASTERIKS;
                }

                if (_rx_buffer_bytes >= (sizeof(_rx_buffer) - 5)) {
                        _decode_state = NMEA_DECODE_UNINIT;
                        _rx_buffer_bytes = 0;

                } else {
                        _rx_buffer[_rx_buffer_bytes++] = b;
                }

                break;

        case NMEA_DECODE_GOT_ASTERIKS:
                _rx_buffer[_rx_buffer_bytes++] = b;
                _decode_state = NMEA_DECODE_GOT_FIRST_CS_BYTE;
                break;

        case NMEA_DECODE_GOT_FIRST_CS_BYTE:
                _rx_buffer[_rx_buffer_bytes++] = b;
                uint8_t checksum = 0;
                uint8_t *buffer = _rx_buffer + 1;
                uint8_t *bufend = _rx_buffer + _rx_buffer_bytes - 3;

                for (; buffer < bufend; buffer++) { checksum ^= *buffer; }

                if ((HEXDIGIT_CHAR(checksum >> 4) == *(_rx_buffer + _rx_buffer_bytes - 2)) &&
                    (HEXDIGIT_CHAR(checksum & 0x0F) == *(_rx_buffer + _rx_buffer_bytes - 1))) {
                        iRet = _rx_buffer_bytes;
                }

                _decode_state = NMEA_DECODE_UNINIT;
                _rx_buffer_bytes = 0;
                break;
        }

        return iRet;
}



void RTKLIB::decode_init(void)
{
    _decode_state = NMEA_DECODE_UNINIT;
    _rx_buffer_bytes = 0;
}

int RTKLIB::configure(unsigned &baudrate)
{
	/* try different baudrates */
	const unsigned baudrates_to_try[] = {9600, 38400, 19200, 57600, 115200};


	for (unsigned int baud_i = 0; baud_i < sizeof(baudrates_to_try) / sizeof(baudrates_to_try[0]); baud_i++) {
		baudrate = baudrates_to_try[baud_i];
                set_baudrate(_fd, baudrate);
	}

	set_baudrate(_fd, 115200);
	return 0;
}
