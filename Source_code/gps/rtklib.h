
#ifndef RTKLIB_H_
#define RTKLIB_H_

#include "gps_helper.h"

#ifndef RECV_BUFFER_SIZE
#define RECV_BUFFER_SIZE 512
#endif


class RTKLIB : public GPS_Helper
{
	enum nmea_decode_state_t {
		NMEA_DECODE_UNINIT,
		NMEA_DECODE_GOT_SYNC1,
		NMEA_DECODE_GOT_ASTERIKS,
		NMEA_DECODE_GOT_FIRST_CS_BYTE
	};

	int                    		_fd;
	struct vehicle_gps_position_s 	*_gps_position;
	nmea_decode_state_t   		_decode_state;
	uint8_t               		_rx_buffer[RECV_BUFFER_SIZE];
	uint16_t              		_rx_buffer_bytes;
	
public:
	RTKLIB(const int &fd, struct vehicle_gps_position_s *gps_position);
	~RTKLIB();
	int             receive(unsigned timeout);
	int             configure(unsigned &baudrate);
	void            decode_init(void);
	int             handle_message(int len);
	int             parse_char(uint8_t b);

};

#endif /* RTKLIB_H_ */
