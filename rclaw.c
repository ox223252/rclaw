
#include <stdint.h>
#include <errno.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <fcntl.h>
#include <string.h>
#include <endian.h>

#include "rclaw.h"

//Calculates CRC16 of nBytes of data in byte array message 
static inline uint16_t crc16( uint8_t *packet, uint32_t nBytes )
{
	uint16_t crc = 0;
	for ( uint32_t byte = 0; byte < nBytes; byte++ )
	{
		crc = crc ^ ((uint8_t)packet[byte] << 8);
		for ( uint8_t bit = 0; bit < 8; bit++ )
		{
			if (crc & 0x8000)
			{
				crc = (crc << 1) ^ 0x1021;
			}
			else
			{
				crc = crc << 1;
			}
		}
	}
	return crc;
}

static inline void decode32 ( uint8_t *buffer, uint32_t value )
{
	buffer[0] = (uint8_t)(value >> 24); //High byte
	buffer[1] = (uint8_t)(value >> 16);
	buffer[2] = (uint8_t)(value >> 8);
	buffer[3] = (uint8_t)(value); //Low byte
}

static inline void decode16 ( uint8_t *buffer, uint16_t value )
{
	buffer[0] = (uint8_t)(value >> 8); //High byte
	buffer[1] = (uint8_t)(value); //Low byte
}

int rclawReadWriteData ( const int fd, const PACKAGE_CMD cmd, void * const restrict data, size_t size )
{
	if ( fd <= 0 )
	{
		errno = EINVAL;
		return ( -1 );
	}

	switch ( cmd )
	{
		case RESETS_ENCODER_REGISTERS_FOR_M1_AND_M2:
		case RESTORE_DEFAULTS:
		case WRITE_SETTINGS_TO_EEPROM:
		{
			uint8_t b[ 2 ];
			b[ 0 ] = 0x80;
			b[ 1 ] = cmd;

			if ( write ( fd, b, 2 ) != ( 2 ) )
			{
				return ( __LINE__ );
			}

			struct timeval tv = { .tv_sec = 0, .tv_usec = 10000 };

			fd_set rfds;
			FD_ZERO( &rfds );
			FD_SET( fd, &rfds ); // watch fd, wait something incomming

			int rt = select( ( fd + 1 ), &rfds, NULL, NULL, &tv );

			if ( rt == -1 )
			{ // error
				return ( __LINE__ );
			}
			else if ( rt )
			{ // data avilable
				uint8_t in[ 48 ];

				read ( fd, in, 48 );
			}
			else
			{ // no data available
				errno = ENODATA;
				return ( 0 );
			}
			break;
		}
		case DRIVE_FORWARD_MOTOR_1:
		case DRIVE_BACKWARDS_MOTOR_1:
		case SET_MAIN_VOLTAGE_MINIMUM:
		case SET_MAIN_VOLTAGE_MAXIMUM:
		case DRIVE_FORWARD_MOTOR_2:
		case DRIVE_BACKWARDS_MOTOR_2:
		case DRIVE_MOTOR_1:
		case DRIVE_MOTOR_2:
		case DRIVE_FORWARD_MIXED_MODE:
		case DRIVE_BACKWARDS_MIXED_MODE:
		case TURN_RIGHT_MIXED_MODE:
		case TURN_LEFT_MIXED_MODE:
		case DRIVE_FORWARD_OR_BACKWARD:
		case TURN_LEFT_OR_RIGHT:
		case SET_ENCODER_1_REGISTER:
		case SET_ENCODER_2_REGISTER:
		case SET_MINIMUM_LOGIC_VOLTAGE_LEVEL:
		case SET_MAXIMUM_LOGIC_VOLTAGE_LEVEL:
		case SET_VELOCITY_PID_CONSTANTS_FOR_M1:
		case SET_VELOCITY_PID_CONSTANTS_FOR_M2:
		case DRIVE_M1_WITH_SIGNED_DUTY_CYCLE:
		case DRIVE_M2_WITH_SIGNED_DUTY_CYCLE:
		case DRIVE_M1_M2_WITH_SIGNED_DUTY_CYCLE:
		case DRIVE_M1_WITH_SIGNED_SPEED:
		case DRIVE_M2_WITH_SIGNED_SPEED:
		case DRIVE_M1_M2_WITH_SIGNED_SPEED:
		case DRIVE_M1_WITH_SIGNED_SPEED_AND_ACCELERATION:
		case DRIVE_M2_WITH_SIGNED_SPEED_AND_ACCELERATION:
		case DRIVE_M1_M2_WITH_SIGNED_SPEED_AND_ACCELERATION:
		case DRIVE_M1_WITH_SIGNED_SPEED_AND_DISTANCE_BUFFERED:
		case DRIVE_M2_WITH_SIGNED_SPEED_AND_DISTANCE_BUFFERED:
		case DRIVE_M1_M2_WITH_SIGNED_SPEED_AND_DISTANCE_BUFFERED:
		case DRIVE_M1_WITH_SIGNED_SPEED_ACCELERATION_AND_DISTANCE_BUFFERED:
		case DRIVE_M2_WITH_SIGNED_SPEED_ACCELERATION_AND_DISTANCE_BUFFERED:
		case DRIVE_M1_M2_WITH_SIGNED_SPEED_ACCELERATION_AND_DISTANCE_BUFFERED:
		case DRIVE_M1_M2_WITH_INDIVIDUAL_SIGNED_SPEED_AND_ACCELERATION:
		case DRIVE_M1_M2_WITH_INDIVIDUAL_SIGNED_SPEED_ACCEL_AND_DISTANCE:
		case DRIVE_M1_WITH_SIGNED_DUTY_AND_ACCEL:
		case DRIVE_M2_WITH_SIGNED_DUTY_AND_ACCEL:
		case DRIVE_M1_M2_WITH_SIGNED_DUTY_AND_ACCEL:
		case SET_MAIN_BATTERY_VOLTAGES:
		case SET_LOGIC_BATTERY_VOLTAGES:
		case SET_POSITION_PID_CONSTANTS_FOR_M1:
		case SET_POSITION_PID_CONSTANTS_FOR_M2:
		case DRIVE_M1_WITH_SPEED_ACCEL_DECCEL_AND_POSITION:
		case DRIVE_M2_WITH_SPEED_ACCEL_DECCEL_AND_POSITION:
		case DRIVE_M1_M2_WITH_SPEED_ACCEL_DECCEL_AND_POSITION:
		case SET_DEFAULT_DUTY_CYCLE_ACCELERATION_FOR_M1:
		case SET_DEFAULT_DUTY_CYCLE_ACCELERATION_FOR_M2:
		case SET_S3_S4_AND_S5_MODES:
		case SET_DEADBAND_FOR_RC_ANALOG_CONTROLS:
		case SET_MOTOR_1_ENCODER_MODE:
		case SET_MOTOR_2_ENCODER_MODE:
		case SET_STANDARD_CONFIG_SETTINGS:
		case SET_CTRL_MODES:
		case SET_CTRL1:
		case SET_CTRL2:
		case SET_M1_MAXIMUM_CURRENT:
		case SET_M2_MAXIMUM_CURRENT:
		case SET_PWM_MODE:
		{
			if ( !data ||
				!size )
			{
				errno = EINVAL;
				return ( __LINE__ );
			}

			uint8_t b[ size + 4 ];
			b[ 0 ] = 0x80;
			b[ 1 ] = cmd;

			size_t length = 0;
			while ( length < size )
			{
				b[ length + 2 ] = ((uint8_t*)data)[ length ];
				length++;
			}
			length += 2;

			uint16_t r = crc16( b, length );
			*((uint16_t*)&b[ length ]) = htobe16( r );

			if ( write ( fd, b, length ) != (int)( length ) )
			{
				return ( __LINE__ );
			}

			struct timeval tv = { .tv_sec = 0, .tv_usec = 10000 };

			fd_set rfds;
			FD_ZERO( &rfds );
			FD_SET( fd, &rfds ); // watch fd, wait something incomming

			int rt = select( ( fd + 1 ), &rfds, NULL, NULL, &tv );

			if ( rt == -1 )
			{ // error
				return ( __LINE__ );
			}
			else if ( rt )
			{ // data avilable
				uint8_t in[ 48 ];

				read ( fd, in, 48 );
			}
			else
			{ // no data available
				errno = ENODATA;
				return ( 0 );
			}
			break;
		}
		case READ_ENCODER_COUNT_VALUE_FOR_M1:
		case READ_ENCODER_COUNT_VALUE_FOR_M2:
		case READ_M1_SPEED_IN_ENCODER_COUNTS_PER_SECOND:
		case READ_M2_SPEED_IN_ENCODER_COUNTS_PER_SECOND:
		case READ_CURRENT_M1_RAW_SPEED:
		case READ_CURRENT_M2_RAW_SPEED:
		case READ_FIRMWARE_VERSION:
		case READ_MAIN_BATTERY_VOLTAGE:
		case READ_LOGIC_BATTERY_VOLTAGE:
		case READ_BUFFER_LENGTH:
		case READ_MOTOR_1_VELOCITY_PID_CONSTANTS:
		case READ_MOTOR_2_VELOCITY_PID_CONSTANTS:
		case READ_MAIN_BATTERY_VOLTAGE_SETTINGS:
		case READ_LOGIC_BATTERY_VOLTAGE_SETTINGS:
		case READ_MOTOR_1_POSITION_PID_CONSTANTS:
		case READ_MOTOR_2_POSITION_PID_CONSTANTS:
		case READ_MOTOR_PWMS:
		case READ_MOTOR_CURRENTS:
		case READ_S3_S4_AND_S5_MODES:
		case READ_DEADBAND_FOR_RC_ANALOG_CONTROLS:
		case READ_ENCODERS_COUNTS:
		case READ_MOTOR_SPEEDS:
		case READ_DEFAULT_DUTY_CYCLE_ACCELERATIONS:
		case READ_TEMPERATURE:
		case READ_TEMPERATURE_2:
		case READ_STATUS:
		case READ_ENCODER_MODES:
		case READ_SETTINGS_FROM_EEPROM:
		case READ_STANDARD_CONFIG_SETTINGS:
		case READ_CTRL_MODES:
		case READ_CTRLS:
		case READ_M1_MAXIMUM_CURRENT:
		case READ_M2_MAXIMUM_CURRENT:
		case READ_PWM_MODE:
		{
			uint8_t b[ 50 ] = {0};
			b[ 0 ] = 0x80;
			b[ 1 ] = cmd;

			if ( write ( fd, b, 2 ) != 2 )
			{
				return ( __LINE__ );
			}


			fd_set rfds;
			FD_ZERO( &rfds );
			FD_SET( fd, &rfds ); // watch fd, wait something incomming

			// data avilable
			size = read ( fd, b+2, size + 2 );

			if ( size < 2 )
			{
				errno = EPROTO;
				return ( __LINE__ );
			}

			if ( cmd == READ_FIRMWARE_VERSION )
			{
				printf ( "-> %d\n", size );
				if ( data )
				{
					strcpy ( (char*)data, b+2 );
					((char*)data)[ strlen(data) - 1 ] = 0; // remove the last '\n' in the returned string
				}
				else
				{
					printf ( "%s\n", b+2 );
				}
			}
			else if ( data )
			{
				if ( be16toh(*(uint16_t*)(&b[ size + 2 ])) != crc16( b, ( size + 2 ) ) )
				{
					printf ( "CRC16 error\n");
					errno = EPROTO;
					return ( __LINE__ );
				}

				memcpy ( data, b+2, size );
			}
			else
			{
				errno = EINVAL;
				return ( __LINE__ );
			}
			break;
		}
		default:
		{
			break;
		}
	}
	return ( 0 );
}

int initLib ( const char * const restrict device )
{
	if ( !device )
	{
		errno = EINVAL;
		return ( -1 );
	}

	return ( open ( device, O_RDWR ) );
}

int rclawReadBattery ( int fd, float *v )
{
	if ( !v )
	{
		errno = EINVAL;
		return ( __LINE__ );
	}

	uint16_t voltage = 0;
	int rt = rclawReadWriteData ( fd, READ_MAIN_BATTERY_VOLTAGE, &voltage, 2 );
	if ( rt )
	{
		return ( rt );
	}

	voltage = be16toh ( voltage );
	*v = (float)voltage/10.0;

	return ( 0 );
}

int __attribute__((weak)) main ( void )
{
	int fd = initLib ( "/dev/ttyACM0" );
	int rt = 0;
	int dataSize = 0;

	if ( fd < 0 )
	{
		printf ( "error : %d\n", __LINE__ );
		return ( __LINE__ );
	}

	char firmware[49] = { 0 };

	rt = rclawReadWriteData ( fd, READ_FIRMWARE_VERSION, firmware, 48 );
	switch ( rt )
	{
		case -1:
		{
			printf ( "Firmware : no data available\n" );
			break;
		}
		case 0:
		{
			printf ( "%s\n", firmware );
			break;
		}
		default:
		{
			printf ( "error : %d\n", __LINE__ );
			return ( __LINE__ );
		}
	}


	float voltage = -1;
	switch ( rclawReadBattery ( fd, &voltage ) )
	{
		case -1:
		{
			printf ( "Voltage      : no data available\n" );
			break;
		}
		case 0:
		{
			printf ( "%.2fV\n", (float)voltage );
			break;
		}
		default:
		{
			printf ( "error : %d\n", __LINE__ );
			return ( __LINE__ );
		}
	}

	uint8_t modes[3];

	rt = rclawReadWriteData ( fd, READ_S3_S4_AND_S5_MODES, modes, 3 );
	switch ( rt )
	{
		case -1:
		{
			printf ( "modes        : no data available\n" );
			break;
		}
		case 0:
		{
			printf ( "modes        : %x %x %xV\n", modes[0], modes[1], modes[2] );
			break;
		}
		default:
		{
			printf ( "error : %d\n", __LINE__ );
			return ( __LINE__ );
		}
	}

	uint32_t encoder[2] = { 1, 1 };

	rt = rclawReadWriteData ( fd, READ_ENCODERS_COUNTS, encoder, 2 * sizeof ( uint32_t ) );
	switch ( rt )
	{
		case -1:
		{
			printf ( "Encoder      : no data available\n" );
			break;
		}
		case 0:
		{
			printf ( "Encoder      : %dp   %dp\n", be32toh ( encoder[0] ), be32toh ( encoder[1] ) );
			break;
		}
		default:
		{
			printf ( "error : %d\n", __LINE__ );
			return ( __LINE__ );
		}
	}

	uint32_t encoderuSpeed[2] = { 1, 1 };

	rt = rclawReadWriteData ( fd, READ_MOTOR_SPEEDS, encoderuSpeed, 2 * sizeof ( uint32_t ) );
	switch ( rt )
	{
		case -1:
		{
			printf ( "EncoderSpeed : no data available\n" );
			break;
		}
		case 0:
		{
			printf ( "EncoderSpeed : %dp/s   %dp/s\n", be32toh ( encoderuSpeed[0] ), be32toh ( encoderuSpeed[1] ) );
			break;
		}
		default:
		{
			printf ( "error : %d\n", __LINE__ );
			return ( __LINE__ );
		}
	}

	return ( 0 );
}
