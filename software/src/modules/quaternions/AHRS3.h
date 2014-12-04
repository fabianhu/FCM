/*
 * AHRS3.h
 *
 * Created: 03.12.2014 00:04:57
 // ****************************************************************************
 // *** Copyright (c) 2011, Universal Air Ltd. All rights reserved.
 // *** Source and binaries are released under the BSD 3-Clause license
 // *** See readme_forebrain.txt files for the text of the license
 // ****************************************************************************
 */ 


#ifndef AHRS3_H_
#define AHRS3_H_

#define GYRO_AVERAGING  3
#define MODE_X1ORPLUS0 	0
#define AHRS_RATE       100
#define AHRS_CORRECTION AHRS_RATE/20
#define DRIFT_AccelKp   10.0f//storage.param[25]
#define DRIFT_AccelKi   10.0f//storage.param[26]
#define DRIFT_AccelDe   10.0f//storage.param[27]

#define DRIFT_MagKp  	10.0f//storage.param[28]
#define DRIFT_MagKi   	10.0f//storage.param[29]
#define DRIFT_MagDe   	10.0f//storage.param[30]
#define BARO_AVERAGING  100
#define BARO_TEMP_EVERY 50
/*#define LOWERROR        500
#define HIGHERROR       2500
#define MINSTICK        1180
#define MIDSTICK        1530
#define MAXSTICK        1880
#define IDLETHROTTLE    1300
#define YAW_DEADZONE    0.008f
// #define PITCH_DEADZONE  0.5f
// #define ROLL_DEADZONE   0.5f
#define THRUST_DEADZONE 0.5f


#define ESC_PWM         150

#define OFFSET_CALC		100


#define INPUT_AVERAGING 3

#define MAX_WAYPOINTS   20 // TODO: maybe move waypoints into EEPROM
#define WAYPOINT_HOME   MAX_WAYPOINTS
#define WAYPOINT_TEMP   MAX_WAYPOINTS+1
#define WAYPOINT_TIMEOUT    500 // in ms
#define WAYPOINT_TRIES  5   // number of times to retry waypoint
#define WAYPOINT_MIN_RADIUS 1
#define WAYPOINT_SAFE_HEIGHT 2  // altitude above home to use



#define DRIFT_Baro      storage.param[31]
#define GROUNDTHRESH    storage.param[37]
#define DESCENDRATE     storage.param[38]
#define ASCENDRATE      storage.param[39]
#define ALT_Ki          storage.param[40]
#define ALT_Decay       storage.param[41]
#define ALT_Bias        storage.param[42]

#define MODE_SIMPLICITY storage.param[44]
#define HOLD_KP		    storage.param[45]
#define HOLD_LIM		storage.param[46]


#define STORAGE_RATES_START    4*STORAGE_DATA-3
#define STORAGE_RATES   MAV_DATA_STREAM_ENUM_END

#define STORAGE_END     STORAGE_RATES_START + STORAGE_RATES

#define GROUNDED_IS_GROUNDED    2
#define GROUNDED_NOT_GROUNDED   0
#define GROUNDED_MAYBE_GROUNDED 1

#define LASTNAV_UNDEF       255
#define LASTNAV_GUIDED      254

#define GPS_PAYLOAD_SIZE    64

#define JOYSTICK_GENERIC    0
#define JOYSTICK_GAMEPAD    1
#define JOYSTICK_TYPE       JOYSTICK_GAMEPAD

#define MAGIC_REPROGRAM     0xec41
#define MAGIC_SHUTDOWN      0xdb30

#define ONDEMAND_GPS_HOME   1
*/

quaternion_t AHRS3(vector3_t Gyro, vector3_t Accel, vector3_t Mag);



#endif /* AHRS3_H_ */


/*



#include "lpc13xx.h"
#include "uafunc.h"
#include "seraphim.h"
#include "mavlink/include/common/mavlink.h"

#include <math.h>

// ****** Definitions
#define LIFTOFF_ALTITUDE    2

#define ENABLE_RTL      0
#define ARM_WITH_CUSTOM_MODE    1

#define CONSTRAIN       300
#define ENABLE_CONSTRAIN    0

#define LOWERROR        500
#define HIGHERROR       2500
#define MINSTICK        1180
#define MIDSTICK        1530
#define MAXSTICK        1880
#define IDLETHROTTLE    1300
#define YAW_DEADZONE    0.008f
// #define PITCH_DEADZONE  0.5f
// #define ROLL_DEADZONE   0.5f
#define THRUST_DEADZONE 0.5f

#define AHRS_RATE       100
#define AHRS_CORRECTION AHRS_RATE/20
#define ESC_PWM         150

#define OFFSET_CALC		100
#define GYRO_AVERAGING  3
#define BARO_AVERAGING  100
#define BARO_TEMP_EVERY 50
#define INPUT_AVERAGING 3

#define MAX_WAYPOINTS   20 // TODO: maybe move waypoints into EEPROM
#define WAYPOINT_HOME   MAX_WAYPOINTS
#define WAYPOINT_TEMP   MAX_WAYPOINTS+1
#define WAYPOINT_TIMEOUT    500 // in ms
#define WAYPOINT_TRIES  5   // number of times to retry waypoint
#define WAYPOINT_MIN_RADIUS 1
#define WAYPOINT_SAFE_HEIGHT 2  // altitude above home to use

#define EEPROM_VERSION  3
#define STORAGE_DATA    47
#define IDENTITY        storage.param[0]
#define PITCH_Kp        storage.param[1]
#define PITCH_Ki        storage.param[2]
#define PITCH_Kd        storage.param[3]
#define PITCH_Kdd       storage.param[4]
#define PITCH_Boost     storage.param[5]
#define PITCH_De        storage.param[6]
#define PITCH_KpT1      storage.param[7]
#define PITCH_KpT2      storage.param[8]
#define ROLL_Kp         storage.param[9]
#define ROLL_Ki         storage.param[10]
#define ROLL_Kd         storage.param[11]
#define ROLL_Kdd        storage.param[12]
#define ROLL_Boost      storage.param[13]
#define ROLL_De         storage.param[14]
#define ROLL_KpT1       storage.param[15]
#define ROLL_KpT2       storage.param[16]
#define YAW_Kp          storage.param[17]
#define YAW_Ki          storage.param[18]
#define YAW_Kd          storage.param[19]
#define YAW_Kdd         storage.param[20]
#define YAW_Boost       storage.param[21]
#define YAW_De          storage.param[22]
#define YAW_KpT1        storage.param[23]
#define YAW_KpT2        storage.param[24]
#define DRIFT_AccelKp   storage.param[25]
#define DRIFT_AccelKi   storage.param[26]
#define DRIFT_AccelDe   storage.param[27]
#define DRIFT_MagKp  	storage.param[28]
#define DRIFT_MagKi   	storage.param[29]
#define DRIFT_MagDe   	storage.param[30]
#define DRIFT_Baro      storage.param[31]
#define THROTTLE_X1     storage.param[32]
#define THROTTLE_Y1     storage.param[33]
#define THROTTLE_X2     storage.param[34]
#define THROTTLE_Y2     storage.param[35]
#define THROTTLE_Yn     storage.param[36]
#define GROUNDTHRESH    storage.param[37]
#define DESCENDRATE     storage.param[38]
#define ASCENDRATE      storage.param[39]
#define ALT_Ki          storage.param[40]
#define ALT_Decay       storage.param[41]
#define ALT_Bias        storage.param[42]
#define MODE_X1ORPLUS0 	storage.param[43]
#define MODE_SIMPLICITY storage.param[44]
#define HOLD_KP		    storage.param[45]
#define HOLD_LIM		storage.param[46]


#define STORAGE_RATES_START    4*STORAGE_DATA-3
#define STORAGE_RATES   MAV_DATA_STREAM_ENUM_END

#define STORAGE_END     STORAGE_RATES_START + STORAGE_RATES

#define GROUNDED_IS_GROUNDED    2
#define GROUNDED_NOT_GROUNDED   0
#define GROUNDED_MAYBE_GROUNDED 1

#define LASTNAV_UNDEF       255
#define LASTNAV_GUIDED      254

#define GPS_PAYLOAD_SIZE    64

#define JOYSTICK_GENERIC    0
#define JOYSTICK_GAMEPAD    1
#define JOYSTICK_TYPE       JOYSTICK_GAMEPAD

#define MAGIC_REPROGRAM     0xec41
#define MAGIC_SHUTDOWN      0xdb30

#define ONDEMAND_GPS_HOME   1

// ****** System
unsigned char manual, startup;

// ****** Input and outputs
typedef struct{
	unsigned short duty;
	unsigned char port;
	unsigned short pin;
} outputStruct;

outputStruct motor[4];

typedef struct{
	unsigned char state;
	unsigned int timer;
	unsigned short duty;
	unsigned short offset;
	unsigned short history[INPUT_AVERAGING];
	unsigned int total;
	unsigned short count;
	unsigned short av;
	unsigned short error;
	unsigned short sync;
	unsigned char port;
	unsigned short pin;
} inputStruct;

inputStruct input[5];
signed short throttle;
unsigned char auxState, risingCatch;

// ****** Sensors

typedef struct{
	float value;
	signed short raw;
	signed short history[GYRO_AVERAGING];
	signed int total;
	float av;
	float offset;
	float si;
	float error;
	float verterror;
	float verterrorInt;
} sensorStruct;

typedef struct{
	sensorStruct X;
	sensorStruct Y;
	sensorStruct Z;
	unsigned short count;
} threeAxisSensorStruct;

threeAxisSensorStruct Gyro;
threeAxisSensorStruct Accel;
threeAxisSensorStruct Magneto;

typedef struct{
	unsigned short payloadLength;
	unsigned char checksumA;
	unsigned char checksumB;
	unsigned short payloadCount;
	unsigned char id1;
	unsigned char id2;
	unsigned char state;
	unsigned char payload[GPS_PAYLOAD_SIZE];
	
	signed int longitude;
	signed int latitude;
	signed int altitude;
	unsigned int hAcc;
	unsigned int vAcc;
	unsigned int speedGround;
	unsigned int speed3D;
	signed int heading;

	unsigned char firstFix;
	unsigned char fix;
	unsigned char hasHome;
} GPSStruct;

GPSStruct GPS;



BaroStruct Baro;

// ****** AHRS

typedef struct{
	float demand;
	float demandOld;
	float demandtemp;
	float gyroOld;
	float derivative;
	float integral;
} directionStruct;

directionStruct pitch;
directionStruct roll;
directionStruct yaw;
directionStruct drift;

float thetaAngle, phiAngle, psiAngle;

typedef struct{
	float u;
	float a;
	float i;
	float r;
} quaternionStruct;

quaternionStruct quaternion;
quaternionStruct quaternionOld;

float altitude, lastAltitude;

// ****** Throttle non-lineariser (RC control only)
typedef struct {
	unsigned short x1;
	unsigned short y1;
	unsigned short x2;
	unsigned short y2;
	unsigned short yn;
} NonLinearStruct;

NonLinearStruct throttlePoint;

// ****** Counters

unsigned short loopCounter50Hz, loopCounter1Hz, loopCounter20Hz;
unsigned int sysTime;
unsigned short ahrsSoftscale, ahrsBarometer;

// ****** Mavlink
mavlink_heartbeat_t mavlink_heartbeat;
mavlink_system_t mavlink_system;
mavlink_status_t mavlink_status;
mavlink_set_mode_t mavlink_set_mode;
mavlink_command_long_t mavlink_command_long;
mavlink_command_ack_t mavlink_command_ack;

mavlink_attitude_t mavlink_attitude;
mavlink_vfr_hud_t mavlink_vfr_hud;
mavlink_gps_raw_int_t mavlink_gps_raw_int;
mavlink_gps_global_origin_t mvalink_gps_global_origin;
// mavlink_safety_allowed_area_t mavlink_safety_allowed_area;

mavlink_raw_imu_t mavlink_raw_imu;
mavlink_raw_pressure_t mavlink_raw_pressure;
mavlink_rc_channels_raw_t mavlink_rc_channels_raw;
mavlink_servo_output_raw_t mavlink_servo_output_raw;
mavlink_roll_pitch_yaw_thrust_setpoint_t mavlink_roll_pitch_yaw_thrust_setpoint;
// mavlink_nav_controller_output_t mavlink_nav_controller_output;

mavlink_manual_control_t mavlink_manual_control;
mavlink_manual_control_t mavlink_manual_control_valid;

mavlink_request_data_stream_t mavlink_request_data_stream;
mavlink_data_stream_t mavlink_data_stream;

mavlink_param_value_t mavlink_param_value;
mavlink_param_set_t mavlink_param_set;

mavlink_statustext_t mavlink_statustext;
mavlink_debug_vect_t mavlink_debug_vect;

mavlink_message_t mavlink_rx_msg;
mavlink_message_t mavlink_tx_msg;

mavlink_mission_request_list_t mavlink_mission_request_list;
mavlink_mission_count_t mavlink_mission_count;
mavlink_mission_request_t mavlink_mission_request;
mavlink_mission_item_t mavlink_mission_item;
mavlink_mission_ack_t mavlink_mission_ack;
mavlink_mission_clear_all_t mavlink_mission_clear_all;
mavlink_mission_set_current_t mavlink_mission_set_current;
mavlink_mission_current_t mavlink_mission_current;
// mavlink_mission_item_reached_t mavlink_mission_item_reached;

unsigned char dataRate[STORAGE_RATES];
unsigned short dataCount[STORAGE_RATES];
unsigned short dataOnDemand;

unsigned char mavlink_message_buf[MAVLINK_MAX_PACKET_LEN];
unsigned short mavlink_message_len;
unsigned char lastNav;

void heartbeat(void);
void setHomeFloat(float latitude, float longitude, float altitude);
void setHomeInt(signed int latitude, signed int longitude, signed int altitude);
void mavlinkSendText(unsigned char severity, char * text);
void mavlinkSendDebugV(char * name, float x, float y, float z);

// ****** waypoints
typedef struct {
	unsigned char command;
	unsigned char autocontinue;
	float param1;
	float param2;
	float param3;
	float param4;
	float x;
	float y;
	float z;
} waypointStruct;

waypointStruct waypoint[MAX_WAYPOINTS+2];

unsigned short waypointCurrent, waypointCount, waypointReceiveIndex;
unsigned char waypointTries, waypointValid;
unsigned short waypointTimer;
unsigned char waypointProviderID, waypointProviderComp;

// ***** Parameter
typedef struct {
	float param[STORAGE_DATA];
	char name[STORAGE_DATA][MAVLINK_MSG_PARAM_SET_FIELD_PARAM_ID_LEN];
	unsigned short sendIndex;
} storageStruct;

storageStruct storage;

// ***** Functions
signed short nonLinearThrottle(unsigned short input, unsigned short offset);
void loadStorage(void);
void saveStorage(void);

void configGPS(void);

void calibrate(void);
void calibrateGyro(void);
void calibrateRC(void);
void calibrateBaro(void);
void AHRSStop(void);
void AHRSStart(void);

unsigned char isGrounded(void);
unsigned char groundTest;

float invSqrt(float x);
float fatan2(float x, float y);
float fasin(float x);
float fsin(float x);

// ****************************************************************************
// *** Setup and initialisation
// ****************************************************************************

void setup() {
	unsigned int i;
	float tempAccelKp, tempMagKp;

	// *** Startup hax for Seraphim's ublox EEPROM problem
	GPSWorkaround();
	
	// *** System shutdown and reprogram support
	if(ReadBootData4() == MAGIC_REPROGRAM) {
		// Enter Programming mode if Commanded
		WriteBootData4(0);
		SysTickStop();
		Reprogram();
	}
	else if(ReadBootData4() == MAGIC_SHUTDOWN) {
		WriteBootData4(0);
		SysTickStop();
		while(1); // alternatively use low-power shutdown, but that's a waste of flash space
	}
	
	// Initialising LEDs
	LEDInit(ALL);
	LEDOn(ALL);
	
	startup = 1;
	
	Delay(4000);
	I2CInit(100, MASTER);
	
	LEDWrite(1);
	
	// Mavlink
	mavlink_system.sysid = EEPROMReadByte(0);
	if(mavlink_system.sysid == 255) mavlink_system.sysid = 0; // ID of 255 reserved for GCS
	mavlink_system.compid = MAV_COMP_ID_SYSTEM_CONTROL;
	mavlink_system.type = MAV_TYPE_QUADROTOR;
	
	mavlink_heartbeat.type = mavlink_system.type;
	mavlink_heartbeat.autopilot = MAV_AUTOPILOT_GENERIC;
	mavlink_heartbeat.mavlink_version = MAVLINK_VERSION;
	
	mavlink_heartbeat.base_mode = MAV_MODE_PREFLIGHT;
	mavlink_heartbeat.system_status = MAV_STATE_BOOT;

	waypointCount = 0;
	waypointValid = 0;
	waypointTimer = 0;
	
	// Initialise UART for telemetry
	UARTInit(115200);
	heartbeat();
	LEDWrite(2);
	
	// Initialise Timers
	loopCounter50Hz = 0;
	loopCounter1Hz = 1000;  // start this counter going
	loopCounter20Hz = 7;    // start at non-zero to stagger loops
	ahrsSoftscale = 0;
	ahrsBarometer = 1;
	sysTime = 0;

	for(i=0; i<STORAGE_RATES; i++) {
		dataCount[i] = 0;
	}
	dataOnDemand = 0;
	
	SysTickInit();
	
	LEDWrite(3);
	
	// *** Sensor initialisation
	GPSInit();
	GyroInit();
	AccelInit();
	BaroInit();
	MagnetoInit();
	configGPS();
	
	LEDWrite(4);
	
	// *** Initialise PWM inputs
	// Enable clock for Timer16B0, this timer is used to count PWMs
	Timer0Init(71);
	
	LEDWrite(5);
	
	// Set input duties
	input[0].pin = (unsigned short)PIN0;
	input[1].pin = (unsigned short)PIN1;
	input[2].pin = (unsigned short)PIN2;
	input[3].pin = (unsigned short)PIN4;
	input[4].pin = (unsigned short)PIN5;
	input[0].sync = 10;  // variables used as watchdog for inputs
	input[1].sync = 10;
	input[2].sync = 10;
	input[3].sync = 10;
	input[4].sync = 10;

	for(i=0; i<INPUT_AVERAGING; i++) {
		input[0].history[i] = 0;
		input[1].history[i] = 0;
		input[2].history[i] = 0;
		input[3].history[i] = 0;
		input[4].history[i] = 0;
	}
	input[0].count = 0;
	input[1].count = 0;
	input[2].count = 0;
	input[3].count = 0;
	input[4].count = 0;
	input[0].total = 0;
	input[1].total = 0;
	input[2].total = 0;
	input[3].total = 0;
	input[4].total = 0;
	
	auxState = 0;
	risingCatch = 0;
	
	// Set up ports
	Port2Init(PIN0 | PIN1 | PIN2 | PIN4 | PIN5);
	Port2SetIn(PIN0 | PIN1 | PIN2 | PIN4 | PIN5);
	// Set up GPIO interrupt
	Port2SetInterrupt(PIN0 | PIN1 | PIN2 | PIN4 | PIN5, BOTH);
	
	// *** Initialise PWM out pins
	// Set up ports
	Port1Init(PIN1 | PIN8 | PIN10);
	Port1SetOut(PIN1 | PIN8 | PIN10);
	Port2Init(PIN6);
	Port2SetOut(PIN6);
	
	LEDWrite(6);
	
	// *** Initialise variables
	loadStorage();
	// Re-program Command Variable
	manual = 0;
	// AHRS
	quaternion.u = 1;
	quaternion.a = 0;
	quaternion.i = 0;
	quaternion.r = 0;
	
	// Angles
	thetaAngle = 0;
	phiAngle = 0;
	psiAngle = 0;

	
	pitch.demand = 0;
	roll.demand = 0;
	yaw.demand = 0;
	throttle = 0;
	
	LEDWrite(7);
	
	// Calibrate
	mavlink_heartbeat.system_status = MAV_STATE_CALIBRATING;
	heartbeat();
	calibrate();
	LEDWrite(8);
	
	// *** Initialise the control loop (nominal 50Hz)
	// No timer init is needed since it uses SysTick
	
	
	// *** Initialise the AHRS timer (nominal > 50Hz)
	// Set high drift gains to rotate AHRS to "true"
	tempAccelKp = DRIFT_AccelKp;
	tempMagKp = DRIFT_MagKp;
	DRIFT_MagKp = 10;
	DRIFT_AccelKp = 10;
	
	Timer1Init(71);
	Timer1Match0((1000000/AHRS_RATE), INTERRUPT | RESET);
	Delay(2000);
	
	LEDWrite(9);
	
	// Restore gains
	DRIFT_MagKp = tempMagKp;
	DRIFT_AccelKp = tempAccelKp;

	LEDWrite(10);
	
	// *** Initialise the motor timer (nominal 50Hz)
	Timer2Init(71);
	Timer2Match0((1000000/ESC_PWM), INTERRUPT | RESET);
	Timer2Match1(1000, INTERRUPT); // Motor
	Timer2Match2(1000, INTERRUPT); // Motor
	Delay(10); // stagger the interrupts
	Timer3Init(71);
	Timer3Match0((1000000/ESC_PWM), INTERRUPT | RESET);
	Timer3Match1(1000, INTERRUPT); // Motor
	Timer3Match2(1000, INTERRUPT); // Motor

	mavlink_heartbeat.system_status = MAV_STATE_STANDBY;
	heartbeat();
	LEDWrite(11);
	
	// Set initial mode if RC detected, start up in Manual mode, otherwise in READY mode, and wait for further instructions
	if(input[3].duty > LOWERROR && input[3].duty < HIGHERROR) {
		mavlinkSendText(0, "RC signal available");
		// Calibrate RC
		calibrateRC();
		mavlink_heartbeat.system_status = MAV_STATE_ACTIVE;
		mavlink_heartbeat.base_mode = MAV_MODE_STABILIZE_ARMED;
		heartbeat();
		manual = 1;
	}
	
	LEDOn(ALL);
	Delay(100);
	LEDOff(ALL);
	startup = 0;
}

// ****************************************************************************
// *** Main loop, bulk commands
// ****************************************************************************

void loop() {
	if(loopCounter50Hz > 20) {
		loopCounter50Hz = 0;
		
		if(mavlink_heartbeat.base_mode & MAV_MODE_FLAG_SAFETY_ARMED && (mavlink_heartbeat.system_status == MAV_STATE_ACTIVE || mavlink_heartbeat.system_status == MAV_STATE_CRITICAL)) {
			signed short motorN, motorE, motorS, motorW, temp;
			float pitcherror, rollerror, yawerror;
			float pitchcorrection, rollcorrection, yawcorrection;

			//float tempf;
			
			// Controller's aux or gear switch
			if(input[4].duty > MIDSTICK && input[4].duty > LOWERROR) {
				if(auxState == 1) {
					// change from low to high
					mavlink_heartbeat.base_mode |= MAV_MODE_FLAG_DECODE_POSITION_GUIDED;
					MODE_SIMPLICITY = 1;
				}
				auxState = 0;
			}
			else {
				if(auxState == 0) {
					// change from high to low
					mavlink_heartbeat.base_mode |= MAV_MODE_FLAG_DECODE_POSITION_MANUAL;
					mavlink_heartbeat.base_mode &= ~MAV_MODE_FLAG_DECODE_POSITION_GUIDED;
					MODE_SIMPLICITY = 0;
					//risingCatch = 1;
				}
				auxState = 1;
			}
			
			
			
			// Manual control
			if(mavlink_heartbeat.base_mode & MAV_MODE_FLAG_DECODE_POSITION_MANUAL) {
				if(manual) {
					#if ENABLE_RTL == 1
					if(input[0].sync == 0 || input[1].sync == 0 || input[2].sync == 0 || input[3].sync == 0) {
						// in manual mode, input desync results in return to launch if home is set
						// TODO: return to launch
						mavlinkSendText(127, "RC signal lost, returning to launch");
						input[0].duty = 0;
						input[1].duty = 0;
						input[2].duty = 0;
						input[3].duty = 0;
						input[4].duty = 0;
					}
					#endif
					
					// *** Flight Control
					// Control Inputs
					
					input[0].av = input[0].total / INPUT_AVERAGING;
					input[1].av = input[1].total / INPUT_AVERAGING;
					input[2].av = input[2].total / INPUT_AVERAGING;
					input[3].av = input[3].total / INPUT_AVERAGING;

					pitch.demand = (MIDSTICK - input[1].av)*0.0016; // 45 degree range on Stick (transmitter range - 1.16-1.80)
					roll.demand = (MIDSTICK - input[0].av)*0.0016;
					
					
					
					// Yaw deadzone
					float tempf = (input[2].av - (float)input[2].offset)*0.0004;//3125;;
					if(fabsf(tempf) > YAW_DEADZONE) {
						yaw.demand += tempf;
						if(yaw.demand > M_PI) {
							yaw.demand -= M_TWOPI;
							yaw.demandOld -= M_TWOPI;
						}
						else if(yaw.demand < -M_PI) {
							yaw.demand += M_TWOPI;
							yaw.demandOld += M_TWOPI;
						}
					}
					throttle = nonLinearThrottle(input[3].av, input[3].offset);
				}
				else {
					pitch.demand = mavlink_manual_control_valid.pitch * 4; // 45 degree range on stick
					roll.demand = mavlink_manual_control_valid.roll * 4; // 45 degree range on stick
					
					float tempf = mavlink_manual_control_valid.yaw / 2;
					
					if(fabsf(tempf) > YAW_DEADZONE) {
						yaw.demand += tempf;
						if(yaw.demand > M_PI) {
							yaw.demand -= M_TWOPI;
							yaw.demandOld -= M_TWOPI;
						}
						else if(yaw.demand < -M_PI) {
							yaw.demand += M_TWOPI;
							yaw.demandOld += M_TWOPI;
						}
					}
					
					#if JOYSTICK_TYPE == JOYSTICK_GAMEPAD
					tempf = (mavlink_manual_control_valid.thrust - 0.5) * 20;
					
					if(fabsf(tempf) > THRUST_DEADZONE) {


						throttle += tempf;
						if(throttle < 0) throttle = 0;
						else if(throttle > (MAXSTICK - MINSTICK)) throttle = (MAXSTICK - MINSTICK);
					}
					
					#else
					throttle = mavlink_manual_control_valid.thrust * (MAXSTICK - MINSTICK);
					#endif
				}
				
				// rising catch to ensure throttle doesn't drop when transitioning from guided to manaul
				// if(risingCatch == 1 && manualThrottle > throttle) {
				// mavlink_heartbeat.base_mode &= ~MAV_MODE_FLAG_DECODE_POSITION_GUIDED;
				// risingCatch = 0;
				// }
			}
			
			// Guided mode
			if(mavlink_heartbeat.base_mode & MAV_MODE_FLAG_DECODE_POSITION_GUIDED) {
				
				
				
				// waypoint[WAYPOINT_HOME].x = 54;
				// waypoint[WAYPOINT_HOME].y = 1;
				
				// GPS.latitude = 55.00001799 * 10000000;
				// GPS.longitude = 2.0000292 * 10000000;
				
				if ((GPS.fix >= 3) && (GPS.hasHome == 1)) {
					pitch.demand = -(waypoint[WAYPOINT_HOME].x - (((float)GPS.latitude)/10000000)) * 111200 * HOLD_KP;
					if(pitch.demand > HOLD_LIM) pitch.demand = HOLD_LIM;
					if(pitch.demand < -HOLD_LIM) pitch.demand = -HOLD_LIM;

					roll.demand = (waypoint[WAYPOINT_HOME].y - (((float)GPS.longitude)/10000000)) * 68460 * HOLD_KP;
					if(roll.demand > HOLD_LIM) roll.demand = HOLD_LIM;
					if(roll.demand < -HOLD_LIM) roll.demand = -HOLD_LIM;
					
					
					// INSERT AUTO YAW HERE
					
				}
				
				throttle = nonLinearThrottle(input[3].av, input[3].offset);
			}
			
			//mavlinkSendDebugV("GPS", waypoint[WAYPOINT_HOME].x, waypoint[WAYPOINT_HOME].y, GPS.fix);
			//mavlinkSendDebugV("GPS2", GPS.latitude, GPS.longitude, GPS.hasHome);
			
			pitch.demandtemp = pitch.demand;
			roll.demandtemp = roll.demand;
			
			if (MODE_SIMPLICITY == 1) {
				pitch.demand = fsin(psiAngle+M_PI_2)*pitch.demandtemp - fsin(psiAngle)*roll.demandtemp;
				roll.demand = fsin(psiAngle)*pitch.demandtemp + fsin(psiAngle+M_PI_2)*roll.demandtemp;
			}
			
			if (MODE_SIMPLICITY == 2) {
				pitch.demand = fsin(0.78539+M_PI_2)*pitch.demandtemp - fsin(0.78539)*roll.demandtemp;
				roll.demand = fsin(0.78539)*pitch.demandtemp + fsin(0.78539+M_PI_2)*roll.demandtemp;
			}
			//mavlinkSendDebugV("AngleDem", pitch.demand, roll.demand, 0);
			
			pitch.derivative = (pitch.demand - pitch.demandOld);    // times 50 for 50 Hz
			roll.derivative = (roll.demand - roll.demandOld);
			yaw.derivative = (yaw.demand - yaw.demandOld);
			
			pitch.demandOld = pitch.demand;
			roll.demandOld = roll.demand;
			yaw.demandOld = yaw.demand;
			
			
			//mavlinkSendDebugV("SIMP", psiAngle, pitch.demand, roll.demand);
			
			

			// Errors for Motor PID loops
			pitcherror = pitch.demand - thetaAngle;
			rollerror = roll.demand - phiAngle;
			yawerror = yaw.demand + psiAngle;

			if(pitcherror > M_PI) pitcherror -= M_TWOPI;
			else if(pitcherror < -M_PI) pitcherror += M_TWOPI;
			
			if(rollerror > M_PI) rollerror -= M_TWOPI;
			else if(rollerror < -M_PI) rollerror += M_TWOPI;
			
			if(yawerror > M_PI) yawerror -= M_TWOPI;
			else if(yawerror < -M_PI) yawerror += M_TWOPI;

			pitch.integral *= PITCH_De;
			roll.integral *= ROLL_De;
			yaw.integral *= YAW_De;
			pitch.integral += pitcherror;
			roll.integral += rollerror;
			yaw.integral += yawerror;
			
			// Attitude control PID loops
			pitchcorrection = ((float)Gyro.Y.av - PITCH_Boost*pitch.derivative) * -PITCH_Kd;
			pitchcorrection += -PITCH_Kdd*((float)Gyro.Y.av - pitch.gyroOld);
			pitchcorrection += PITCH_Kp*pitcherror;
			pitchcorrection += PITCH_Ki*pitch.integral;

			rollcorrection = ((float)Gyro.X.av - ROLL_Boost*roll.derivative) * -ROLL_Kd;
			rollcorrection += -ROLL_Kdd*((float)Gyro.X.av - roll.gyroOld);
			rollcorrection += ROLL_Kp*rollerror;
			rollcorrection += ROLL_Ki*roll.integral;

			yawcorrection = ((float)Gyro.Z.av + YAW_Boost*yaw.derivative) * YAW_Kd;
			yawcorrection += YAW_Kdd*((float)Gyro.Z.av - yaw.gyroOld);
			yawcorrection += YAW_Kp*yawerror;
			yawcorrection += YAW_Ki*yaw.integral;
			
			pitch.gyroOld = (float)Gyro.Y.av;
			roll.gyroOld = (float)Gyro.X.av;
			yaw.gyroOld = (float)Gyro.Z.av;
			
			
			
			
			
			// Motor Mixing
			// if (MODE_X1ORPLUS0 == 1) {
			// motorN = pitchcorrection - rollcorrection - yawcorrection;
			// motorE = pitchcorrection + rollcorrection  + yawcorrection;
			// motorS = -pitchcorrection + rollcorrection  - yawcorrection;
			// motorW = -pitchcorrection - rollcorrection  + yawcorrection;
			// }
			// else {
			motorN = pitchcorrection - yawcorrection;
			motorE = -rollcorrection + yawcorrection;
			motorS = -pitchcorrection - yawcorrection;
			motorW = rollcorrection + yawcorrection;
			// }
			
			
			
			
			#if ENABLE_CONSTRAIN
			if(motorN > CONSTRAIN) motorN = CONSTRAIN;
			if(motorN < -CONSTRAIN) motorN = -CONSTRAIN;
			if(motorE > CONSTRAIN) motorE = CONSTRAIN;
			if(motorE < -CONSTRAIN) motorE = -CONSTRAIN;
			if(motorS > CONSTRAIN) motorS = CONSTRAIN;
			if(motorS < -CONSTRAIN) motorS = -CONSTRAIN;
			if(motorW > CONSTRAIN) motorW = CONSTRAIN;
			if(motorW < -CONSTRAIN) motorW = -CONSTRAIN;
			#endif
			
			if (throttle < 50){
				pitch.integral=0;
				roll.integral=0;
				yaw.integral=0;
				Gyro.X.verterrorInt = 0;
				Gyro.Y.verterrorInt = 0;
				Gyro.Z.verterrorInt = 0;
				
				if(throttle < 0) {
					throttle = 0;
				}
				yaw.demand = -psiAngle;
				yaw.demandOld = -psiAngle;
				motor[0].duty = throttle + MINSTICK;
				motor[1].duty = throttle + MINSTICK;
				motor[2].duty = throttle + MINSTICK;
				motor[3].duty = throttle + MINSTICK;
			}
			else {
				if(throttle > MAXSTICK - MINSTICK) throttle = MAXSTICK - MINSTICK;
				
				temp = (signed short)motorN + (signed short)throttle + MINSTICK;
				if(temp > MAXSTICK) temp = MAXSTICK;
				else if(temp < IDLETHROTTLE) temp = IDLETHROTTLE;
				motor[0].duty = temp;
				
				temp = (signed short)motorE + (signed short)throttle + MINSTICK;
				if(temp > MAXSTICK) temp = MAXSTICK;
				else if(temp < IDLETHROTTLE) temp = IDLETHROTTLE;
				motor[1].duty = temp;
				
				temp = (signed short)motorS + (signed short)throttle + MINSTICK;
				if(temp > MAXSTICK) temp = MAXSTICK;
				else if(temp < IDLETHROTTLE) temp = IDLETHROTTLE;
				motor[2].duty = temp;
				
				temp = (signed short)motorW + (signed short)throttle + MINSTICK;
				if(temp > MAXSTICK) temp = MAXSTICK;
				else if(temp < IDLETHROTTLE) temp = IDLETHROTTLE;
				motor[3].duty = temp;
			}
			
			LEDToggle(LED0);
		}

	}
	
	if(loopCounter20Hz > 50) { // *** 20Hz loop
		loopCounter20Hz = 0;

		// GPS data is attempted collection at 20Hz, actual data sent at 5Hz
		unsigned short length, i;
		unsigned char I2CBuffer[GPS_PAYLOAD_SIZE];
		unsigned short idComp;
		unsigned char * ptr; // mess with pointers to shoehorn chars into int array
		
		I2CBuffer[0] = SPH_GPS_ADDR;
		I2CBuffer[1] = 0xfd;	    // Contains the number of valid bytes
		I2CBuffer[2] = SPH_GPS_ADDR | 1;
		
		AHRSStop();// Can has watchdog plx?
		I2CMaster(I2CBuffer, 2, I2CBuffer, 2);
		AHRSStart();
		length = I2CBuffer[1] + (I2CBuffer[0] << 8);
		
		if(length > 0) {
			if(length > GPS_PAYLOAD_SIZE) { // prevent overflow
				length = GPS_PAYLOAD_SIZE;
			}
			I2CBuffer[0] = SPH_GPS_ADDR;
			I2CBuffer[1] = 0xff;	    // Contains the message stream
			I2CBuffer[2] = SPH_GPS_ADDR | 1;
			
			AHRSStop();
			I2CMaster(I2CBuffer, 2, I2CBuffer, length);  // save direct to USB data buffer
			AHRSStart();
			
			for(i=0; i<length; i++) {
				switch(GPS.state) {
					case 0:  // search for 0xB5 first start header, also allow tarting on the second start header due to a bug
					if(I2CBuffer[i] == 0xB5) GPS.state = 1;
					else if(I2CBuffer[i] == 0x62) GPS.state = 2;
					break;
					case 1:  // search for 0x62 second start header
					if(I2CBuffer[i] == 0x62) GPS.state = 2;
					else GPS.state = 0;
					break;
					case 2:  // read the first ID
					GPS.id1 = I2CBuffer[i];
					GPS.state = 3;
					GPS.checksumA = I2CBuffer[i];
					GPS.checksumB = GPS.checksumA;
					break;
					case 3:  // read the second ID
					GPS.id2 = I2CBuffer[i];
					GPS.state = 4;
					GPS.checksumA += I2CBuffer[i];
					GPS.checksumB += GPS.checksumA;
					break;
					case 4:  // read the first byte of length
					GPS.payloadLength = I2CBuffer[i];
					GPS.checksumA += I2CBuffer[i];
					GPS.checksumB += GPS.checksumA;
					GPS.state = 5;
					break;
					case 5:  // read the second byte of length
					GPS.payloadLength += I2CBuffer[i] << 8;
					GPS.checksumA += I2CBuffer[i];
					GPS.checksumB += GPS.checksumA;
					GPS.payloadCount = 0;
					if(GPS.payloadLength <= GPS_PAYLOAD_SIZE) { // prevent array trying to get out of bounds
						GPS.state = 6;
					}
					else {
						GPS.state = 0;
					}
					break;
					case 6:  // read a byte of payload
					if(GPS.payloadLength > 0) {
						GPS.payload[GPS.payloadCount++] = I2CBuffer[i];
						GPS.payloadLength --;
						GPS.checksumA += I2CBuffer[i];
						GPS.checksumB += GPS.checksumA;
						if(GPS.payloadLength == 0) {
							GPS.state = 7;
						}
					}
					break;
					case 7:  // check first GPS.checksum
					if(I2CBuffer[i] == GPS.checksumA) GPS.state = 8;
					else GPS.state = 0;
					break;
					case 8:  // check second GPS.checksum
					if(I2CBuffer[i] == GPS.checksumB) {
						idComp = (GPS.id1 << 8) | GPS.id2;
						switch(idComp) {
							case 0x0102:
							ptr = (unsigned char *)&GPS.longitude;
							ptr[0] = GPS.payload[4];
							ptr[1] = GPS.payload[5];
							ptr[2] = GPS.payload[6];
							ptr[3] = GPS.payload[7];
							ptr = (unsigned char *)&GPS.latitude;
							ptr[0] = GPS.payload[8];
							ptr[1] = GPS.payload[9];
							ptr[2] = GPS.payload[10];
							ptr[3] = GPS.payload[11];
							ptr = (unsigned char *)&GPS.altitude;
							ptr[0] = GPS.payload[16];
							ptr[1] = GPS.payload[17];
							ptr[2] = GPS.payload[18];
							ptr[3] = GPS.payload[19];
							ptr = (unsigned char *)&GPS.hAcc;
							ptr[0] = GPS.payload[20];
							ptr[1] = GPS.payload[21];
							ptr[2] = GPS.payload[22];
							ptr[3] = GPS.payload[23];
							ptr = (unsigned char *)&GPS.vAcc;
							ptr[0] = GPS.payload[24];
							ptr[1] = GPS.payload[25];
							ptr[2] = GPS.payload[26];
							ptr[3] = GPS.payload[27];
							if(GPS.firstFix == 0) GPS.firstFix++;
							break;
							case 0x0103:
							if(GPS.payload[4] == 0x03) {
								if(GPS.firstFix > 50) {
									LEDOn(LED2);
									GPS.fix = 3; // allow for 3D fix only
								}
								else if(GPS.firstFix == 50) {
									GPS.firstFix++;
									setHomeInt(GPS.latitude, GPS.longitude, GPS.altitude);
									dataOnDemand = ONDEMAND_GPS_HOME;
									Baro.altitudeOffset = GPS.altitude - Baro.altitude;
								}
								else if(GPS.firstFix > 0) {
									GPS.firstFix++;
								}
							}
							else {
								GPS.fix = 0;
								LEDOff(LED2);
							}
							break;
							case 0x0112:
							ptr = (unsigned char *)&GPS.speed3D;
							ptr[0] = GPS.payload[16];
							ptr[1] = GPS.payload[17];
							ptr[2] = GPS.payload[18];
							ptr[3] = GPS.payload[19];
							ptr = (unsigned char *)&GPS.speedGround;
							ptr[0] = GPS.payload[20];
							ptr[1] = GPS.payload[21];
							ptr[2] = GPS.payload[22];
							ptr[3] = GPS.payload[23];
							ptr = (unsigned char *)&GPS.heading;
							ptr[0] = GPS.payload[24];
							ptr[1] = GPS.payload[25];
							ptr[2] = GPS.payload[26];
							ptr[3] = GPS.payload[27];
							break;
						}
					}
					else GPS.state = 0;
					break;
				}
			}
		}
		
		// MAVLink parameter protocol
		if(storage.sendIndex < STORAGE_DATA) {
			strcpy((char *)mavlink_param_value.param_id, storage.name[storage.sendIndex]);
			mavlink_param_value.param_value = storage.param[storage.sendIndex];
			mavlink_param_value.param_index = storage.sendIndex;
			mavlink_param_value.param_count = STORAGE_DATA;
			//mavlink_param_value.param_type = MAV_VAR_FLOAT;
			mavlink_param_value.param_type = 9; // MAV_VAR_FLOAT defined wrongly in this version of MAVLink or QGroundControl
			mavlink_msg_param_value_encode(mavlink_system.sysid, mavlink_system.compid, &mavlink_tx_msg, &mavlink_param_value);
			mavlink_message_len = mavlink_msg_to_send_buffer(mavlink_message_buf, &mavlink_tx_msg);
			UARTWrite(mavlink_message_buf, mavlink_message_len);
			storage.sendIndex++;
		}
		
		
		// MAVLink waypoint protocol
		if(waypointReceiveIndex < waypointCount) {
			if(waypointTimer > WAYPOINT_TIMEOUT) {
				mavlink_mission_request.seq = waypointReceiveIndex;
				mavlink_mission_request.target_system = waypointProviderID;
				mavlink_mission_request.target_component = waypointProviderComp;
				mavlink_msg_mission_request_encode(mavlink_system.sysid, MAV_COMP_ID_MISSIONPLANNER, &mavlink_tx_msg, &mavlink_mission_request);
				mavlink_message_len = mavlink_msg_to_send_buffer(mavlink_message_buf, &mavlink_tx_msg);
				UARTWrite(mavlink_message_buf, mavlink_message_len);
				
				waypointTimer = 0;
				waypointTries++;
			}
			if(waypointTries > waypointTries) { // timeout failure
				waypointCount = 0;
				mavlinkSendText(255, "Receiving Waypoint timeout");
			}
		}
	}

	if(loopCounter1Hz > 1000) { // *** 1Hz loop
		loopCounter1Hz = 0;
		heartbeat();
	}
	
	// MAVLink report raw sensor readings
	if(dataRate[MAV_DATA_STREAM_RAW_SENSORS] && dataCount[MAV_DATA_STREAM_RAW_SENSORS] > 1000/dataRate[MAV_DATA_STREAM_RAW_SENSORS]) {
		dataCount[MAV_DATA_STREAM_RAW_SENSORS] = 0;
		
		mavlink_raw_imu.time_usec = sysTime * 1000;
		mavlink_raw_imu.xacc = Accel.X.raw;
		mavlink_raw_imu.yacc = Accel.Y.raw;
		mavlink_raw_imu.zacc = Accel.Z.raw;
		mavlink_raw_imu.xgyro = Gyro.X.raw;
		mavlink_raw_imu.ygyro = Gyro.Y.raw;
		mavlink_raw_imu.zgyro = Gyro.Z.raw;
		mavlink_raw_imu.xmag = Magneto.X.raw;
		mavlink_raw_imu.ymag = Magneto.Y.raw;
		mavlink_raw_imu.zmag = Magneto.Z.raw;
		mavlink_msg_raw_imu_encode(mavlink_system.sysid, MAV_COMP_ID_IMU, &mavlink_tx_msg, &mavlink_raw_imu);
		mavlink_message_len = mavlink_msg_to_send_buffer(mavlink_message_buf, &mavlink_tx_msg);
		UARTWrite(mavlink_message_buf, mavlink_message_len);
		
		mavlink_raw_pressure.time_usec = sysTime * 1000;
		mavlink_raw_pressure.press_abs = Baro.pressure;
		mavlink_raw_pressure.press_diff1 = 0;
		mavlink_raw_pressure.press_diff2 = 0;
		mavlink_raw_pressure.temperature = Baro.temperature;
		
		mavlink_msg_raw_pressure_encode(mavlink_system.sysid, MAV_COMP_ID_IMU, &mavlink_tx_msg, &mavlink_raw_pressure);
		mavlink_message_len = mavlink_msg_to_send_buffer(mavlink_message_buf, &mavlink_tx_msg);
		UARTWrite(mavlink_message_buf, mavlink_message_len);
		
		// GPS Raw and status (not used, already reported by position message)
	}
	
	// Mavlink report extended/aux status
	if(dataRate[MAV_DATA_STREAM_EXTENDED_STATUS] && dataCount[MAV_DATA_STREAM_EXTENDED_STATUS] > 1000/dataRate[MAV_DATA_STREAM_EXTENDED_STATUS]) {
		dataCount[MAV_DATA_STREAM_EXTENDED_STATUS] = 0;
		
		// (none at the moment)
	}
	
	// RC Channels
	if(dataRate[MAV_DATA_STREAM_RC_CHANNELS] && dataCount[MAV_DATA_STREAM_RC_CHANNELS] > 1000/dataRate[MAV_DATA_STREAM_RC_CHANNELS]) {
		dataCount[MAV_DATA_STREAM_RC_CHANNELS] = 0;
		
		mavlink_rc_channels_raw.time_boot_ms = sysTime;
		mavlink_rc_channels_raw.chan1_raw = input[0].av;
		mavlink_rc_channels_raw.chan2_raw = input[1].av;
		mavlink_rc_channels_raw.chan3_raw = input[2].av;
		mavlink_rc_channels_raw.chan4_raw = input[3].av;
		mavlink_rc_channels_raw.chan5_raw = input[4].av;
		mavlink_rc_channels_raw.chan6_raw = 0;
		mavlink_rc_channels_raw.chan7_raw = 0;
		mavlink_rc_channels_raw.chan8_raw = 0;
		mavlink_rc_channels_raw.port = 0;
		mavlink_rc_channels_raw.rssi = 0;
		mavlink_msg_rc_channels_raw_encode(mavlink_system.sysid, mavlink_system.compid, &mavlink_tx_msg, &mavlink_rc_channels_raw);
		mavlink_message_len = mavlink_msg_to_send_buffer(mavlink_message_buf, &mavlink_tx_msg);
		UARTWrite(mavlink_message_buf, mavlink_message_len);
		
		// Mavlink Servo status
		mavlink_servo_output_raw.time_usec = sysTime * 1000;
		mavlink_servo_output_raw.servo1_raw = motor[0].duty;
		mavlink_servo_output_raw.servo2_raw = motor[1].duty;
		mavlink_servo_output_raw.servo3_raw = motor[2].duty;
		mavlink_servo_output_raw.servo4_raw = motor[3].duty;
		mavlink_servo_output_raw.servo5_raw = 0;
		mavlink_servo_output_raw.servo6_raw = 0;
		mavlink_servo_output_raw.servo7_raw = 0;
		mavlink_servo_output_raw.servo8_raw = 0;
		mavlink_servo_output_raw.port = 0;
		mavlink_msg_servo_output_raw_encode(mavlink_system.sysid, MAV_COMP_ID_SERVO1, &mavlink_tx_msg, &mavlink_servo_output_raw);
		mavlink_message_len = mavlink_msg_to_send_buffer(mavlink_message_buf, &mavlink_tx_msg);
		UARTWrite(mavlink_message_buf, mavlink_message_len);
	}
	
	// Raw control (using this for raw outputs)
	if(dataRate[MAV_DATA_STREAM_RAW_CONTROLLER] && dataCount[MAV_DATA_STREAM_RAW_CONTROLLER] > 1000/dataRate[MAV_DATA_STREAM_RAW_CONTROLLER]) {
		dataCount[MAV_DATA_STREAM_RAW_CONTROLLER] = 0;
		
		mavlink_attitude.time_boot_ms = sysTime;
		mavlink_attitude.roll = phiAngle;
		mavlink_attitude.pitch = thetaAngle;
		mavlink_attitude.yaw = psiAngle;
		mavlink_attitude.rollspeed = Gyro.X.si;
		mavlink_attitude.pitchspeed = Gyro.Y.si;
		mavlink_attitude.yawspeed = Gyro.Z.si;
		mavlink_msg_attitude_encode(mavlink_system.sysid, MAV_COMP_ID_IMU, &mavlink_tx_msg, &mavlink_attitude);
		mavlink_message_len = mavlink_msg_to_send_buffer(mavlink_message_buf, &mavlink_tx_msg);
		UARTWrite(mavlink_message_buf, mavlink_message_len);

		// Mavlink demands
		mavlink_roll_pitch_yaw_thrust_setpoint.time_boot_ms = sysTime;
		mavlink_roll_pitch_yaw_thrust_setpoint.roll = roll.demand;
		mavlink_roll_pitch_yaw_thrust_setpoint.pitch = pitch.demand;
		mavlink_roll_pitch_yaw_thrust_setpoint.yaw = yaw.demand;
		mavlink_roll_pitch_yaw_thrust_setpoint.thrust = (float)throttle/(float)(MAXSTICK-MINSTICK);;
		mavlink_msg_roll_pitch_yaw_thrust_setpoint_encode(mavlink_system.sysid, mavlink_system.compid, &mavlink_tx_msg, &mavlink_roll_pitch_yaw_thrust_setpoint);
		mavlink_message_len = mavlink_msg_to_send_buffer(mavlink_message_buf, &mavlink_tx_msg);
		UARTWrite(mavlink_message_buf, mavlink_message_len);
		
		// Nav controller
	}
	
	// Position
	if(dataRate[MAV_DATA_STREAM_POSITION] && dataCount[MAV_DATA_STREAM_POSITION] > 1000/dataRate[MAV_DATA_STREAM_POSITION]) {
		dataCount[MAV_DATA_STREAM_POSITION] = 0;
		
		// GPS
		mavlink_gps_raw_int.time_usec = sysTime * 1000;
		mavlink_gps_raw_int.lat = GPS.latitude;
		mavlink_gps_raw_int.lon = GPS.longitude;
		mavlink_gps_raw_int.alt = GPS.altitude;
		mavlink_gps_raw_int.eph = GPS.hAcc/10;
		mavlink_gps_raw_int.epv = GPS.vAcc/10;
		mavlink_gps_raw_int.vel = GPS.speedGround;
		mavlink_gps_raw_int.cog = GPS.heading; // This is 2D heading from GPS
		mavlink_gps_raw_int.fix_type = GPS.fix;
		mavlink_gps_raw_int.satellites_visible = 255;
		
		mavlink_msg_gps_raw_int_encode(mavlink_system.sysid, MAV_COMP_ID_GPS, &mavlink_tx_msg, &mavlink_gps_raw_int);
		mavlink_message_len = mavlink_msg_to_send_buffer(mavlink_message_buf, &mavlink_tx_msg);
		UARTWrite(mavlink_message_buf, mavlink_message_len);
		
		// Mavlink VFR HUD
		mavlink_vfr_hud.airspeed = GPS.speed3D; // airspeed in m/s
		mavlink_vfr_hud.heading = (psiAngle * 180) / M_PI;
		mavlink_vfr_hud.alt = altitude/(float)1000; // altitude (MSL), in meters
		mavlink_vfr_hud.climb = (altitude - lastAltitude)/(float)1000; // climb rate in m/s

		if(throttle > 0) mavlink_vfr_hud.throttle = (float)throttle*(float)100/(float)(MAXSTICK-MINSTICK);
		else mavlink_vfr_hud.throttle = 0;
		
		mavlink_msg_vfr_hud_encode(mavlink_system.sysid, MAV_COMP_ID_IMU, &mavlink_tx_msg, &mavlink_vfr_hud);
		mavlink_message_len = mavlink_msg_to_send_buffer(mavlink_message_buf, &mavlink_tx_msg);
		UARTWrite(mavlink_message_buf, mavlink_message_len);
	}
	
	// Extra messages
	if(dataRate[MAV_DATA_STREAM_EXTRA1] && dataCount[MAV_DATA_STREAM_EXTRA1] > 1000/dataRate[MAV_DATA_STREAM_EXTRA1]) {
		dataCount[MAV_DATA_STREAM_EXTRA1] = 0;
		
		// (none at the moment)
	}
	
	// More extra messages
	if(dataRate[MAV_DATA_STREAM_EXTRA2] && dataCount[MAV_DATA_STREAM_EXTRA2] > 1000/dataRate[MAV_DATA_STREAM_EXTRA2]) {
		dataCount[MAV_DATA_STREAM_EXTRA2] = 0;
		
		// (none at the moment)
	}
	
	// MOAR extra messages
	if(dataRate[MAV_DATA_STREAM_EXTRA3] && dataCount[MAV_DATA_STREAM_EXTRA3] > 1000/dataRate[MAV_DATA_STREAM_EXTRA3]) {
		dataCount[MAV_DATA_STREAM_EXTRA3] = 0;

		// (none at the moment)
	}
	
	// MAVLink data on demand
	if(dataOnDemand) {
		switch(dataOnDemand) {
			case ONDEMAND_GPS_HOME:
			// note: QGroundControl's implementation of this message is broken
			mvalink_gps_global_origin.latitude = waypoint[WAYPOINT_HOME].x * 10000000;
			mvalink_gps_global_origin.longitude =  waypoint[WAYPOINT_HOME].y * 10000000;
			mvalink_gps_global_origin.altitude =  waypoint[WAYPOINT_HOME].z * 1000;
			
			mavlink_msg_gps_global_origin_encode(mavlink_system.sysid, MAV_COMP_ID_MISSIONPLANNER, &mavlink_tx_msg, &mvalink_gps_global_origin);
			mavlink_message_len = mavlink_msg_to_send_buffer(mavlink_message_buf, &mavlink_tx_msg);
			UARTWrite(mavlink_message_buf, mavlink_message_len);
			break;
		}
		dataOnDemand = 0;
	}
}

// ****************************************************************************
// *** ARHS timer, >50Hz
// ****************************************************************************

void Timer1Interrupt0(void) {
	signed short sensor[4];
	float sumsqu, temp, temp2;
	
	// *** Read Gyro
	GyroGetData(sensor);
	
	Gyro.X.raw = sensor[0];
	Gyro.Y.raw = sensor[1];
	Gyro.Z.raw = sensor[2];
	
	// Gyro averaging
	Gyro.X.total -= Gyro.X.history[Gyro.count];
	Gyro.X.history[Gyro.count] = Gyro.X.raw;
	Gyro.X.total += Gyro.X.raw;
	
	Gyro.Y.total -= Gyro.Y.history[Gyro.count];
	Gyro.Y.history[Gyro.count] = Gyro.Y.raw;
	Gyro.Y.total += Gyro.Y.raw;
	
	Gyro.Z.total -= Gyro.Z.history[Gyro.count];
	Gyro.Z.history[Gyro.count] = Gyro.Z.raw;
	Gyro.Z.total += Gyro.Z.raw;
	
	if(++Gyro.count >= GYRO_AVERAGING) Gyro.count = 0;
	
	Gyro.X.av = ((float)Gyro.X.total / (float)GYRO_AVERAGING) - Gyro.X.offset;
	Gyro.Y.av = ((float)Gyro.Y.total / (float)GYRO_AVERAGING) - Gyro.Y.offset;
	Gyro.Z.av = ((float)Gyro.Z.total / (float)GYRO_AVERAGING) - Gyro.Z.offset;

	Gyro.X.si = Gyro.X.av/823.62683050055836260397f; // Into radians per second
	Gyro.Y.si = Gyro.Y.av/823.62683050055836260397f;
	Gyro.Z.si = Gyro.Z.av/823.62683050055836260397f;
	
	// *** QUATERNION!

	if (MODE_X1ORPLUS0 == 1) {
		temp = Gyro.X.si;
		temp2 = Gyro.Y.si;
		Gyro.X.si = M_SQRT1_2 * temp - M_SQRT1_2 * temp2;
		Gyro.Y.si = M_SQRT1_2 * temp2 + M_SQRT1_2 * temp;
	}
	
	Gyro.X.value = (Gyro.X.si - (float)Gyro.X.error)/(float)AHRS_RATE;
	Gyro.Y.value = (Gyro.Y.si - (float)Gyro.Y.error)/(float)AHRS_RATE;
	Gyro.Z.value = (Gyro.Z.si - (float)Gyro.Z.error)/(float)AHRS_RATE;
	
	quaternionOld.u = quaternion.u;
	quaternionOld.a = quaternion.a;
	quaternionOld.i = quaternion.i;
	quaternionOld.r = quaternion.r;

	quaternion.u -= 0.5*(Gyro.X.value*quaternionOld.a + Gyro.Y.value*quaternionOld.i + Gyro.Z.value*quaternionOld.r);
	quaternion.a += 0.5*(Gyro.X.value*quaternionOld.u + Gyro.Z.value*quaternionOld.i - Gyro.Y.value*quaternionOld.r);
	quaternion.i += 0.5*(Gyro.Y.value*quaternionOld.u - Gyro.Z.value*quaternionOld.a + Gyro.X.value*quaternionOld.r);
	quaternion.r += 0.5*(Gyro.Z.value*quaternionOld.u + Gyro.Y.value*quaternionOld.a - Gyro.X.value*quaternionOld.i);
	
	// precalculated values for optimisation
	float quu = quaternion.u * quaternion.u;
	float qua = quaternion.u * quaternion.a;
	float qui = quaternion.u * quaternion.i;
	// float qur = quaternion.u * quaternion.r; // No gain from precalc
	float qaa = quaternion.a * quaternion.a;
	// float qai = quaternion.a * quaternion.i; // No gain from precalc
	float qar = quaternion.a * quaternion.r;
	float qii = quaternion.i * quaternion.i;
	float qir = quaternion.i * quaternion.r;
	float qrr = quaternion.r * quaternion.r;
	float qaaPqii = qaa + qii;
	float TquiMqar = 2 * (qui - qar);
	float TqirPqua = 2 * (qir + qua);
	float m9 = (quu - qaa - qii + qrr);
	
	// renormalise using Taylor expansion
	// sumsqu = (3-(quu + qaaPqii + qrr))/2;
	// quaternion.u *= sumsqu;
	// quaternion.a *= sumsqu;
	// quaternion.i *= sumsqu;
	// quaternion.r *= sumsqu;
	
	// renormalise using fast inverse square root
	sumsqu = invSqrt(quu + qaaPqii + qrr);
	quaternion.u *= sumsqu;
	quaternion.a *= sumsqu;
	quaternion.i *= sumsqu;
	quaternion.r *= sumsqu;

	// avoid gimbal lock at singularity points
	if (TquiMqar == 1) {
		psiAngle = 2 * fatan2(quaternion.a, quaternion.u);
		thetaAngle = M_PI_2;
		phiAngle = 0;
	}
	else if (TquiMqar == -1) {
		psiAngle = -2 * fatan2(quaternion.a, quaternion.u);
		thetaAngle = - M_PI_2;
		phiAngle = 0;
	}
	else {
		thetaAngle = fasin(TquiMqar);
		phiAngle = fatan2(TqirPqua, (1 - 2*qaaPqii));
		psiAngle = fatan2((2*(quaternion.u * quaternion.r + quaternion.a * quaternion.i)), (1 - 2*(qii + qrr)));
	}
	
	if(++ahrsSoftscale > AHRS_CORRECTION) {
		ahrsSoftscale = 0;
		// *** Read Accelerometers
		AccelGetData(sensor);
		Accel.X.raw = sensor[0];
		Accel.Y.raw = sensor[1];
		Accel.Z.raw = sensor[2];
		
		sumsqu = invSqrt((float)Accel.X.raw*(float)Accel.X.raw + (float)Accel.Y.raw*(float)Accel.Y.raw + (float)Accel.Z.raw*(float)Accel.Z.raw);	// Accelerometr data is normalised so no need to convert units.
		Accel.X.value = (float)Accel.X.raw * sumsqu;
		Accel.Y.value = (float)Accel.Y.raw * sumsqu;
		Accel.Z.value = (float)Accel.Z.raw * sumsqu;
		
		if (MODE_X1ORPLUS0 == 1) {
			temp = Accel.X.value;
			temp2 = Accel.Y.value;
			Accel.X.value = M_SQRT1_2 * temp - M_SQRT1_2 * temp2;
			Accel.Y.value = M_SQRT1_2 * temp2 + M_SQRT1_2 * temp;
		}
		
		// *** Read Magnetometers
		MagnetoGetData(sensor);
		Magneto.X.raw = sensor[0];
		Magneto.Y.raw = sensor[1];
		Magneto.Z.raw = sensor[2];
		
		if (MODE_X1ORPLUS0 == 1) {
			temp = Magneto.X.raw;
			temp2 = Magneto.Y.raw;
			Magneto.X.value = M_SQRT1_2 * temp - M_SQRT1_2 * temp2;
			Magneto.Y.value = M_SQRT1_2 * temp2 + M_SQRT1_2 * temp;
		}
		else {
			Magneto.X.value = (float)Magneto.X.raw;
			Magneto.Y.value = (float)Magneto.Y.raw;
		}
		
		Magneto.Z.value = (float)Magneto.Z.raw;
		
		// *** Drift correction
		Gyro.X.verterror = TqirPqua*Accel.Z.value - m9*Accel.Y.value;  // Correction vector
		Gyro.Y.verterror = m9*Accel.X.value + TquiMqar*Accel.Z.value;
		
		float MdotA = Magneto.X.value*Accel.X.value + Magneto.Y.value*Accel.Y.value + Magneto.Z.value*Accel.Z.value;
		float X = (Magneto.X.value - MdotA*Accel.X.value);  // Lagranges Theorum
		float Y = (Magneto.Y.value - MdotA*Accel.Y.value);

		if(Accel.Z.value > 0.3) {
			Gyro.Z.verterror = psiAngle + fatan2(Y, X); // Heading correction error calculation
			//if (Accel.Z.value < 0) Gyro.Z.verterror = psiAngle - fatan2(Y, X) + M_PI; // Todo: fix upside down case

			if (Gyro.Z.verterror > M_PI) Gyro.Z.verterror -= M_TWOPI;
			if (Gyro.Z.verterror < -M_PI) Gyro.Z.verterror += M_TWOPI;
		}
		else {
			Gyro.Z.verterror = 0;
		}

		Gyro.X.verterrorInt *= DRIFT_AccelDe;    // Integral decay
		Gyro.Y.verterrorInt *= DRIFT_AccelDe;
		Gyro.Z.verterrorInt *= DRIFT_MagDe;
		Gyro.X.verterrorInt += Gyro.X.verterror;
		Gyro.Y.verterrorInt += Gyro.Y.verterror;
		Gyro.Z.verterrorInt += Gyro.Z.verterror;
		Gyro.X.error = DRIFT_AccelKp*Gyro.X.verterror + DRIFT_AccelKi*Gyro.X.verterrorInt;  // Error is inserted into a PI loop
		Gyro.Y.error = DRIFT_AccelKp*Gyro.Y.verterror + DRIFT_AccelKi*Gyro.Y.verterrorInt;
		Gyro.Z.error = DRIFT_MagKp*Gyro.Z.verterror + DRIFT_MagKi*Gyro.Z.verterrorInt;
	}
	
	if(++ahrsBarometer > AHRS_RATE/200) {    // scale down so Barometer is working at max of 200Hz
		ahrsBarometer = 0;
		// *** Read Barometer
		if(Baro.reading < BARO_TEMP_EVERY) {
			Baro.raw = BaroReadPress();
			Baro.total -= Baro.history[Baro.count];
			Baro.total += Baro.raw;
			Baro.history[Baro.count] = Baro.raw;
			if(++Baro.count >= BARO_AVERAGING) Baro.count = 0;
		}
		else {
			Baro.temperature = BaroReadTemp();
			Baro.reading = 0;
		}
		
		if(++Baro.reading < BARO_TEMP_EVERY) BaroStartPress();
		else BaroStartTemp();
		
		Baro.pressure = (float)Baro.total/(float)BARO_AVERAGING;
		lastAltitude = Baro.altitude;
		Baro.altitude = BaroApproxAlt(Baro.pressure) + Baro.altitudeOffset;
		altitude = Baro.altitude;
		
		if(GPS.fix) {
			float altitudeError = (float)GPS.altitude - (float)Baro.altitude;
			Baro.altitudeOffset += altitudeError * DRIFT_Baro;
			//altitude = Baro.altitude * ALT_Bias + GPS.altitude * (1-ALT_Bias);
		}
	}
}

// ****************************************************************************
// *** Output PWM timer, nominally 50Hz
// ****************************************************************************

void Timer2Interrupt0(void) {
	Port1Write(PIN8, 1);
	Port2Write(PIN6, 1);
	
	if(input[0].sync > 0) input[0].sync--;
	if(input[1].sync > 0) input[1].sync--;
	
	if(mavlink_heartbeat.base_mode & MAV_MODE_FLAG_SAFETY_ARMED && (mavlink_heartbeat.system_status == MAV_STATE_ACTIVE || mavlink_heartbeat.system_status == MAV_STATE_CRITICAL)) {
		Timer2Match1(motor[0].duty, INTERRUPT);
		Timer2Match2(motor[1].duty, INTERRUPT);
	}
	else {
		Timer2Match1(1000, INTERRUPT);
		Timer2Match2(1000, INTERRUPT);
		motor[0].duty = 1000;
		motor[1].duty = 1000;
	}
}
void Timer3Interrupt0(void) {
	Port1Write(PIN10 | PIN1, 1);
	
	if(input[2].sync > 0) input[2].sync--;
	if(input[3].sync > 0) input[3].sync--;
	
	if(mavlink_heartbeat.base_mode & MAV_MODE_FLAG_SAFETY_ARMED && (mavlink_heartbeat.system_status == MAV_STATE_ACTIVE || mavlink_heartbeat.system_status == MAV_STATE_CRITICAL)) {
		Timer3Match1(motor[2].duty, INTERRUPT);
		Timer3Match2(motor[3].duty, INTERRUPT);
	}
	else {
		Timer3Match1(1000, INTERRUPT);
		Timer3Match2(1000, INTERRUPT);
		motor[2].duty = 1000;
		motor[3].duty = 1000;
	}
}
void Timer2Interrupt1(void) {
	Port1Write(PIN8, 0);
}
void Timer2Interrupt2(void) {
	Port2Write(PIN6, 0);
}
void Timer3Interrupt1(void) {
	Port1Write(PIN10, 0);
}
void Timer3Interrupt2(void) {
	Port1Write(PIN1, 0);
}


// ****************************************************************************
// *** Control timers, nominally 1kHz
// ****************************************************************************

void TickInterrupt(void) {
	loopCounter1Hz++;
	loopCounter20Hz++;
	loopCounter50Hz++;
	sysTime++;
	waypointTimer++;
	dataCount[MAV_DATA_STREAM_RAW_SENSORS]++;
	dataCount[MAV_DATA_STREAM_EXTENDED_STATUS]++;
	dataCount[MAV_DATA_STREAM_RC_CHANNELS]++;
	dataCount[MAV_DATA_STREAM_RAW_CONTROLLER]++;
	dataCount[MAV_DATA_STREAM_POSITION]++;
	dataCount[MAV_DATA_STREAM_EXTRA1]++;
	dataCount[MAV_DATA_STREAM_EXTRA2]++;
	dataCount[MAV_DATA_STREAM_EXTRA3]++;
}


							default:
							mavlink_command_ack.result = MAV_CMD_ACK_ERR_NOT_SUPPORTED;
							break;
						}
						
						mavlink_command_ack.command = mavlink_command_long.command;
						mavlink_msg_command_ack_encode(mavlink_system.sysid, mavlink_system.compid, &mavlink_tx_msg, &mavlink_command_ack);
						mavlink_message_len = mavlink_msg_to_send_buffer(mavlink_message_buf, &mavlink_tx_msg);
						UARTWrite(mavlink_message_buf, mavlink_message_len);
						
						mavlinkSendDebugV("CMD", mavlink_command_long.command, mavlink_command_long.param1, mavlink_command_long.param2);
						break;
					}
					break;
					case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
					// request send of all parameters
					storage.sendIndex = 0;
					break;
					case MAVLINK_MSG_ID_PARAM_SET:
					// request set parameter
					mavlink_msg_param_set_decode(&mavlink_rx_msg, &mavlink_param_set);
					if(mavlink_param_set.target_system == mavlink_system.sysid) {
						// search through parameters for the one named (MAVLink, y u no give index?)
						unsigned short i, j;
						for (i=0; i<STORAGE_DATA; i++){
							unsigned char match = TRUE;
							for (j=0; j<MAVLINK_MSG_PARAM_SET_FIELD_PARAM_ID_LEN; j++) {
								if (storage.name[i][j] !=  mavlink_param_set.param_id[j]) {
									match = FALSE;
									break;
								}
								if (storage.name[i][j] == '\0') break;
							}
							
							if(match == TRUE) {
								storage.param[i] = mavlink_param_set.param_value;
								
								strcpy((char *)mavlink_param_value.param_id, storage.name[i]);
								mavlink_param_value.param_value = storage.param[i];
								mavlink_param_value.param_index = i;
								
								mavlink_msg_param_value_encode(mavlink_system.sysid, mavlink_system.compid, &mavlink_tx_msg, &mavlink_param_value);
								mavlink_message_len = mavlink_msg_to_send_buffer(mavlink_message_buf, &mavlink_tx_msg);
								UARTWrite(mavlink_message_buf, mavlink_message_len);
								
								break;
							}
						}
					}
					break;
					case MAVLINK_MSG_ID_MISSION_CLEAR_ALL:
					mavlink_msg_mission_clear_all_decode(&mavlink_rx_msg, &mavlink_mission_clear_all);
					if (mavlink_mission_clear_all.target_system == mavlink_system.sysid) {
						waypointCurrent = 0;
						waypointCount = 0;
						waypointValid = 0;
						
						mavlink_mission_ack.target_system = mavlink_rx_msg.sysid;
						mavlink_mission_ack.target_component = mavlink_rx_msg.compid;
						
						mavlink_mission_ack.type = MAV_MISSION_ACCEPTED;
						mavlink_msg_mission_ack_encode(mavlink_system.sysid, mavlink_system.compid, &mavlink_tx_msg, &mavlink_mission_ack);
						mavlink_message_len = mavlink_msg_to_send_buffer(mavlink_message_buf, &mavlink_tx_msg);
						UARTWrite(mavlink_message_buf, mavlink_message_len);
					}
					break;
					case MAVLINK_MSG_ID_MISSION_SET_CURRENT:
					mavlink_msg_mission_set_current_decode(&mavlink_rx_msg, &mavlink_mission_set_current);
					if (mavlink_mission_set_current.target_system == mavlink_system.sysid) {
						waypointCurrent = mavlink_mission_set_current.seq;
						mavlink_mission_current.seq = waypointCurrent;
						mavlink_msg_mission_current_encode(mavlink_system.sysid, MAV_COMP_ID_MISSIONPLANNER, &mavlink_tx_msg, &mavlink_mission_current);
						mavlink_message_len = mavlink_msg_to_send_buffer(mavlink_message_buf, &mavlink_tx_msg);
						UARTWrite(mavlink_message_buf, mavlink_message_len);
					}
					break;
					case MAVLINK_MSG_ID_MISSION_COUNT:
					mavlink_msg_mission_count_decode(&mavlink_rx_msg, &mavlink_mission_count);
					if (mavlink_mission_count.target_system == mavlink_system.sysid) {
						waypointCount = mavlink_mission_count.count;
						waypointReceiveIndex = 0;
						waypointTimer = WAYPOINT_TIMEOUT; // set waypoint timeout to timed out so that request is immediate
						waypointTries = 0;
						waypointValid = 0;  // invalidate waypoint storage until full set is received
						
						waypointProviderID = mavlink_rx_msg.sysid;
						waypointProviderComp = mavlink_rx_msg.compid;
					}
					break;
					case MAVLINK_MSG_ID_MISSION_REQUEST_LIST:
					mavlink_msg_mission_request_list_decode(&mavlink_rx_msg, &mavlink_mission_request_list);
					if (mavlink_mission_request_list.target_system == mavlink_system.sysid) {
						if(waypointValid == 0) {
							mavlink_mission_count.count = 0;
						}
						else {
							mavlink_mission_count.count = waypointCount;
						}
						
						mavlink_mission_count.target_system = mavlink_rx_msg.sysid;
						mavlink_mission_count.target_component = mavlink_rx_msg.compid;
						mavlink_msg_mission_count_encode(mavlink_system.sysid, MAV_COMP_ID_MISSIONPLANNER, &mavlink_tx_msg, &mavlink_mission_count);
						mavlink_message_len = mavlink_msg_to_send_buffer(mavlink_message_buf, &mavlink_tx_msg);
						UARTWrite(mavlink_message_buf, mavlink_message_len);
					}
					break;
					case MAVLINK_MSG_ID_MISSION_REQUEST:
					mavlink_msg_mission_request_decode(&mavlink_rx_msg, &mavlink_mission_request);
					if (mavlink_mission_request.target_system == mavlink_system.sysid) {
						if(waypointValid != 0) {
							mavlink_mission_item.target_system = mavlink_rx_msg.sysid;
							mavlink_mission_item.target_component = mavlink_rx_msg.compid;
							
							mavlink_mission_item.seq = mavlink_mission_request.seq;
							mavlink_mission_item.frame = MAV_FRAME_GLOBAL;
							mavlink_mission_item.command = waypoint[mavlink_mission_request.seq].command;
							mavlink_mission_item.autocontinue = waypoint[mavlink_mission_request.seq].autocontinue;
							mavlink_mission_item.param1 = waypoint[mavlink_mission_request.seq].param1;
							mavlink_mission_item.param2 = waypoint[mavlink_mission_request.seq].param2;
							mavlink_mission_item.param3 = waypoint[mavlink_mission_request.seq].param3;
							mavlink_mission_item.param4 = waypoint[mavlink_mission_request.seq].param4;
							mavlink_mission_item.x = waypoint[mavlink_mission_request.seq].x;
							mavlink_mission_item.y = waypoint[mavlink_mission_request.seq].y;
							mavlink_mission_item.z = waypoint[mavlink_mission_request.seq].z;
							
							if(waypointCurrent == mavlink_mission_request.seq) {
								mavlink_mission_item.current = 1;
							}
							else {
								mavlink_mission_item.current = 0;
							}
							
							mavlink_msg_mission_item_encode(mavlink_system.sysid, MAV_COMP_ID_MISSIONPLANNER, &mavlink_tx_msg, &mavlink_mission_item);
							mavlink_message_len = mavlink_msg_to_send_buffer(mavlink_message_buf, &mavlink_tx_msg);
							UARTWrite(mavlink_message_buf, mavlink_message_len);
						}
					}
					break;
					case MAVLINK_MSG_ID_MISSION_ITEM:
					mavlink_msg_mission_item_decode(&mavlink_rx_msg, &mavlink_mission_item);
					if (mavlink_mission_item.target_system == mavlink_system.sysid) {
						mavlink_mission_ack.type = MAV_MISSION_ERROR;
						if(mavlink_mission_item.frame == MAV_FRAME_GLOBAL) {
							if(mavlink_mission_item.seq < MAX_WAYPOINTS) {
								//waypoint[mavlink_mission_item.seq].frame = mavlink_mission_item.frame;
								waypoint[mavlink_mission_item.seq].command = mavlink_mission_item.command;
								waypoint[mavlink_mission_item.seq].autocontinue = mavlink_mission_item.autocontinue;
								waypoint[mavlink_mission_item.seq].param1 = mavlink_mission_item.param1;
								waypoint[mavlink_mission_item.seq].param2 = mavlink_mission_item.param2;
								waypoint[mavlink_mission_item.seq].param3 = mavlink_mission_item.param3;
								waypoint[mavlink_mission_item.seq].param4 = mavlink_mission_item.param4;
								waypoint[mavlink_mission_item.seq].x = mavlink_mission_item.x;
								waypoint[mavlink_mission_item.seq].y = mavlink_mission_item.y;
								waypoint[mavlink_mission_item.seq].z = mavlink_mission_item.z;
								if(mavlink_mission_item.current == 1) {
									waypointCurrent = mavlink_mission_item.seq;
								}
								
								waypointReceiveIndex++;
								waypointTimer = WAYPOINT_TIMEOUT; // set waypoint timeout to timed out so that request is immediate
								waypointTries = 0;
								mavlink_mission_ack.target_system = mavlink_rx_msg.sysid;
								mavlink_mission_ack.target_component = mavlink_rx_msg.compid;
								if(waypointReceiveIndex >= waypointCount) {
									mavlink_mission_ack.type = MAV_MISSION_ACCEPTED;
									waypointValid = 1;
								}
								else if(waypointReceiveIndex >= MAX_WAYPOINTS) {
									mavlink_mission_ack.type = MAV_MISSION_NO_SPACE;
								}
							}
							else {
								mavlink_mission_ack.type = MAV_MISSION_NO_SPACE;
							}
						}
						else {
							mavlink_mission_ack.type = MAV_MISSION_UNSUPPORTED_FRAME;
						}
						
						mavlink_msg_mission_ack_encode(mavlink_system.sysid, MAV_COMP_ID_MISSIONPLANNER, &mavlink_tx_msg, &mavlink_mission_ack);
						mavlink_message_len = mavlink_msg_to_send_buffer(mavlink_message_buf, &mavlink_tx_msg);
						UARTWrite(mavlink_message_buf, mavlink_message_len);
					}
					break;
					case MAVLINK_MSG_ID_MISSION_WRITE_PARTIAL_LIST:
					mavlink_mission_ack.type = MAV_MISSION_UNSUPPORTED;
					mavlink_msg_mission_ack_encode(mavlink_system.sysid, MAV_COMP_ID_MISSIONPLANNER, &mavlink_tx_msg, &mavlink_mission_ack);
					mavlink_message_len = mavlink_msg_to_send_buffer(mavlink_message_buf, &mavlink_tx_msg);
					UARTWrite(mavlink_message_buf, mavlink_message_len);
					break;
					
					case MAVLINK_MSG_ID_REQUEST_DATA_STREAM:
					mavlink_msg_request_data_stream_decode(&mavlink_rx_msg, &mavlink_request_data_stream);
					if (mavlink_request_data_stream.target_system == mavlink_system.sysid) {
						if(mavlink_request_data_stream.req_message_rate > 255) mavlink_request_data_stream.req_message_rate = 255;
						dataRate[mavlink_request_data_stream.req_stream_id] = mavlink_request_data_stream.req_message_rate;
						dataCount[mavlink_request_data_stream.req_stream_id] = 0;
					}
					break;
					case MAVLINK_MSG_ID_MISSION_ACK:
					//ignored
					break;
					default:
					mavlinkSendDebugV("MSGID", mavlink_rx_msg.msgid, 0, 0);
					break;
				}
			}
		}
	}

	// ****************************************************************************
	// *** Inputs
	// ****************************************************************************

	void Port2Interrupt(unsigned short GPIOPins) {
		unsigned char i;
		unsigned short time;
		unsigned short duty;
		time = Timer0Read();

		for(i=0; i<5; i++) {
			if(Port2GetInterrupt() & input[i].pin) {		// an interrupt on this pin!
				if(Port2Read(input[i].pin)) {		// pin is high state
					if(input[i].state == 0) { // begin timer
						input[i].timer = time;
					}
				}
				else {						// pin is low state
					if(input[i].state == 1) { // end timer
						if(time > input[i].timer) {	// save the duty timer
							duty = time - input[i].timer;
						}
						else {
							duty = (unsigned int)time + (unsigned int)0x10000 - (unsigned int)input[i].timer;	// timer wrap-around condition
						}

						if(duty < LOWERROR || duty > HIGHERROR) { // check bounds (if out of bounds, ignore reading, if out of bounds thrice or more in a row, reset input)
							input[i].error ++;
							if(input[i].error > 2) {	// if out of bounds too many times in a row, reset input
								
								input[i].duty = 1000;
								input[i].total -= input[i].history[input[i].count];
								input[i].history[input[i].count] = 1000;
								input[i].total += 1000;
							}
						}
						else {	// if no error, update
							input[i].duty = duty;
							input[i].total -= input[i].history[input[i].count];
							input[i].history[input[i].count] = duty;
							input[i].total += duty;
							if(++input[i].count >= INPUT_AVERAGING) input[i].count = 0;
							
							input[i].error = 0;
							input[i].sync = 10;
						}
					}
				}
				input[i].state = Port2Read(input[i].pin);
			}
		}
	}

	// ****************************************************************************
	// *** Functions
	// ****************************************************************************

	signed short nonLinearThrottle(unsigned short input, unsigned short offset) {
		signed int output;
		signed short input2 = ((signed short) input - (signed short) offset);
		
		if (input2 < 10*THROTTLE_X1) {
			output = input2 * THROTTLE_Y1;
			output /= THROTTLE_X1;
		}
		else if(input2 < 10*THROTTLE_X2) {
			output = (input2-(THROTTLE_X1*10));
			output *= (THROTTLE_Y2-THROTTLE_Y1);
			output /= (THROTTLE_X2-THROTTLE_X1);
			output += (10*THROTTLE_Y1);
		}
		else {
			output = (THROTTLE_Yn-THROTTLE_Y2);
			output *= (input2-(10*THROTTLE_X2));
			output /= (100-THROTTLE_X2);
			output += (10*THROTTLE_Y2);
		}
		return (signed short) output;
	}

	void loadStorage(void) {
		unsigned char data[4], checksum;
		unsigned short i;
		float * ptr;
		ptr = (float *) data;
		
		strcpy(storage.name[0], "IDENTITY");
		strcpy(storage.name[1], "PITCH_Kp");
		strcpy(storage.name[2], "PITCH_Ki");
		strcpy(storage.name[3], "PITCH_Kd");
		strcpy(storage.name[4], "PITCH_Kdd");
		strcpy(storage.name[5], "PITCH_Boost");
		strcpy(storage.name[6], "PITCH_De");
		strcpy(storage.name[7], "PITCH_KpT1");
		strcpy(storage.name[8], "PITCH_KpT2");
		strcpy(storage.name[9], "ROLL_Kp");
		strcpy(storage.name[10], "ROLL_Ki");
		strcpy(storage.name[11], "ROLL_Kd");
		strcpy(storage.name[12], "ROLL_Kdd");
		strcpy(storage.name[13], "ROLL_Boost");
		strcpy(storage.name[14], "ROLL_De");
		strcpy(storage.name[15], "ROLL_KpT1");
		strcpy(storage.name[16], "ROLL_KpT2");
		strcpy(storage.name[17], "YAW_Kp");
		strcpy(storage.name[18], "YAW_Ki");
		strcpy(storage.name[19], "YAW_Kd");
		strcpy(storage.name[20], "YAW_Kdd");
		strcpy(storage.name[21], "YAW_Boost");
		strcpy(storage.name[22], "YAW_De");
		strcpy(storage.name[23], "YAW_KpT1");
		strcpy(storage.name[24], "YAW_KpT2");
		strcpy(storage.name[25], "DRIFT_AccelKp");
		strcpy(storage.name[26], "DRIFT_AccelKi");
		strcpy(storage.name[27], "DRIFT_AccelDe");
		strcpy(storage.name[28], "DRIFT_MagKp");
		strcpy(storage.name[29], "DRIFT_MagKi");
		strcpy(storage.name[30], "DRIFT_MagDe");
		strcpy(storage.name[31], "DRIFT_Baro");
		strcpy(storage.name[32], "THROTTLE_X1");
		strcpy(storage.name[33], "THROTTLE_Y1");
		strcpy(storage.name[34], "THROTTLE_X2");
		strcpy(storage.name[35], "THROTTLE_Y2");
		strcpy(storage.name[36], "THROTTLE_Yn");
		strcpy(storage.name[37], "GROUNDTHRESH");
		strcpy(storage.name[38], "DESCENDRATE");
		strcpy(storage.name[39], "ASCENDRATE");
		strcpy(storage.name[40], "ALT_Ki");
		strcpy(storage.name[41], "ALT_Decay");
		strcpy(storage.name[42], "ALT_Bias");
		strcpy(storage.name[43], "MODE_X1ORPLUS0");
		strcpy(storage.name[44], "MODE_SIMPLICITY");
		strcpy(storage.name[45], "HOLD_KP");
		strcpy(storage.name[46], "HOLD_LIM");
		
		AHRSStop();
		checksum = EEPROMReadByte(0); // read MAV ID, note: checksum is being used as a temporary variable to avoid double EEPROM read
		storage.param[0] = (float)checksum;
		for(i=1; i<STORAGE_DATA; i++) {
			//EEPROMRead(4*i-3, data, 4);
			data[0] = EEPROMReadByte(4*i-3); // not using EEPROMRead to save space!
			data[1] = EEPROMReadByte(4*i-2);
			data[2] = EEPROMReadByte(4*i-1);
			data[3] = EEPROMReadByte(4*i-0);
			storage.param[i] = *ptr;
			checksum += data[0];
			checksum += data[1];
			checksum += data[2];
			checksum += data[3];
		}
		
		for(i=0; i<STORAGE_RATES; i++) {
			dataRate[i] = EEPROMReadByte(STORAGE_RATES_START+i);
			checksum += dataRate[i];
		}
		
		checksum += EEPROM_VERSION;
		if(EEPROMReadByte(STORAGE_END) != checksum) {
			mavlinkSendText(0, "EEPROM checksum mismatch, restoring defaults");
			IDENTITY = 1;
			PITCH_Kp = 140;
			PITCH_Ki = 0.005;
			PITCH_Kd = 0.025;
			PITCH_Kdd = 0.16;
			PITCH_Boost = 0.1;
			PITCH_De = 0.98;
			PITCH_KpT1 = 0;
			PITCH_KpT2 = 0;
			ROLL_Kp = 140;
			ROLL_Ki = 0.005;
			ROLL_Kd = 0.025;
			ROLL_Kdd = 0.16;
			ROLL_Boost = 0.1;
			ROLL_De = 0.98;
			ROLL_KpT1 = 0;
			ROLL_KpT2 = 0;
			YAW_Kp = 140;
			YAW_Ki = 0;
			YAW_Kd = 0.1;
			YAW_Kdd = 0;
			YAW_Boost = 0.4;
			YAW_De = 1;
			YAW_KpT1 = 0;
			YAW_KpT2 = 0;
			DRIFT_AccelKp = 0.1;
			DRIFT_AccelKi = 0;
			DRIFT_AccelDe = 0;
			DRIFT_MagKp = 0.1;
			DRIFT_MagKi = 0;
			DRIFT_MagDe = 0;
			DRIFT_Baro = 0.01;
			THROTTLE_X1 = 20;
			THROTTLE_Y1 = 20;
			THROTTLE_X2 = 80;
			THROTTLE_Y2 = 80;
			THROTTLE_Yn = 100;
			GROUNDTHRESH = 5000;
			DESCENDRATE = 1;
			ASCENDRATE = 1;
			ALT_Ki = 0;
			ALT_Decay = 0.95;
			ALT_Bias = 0.5;
			MODE_X1ORPLUS0 = 0;
			MODE_SIMPLICITY = 0;
			HOLD_KP = 0.16;
			HOLD_LIM = 0.16;
			
			
			dataRate[MAV_DATA_STREAM_RAW_SENSORS] = 0;    // Raw sensor
			dataRate[MAV_DATA_STREAM_EXTENDED_STATUS] = 0;   // Ext status
			dataRate[MAV_DATA_STREAM_RC_CHANNELS] = 0;    // RC Channel
			dataRate[MAV_DATA_STREAM_RAW_CONTROLLER] = 20;    // Raw Control
			dataRate[MAV_DATA_STREAM_POSITION] = 5;    // Position
			saveStorage();
		}
		AHRSStart();
		storage.sendIndex = 0;
	}

	void saveStorage(void) {
		unsigned char data[4], checksum;
		unsigned short i;
		float * ptr;
		ptr = (float *) data;
		
		AHRSStop();
		checksum = IDENTITY;
		EEPROMWriteByte(0, (unsigned char)storage.param[0]); // write ID
		for(i=1; i<STORAGE_DATA; i++) {
			*ptr = storage.param[i];
			// EEPROMWrite(4*i-3, data, 4);
			// EEPROMWrite sometimes corrupts data, use single writes if that is the case
			EEPROMWriteByte(4*i-3, data[0]);
			EEPROMWriteByte(4*i-2, data[1]);
			EEPROMWriteByte(4*i-1, data[2]);
			EEPROMWriteByte(4*i-0, data[3]);
			checksum += data[0];
			checksum += data[1];
			checksum += data[2];
			checksum += data[3];
		}
		
		for(i=0; i<STORAGE_RATES; i++) {
			EEPROMWriteByte(STORAGE_RATES_START+i, dataRate[i]);
			checksum += dataRate[i];
		}
		
		checksum += EEPROM_VERSION;
		EEPROMWriteByte(STORAGE_END, checksum);
		AHRSStart();
	}

	void heartbeat(void) {
		mavlink_msg_heartbeat_encode(mavlink_system.sysid, mavlink_system.compid, &mavlink_tx_msg, &mavlink_heartbeat);
		mavlink_message_len = mavlink_msg_to_send_buffer(mavlink_message_buf, &mavlink_tx_msg);
		UARTWrite(mavlink_message_buf, mavlink_message_len);
	}

	void setHomeFloat(float latitude, float longitude, float altitude) {
		waypoint[WAYPOINT_HOME].x = latitude;
		waypoint[WAYPOINT_HOME].y = longitude;
		waypoint[WAYPOINT_HOME].z = altitude + WAYPOINT_SAFE_HEIGHT;
		waypoint[WAYPOINT_HOME].param1 = WAYPOINT_MIN_RADIUS;
		waypoint[WAYPOINT_HOME].param2 = 0;
		waypoint[WAYPOINT_HOME].param3 = 0;
		waypoint[WAYPOINT_HOME].param4 = 0;
		waypoint[WAYPOINT_HOME].command = MAV_CMD_NAV_LAND;
		waypoint[WAYPOINT_HOME].autocontinue = 0;
		GPS.hasHome = 1;
	}
	void setHomeInt(signed int latitude, signed int longitude, signed int altitude) {
		setHomeFloat((float)latitude/10000000, (float)longitude/10000000, (float)altitude/1000);
	}

	void mavlinkSendText(unsigned char severity, char * text) {
		mavlink_statustext.severity = severity;
		strcpy((char*) mavlink_statustext.text, text);
		mavlink_msg_statustext_encode(mavlink_system.sysid, mavlink_system.compid, &mavlink_tx_msg, &mavlink_statustext);
		mavlink_message_len = mavlink_msg_to_send_buffer(mavlink_message_buf, &mavlink_tx_msg);
		UARTWrite(mavlink_message_buf, mavlink_message_len);
	}

	void mavlinkSendDebugV(char * name, float x, float y, float z) {
		strcpy(mavlink_debug_vect.name, name);
		mavlink_debug_vect.time_usec = sysTime * 1000;
		mavlink_debug_vect.x = x;
		mavlink_debug_vect.y = y;
		mavlink_debug_vect.z = z;
		mavlink_msg_debug_vect_encode(mavlink_system.sysid, mavlink_system.compid, &mavlink_tx_msg, &mavlink_debug_vect);
		mavlink_message_len = mavlink_msg_to_send_buffer(mavlink_message_buf, &mavlink_tx_msg);
		UARTWrite(mavlink_message_buf, mavlink_message_len);
	}

	void AHRSStop(void) {
		Timer1DisableInterrupt();
	}

	void AHRSStart(void) {
		if(Timer1Read() > (1000000/AHRS_RATE)) { // Timer 1 range is set by interrupt, disabling interrupt disrupt
			Timer1Reset();
			Timer1Go();
		}
		Timer1EnableInterrupt();
	}

	void calibrate(void) {
		calibrateBaro();
		calibrateGyro();
		LEDOff(ALL);
	}

	void calibrateGyro(void) {
		unsigned short i;
		signed short sensor[4];
		
		Gyro.X.offset = 0;
		Gyro.Y.offset = 0;
		Gyro.Z.offset = 0;
		Gyro.X.verterrorInt = 0;
		Gyro.Y.verterrorInt = 0;
		Gyro.Z.verterrorInt = 0;
		Gyro.X.error = 0;
		Gyro.Y.error = 0;
		Gyro.Z.error = 0;
		Gyro.X.total = 0;
		Gyro.Y.total = 0;
		Gyro.Z.total = 0;
		
		for(i=0; i<GYRO_AVERAGING; i++) {
			Gyro.X.history[i] = 0;
			Gyro.Y.history[i] = 0;
			Gyro.Z.history[i] = 0;
		}
		Gyro.count = 0;
		
		for(i=0;i<OFFSET_CALC;i++) {
			GyroGetData(sensor);
			Gyro.X.raw = sensor[0];
			Gyro.Y.raw = sensor[1];
			Gyro.Z.raw = sensor[2];

			Gyro.Y.offset += (float)Gyro.Y.raw/(float)OFFSET_CALC;
			Gyro.X.offset += (float)Gyro.X.raw/(float)OFFSET_CALC;
			Gyro.Z.offset += (float)Gyro.Z.raw/(float)OFFSET_CALC;
			
			Gyro.X.total -= Gyro.X.history[Gyro.count];
			Gyro.X.history[Gyro.count] = Gyro.X.raw;
			Gyro.X.total += Gyro.X.raw;
			
			Gyro.Y.total -= Gyro.Y.history[Gyro.count];
			Gyro.Y.history[Gyro.count] = Gyro.Y.raw;
			Gyro.Y.total += Gyro.Y.raw;
			
			Gyro.Z.total -= Gyro.Z.history[Gyro.count];
			Gyro.Z.history[Gyro.count] = Gyro.Z.raw;
			Gyro.Z.total += Gyro.Z.raw;

			if(++Gyro.count >= GYRO_AVERAGING) Gyro.count = 0;
			
			LEDWrite(0x1 << ((Gyro.count>>2) & 0x3));
			Delay(1000/AHRS_RATE);
		}
	}
	
	void calibrateRC(void) {
		Delay(INPUT_AVERAGING * 20);
		input[0].offset = (unsigned int)input[0].total / INPUT_AVERAGING;
		input[1].offset = (unsigned int)input[1].total / INPUT_AVERAGING;
		input[2].offset = (unsigned int)input[2].total / INPUT_AVERAGING;
		input[3].offset = (unsigned int)input[3].total / INPUT_AVERAGING;
	}

	void calibrateBaro(void) {
		Baro.total = 0;
		Baro.reading = 0;
		
		BaroGetTemp();
		BaroStartPress();
		Delay(5);
		
		for(Baro.count=0; Baro.count<BARO_AVERAGING; Baro.count++) {
			Baro.raw = BaroReadPress();
			Baro.history[Baro.count] = Baro.raw;
			Baro.total += Baro.raw;

			BaroStartPress();
			LEDWrite(0x8 >> ((Baro.count>>2) & 0x3));
			Delay(5);
		}
		altitude = 0;
		lastAltitude = 0;
		Baro.count = 0;
		Baro.altitudeOffset = 0;
	}

	unsigned char isGrounded(void) {
		// use two types of test, test to see if gyros are static
		if(fabsf(Gyro.X.raw) < GROUNDTHRESH && fabsf(Gyro.Y.raw) < GROUNDTHRESH && fabsf(Gyro.Z.raw) < GROUNDTHRESH) {
			// and then try peturbing the system
			if(groundTest > 20 && groundTest < 30) {
				if(groundTest % 2) yaw.demand += 0.002;
				else yaw.demand -= 0.002;
			}
			else if(groundTest < 50) {
				// try something more drastic
				if(groundTest % 2) pitch.demand = 0.02;
				else pitch.demand = 0.02;
			}
			else if(groundTest >= 60) {
				return GROUNDED_IS_GROUNDED;
			}
			groundTest++;
			return GROUNDED_MAYBE_GROUNDED;
		}
		else {
			groundTest = 0;
		}
		return GROUNDED_NOT_GROUNDED;
	}

	// fast inverse square root
	float invSqrt(float x) {
		union {
			float f;
			int i;
		} tmp;
		tmp.f = x;
		tmp.i = 0x5f3759df - (tmp.i >> 1);
		float y = tmp.f;
		return y * (1.5f - 0.5f * x * y * y);
	}

	float fatan2(float y, float x) {
		if (x == 0.0f) {
			if (y > 0.0f) return M_PI_2;
			if (y == 0.0f) return 0.0f;
			return -M_PI_2;
		}
		float atan;
		float z = y/x;
		if (fabsf(z) < 1.0f) {
			atan = z/(1.0f + 0.28f*z*z);
			if (x < 0.0f) {
				if (y < 0.0f) return atan - M_PI;
				return atan + M_PI;
			}
		}
		else {
			atan = M_PI_2 - z/(z*z + 0.28f);
			if (y < 0.0f) return atan - M_PI;
		}
		return atan;
	}

	float fasin(float x) {
		float temp, arcsin, xabs;
		xabs = fabsf(x);
		temp = M_PI_2 - (1.5707288f + (-0.2121144f + (0.0742610f - 0.0187293f*xabs)*xabs)*xabs)/invSqrt(1-xabs);
		arcsin = copysignf(temp, x);
		return arcsin;
	}


	float fsin(float x) {
		const float B = 4/M_PI;
		const float C = -4/(M_PI*M_PI);

		float y = B * x + C * x * fabsf(x);

		
		//  const float Q = 0.775;
		const float P = 0.225;

		y = P * (y * fabsf(y) - y) + y;   // Q * y + P * y * abs(y)
		return y;
	}

	void configGPS(void) {
		// Set up navigation message 0x01 0x02 at 5Hz
		unsigned char config[12] = {SPH_GPS_ADDR, 0xb5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x02, 0x01, 0x0e, 0x47};
		I2CMaster(config, 12, 0, 0);

		// Set up status message 0x01 0x03 at 1Hz
		config[8] = 0x03;
		config[9] = 0x05;    // Every five navigation solution (i.e. 1Hz)
		config[10] = 0x13;
		config[11] = 0x4D;
		I2CMaster(config, 12, 0, 0);

		// Set up velocity message 0x01 0x12 at 1Hz
		config[8] = 0x12;
		config[10] = 0x22;
		config[11] = 0x6B;
		I2CMaster(config, 12, 0, 0);
		
		GPS.fix = 0;
		GPS.firstFix = 0;
		GPS.hasHome = 0;
	}

*/