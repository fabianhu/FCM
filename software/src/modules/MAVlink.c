/*
 * MAVlink.c
 * This is a basic implementation of a MAVlink protocol engine.
 * 
 * Created: 07.02.2014 20:32:24
 *
 * (c) 2014-2015 by Fabian Huslik (with portions of a mavlink example, of course!)
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */ 
#include <asf.h>
#include "modules/types.h"
#include "FabOS_config.h"
#include "FabOS/FabOS.h"
#include "MAVlink.h"
#include "modules/usart.h"

#include "mavlink/mavlink_types.h"
mavlink_system_t mavlink_system; // this has to be after types and before mavlink.h
#include "mavlink/common/mavlink.h"

#include "menu/menu.h" // for accessing the parameters
#include "menu/menu_variant.h"

#if COM_MAVLINK_AND_GPS == 1
	#if MAX_ITEM_NAME_CHARLENGTH >16
	#warning "parameter names will be cut!"
	#endif
	#if MAX_ITEM_NAME_CHARLENGTH >12
	#warning "child parameter names will be cut!"
	#endif
#endif

#define MAVPARCOUNT MENUE_PARCOUNT

MAVparam_t MAVParameterIndex[MAVPARCOUNT]; // index is the MAVLINK parameter ID

MAVAbo_t MavLink_abos[MAVMAXABOS];

OS_DeclareQueue(MavQueue,500,1); // 500 = nearly twice the max length

uint32_t _MAV_processSendMessages(void);
int _MAV_registerAbo(uint8_t id, uint16_t rate, uint8_t state);
void _MAV_ProduceStreamMessage(uint8_t id);
void _MAV_PrepareParameters(void);


void MAV_init(void)
{
	mavlink_system.sysid = MAVSYSTEMID; // System ID, 1-255
	mavlink_system.compid = MAV_COMP_ID_IMU; // Component/Subsystem ID, 1-255
	_MAV_PrepareParameters();
		
	_MAV_registerAbo(MAVLINK_MSG_ID_HEARTBEAT,1000,1); // switch on heartbeat
	
}

void _MAV_PrepareParameters(void)
{
	extern MenuItem_t m_items[MENUESIZE];
	
	int n = 0;
	for(int i=0;i<MENUESIZE;i++)
	{
		if(m_items[i].pParam != NULL)
		{
			if(m_items[i].ucParent != 0)
			{
				strncpy(MAVParameterIndex[n].name,m_items[m_items[i].ucParent].strName,3); // copy first three letters of parent
				MAVParameterIndex[n].name[3] = '_'; // separate by underscore for grouping
				strncpy(&(MAVParameterIndex[n].name[4]),m_items[i].strName,12 ); // copy name
			}
			else
			{
				strncpy(MAVParameterIndex[n].name,m_items[i].strName,16 ); // copy name
			}
			
			MAVParameterIndex[n].pValue = &(m_items[i].pParam->sValue);
			MAVParameterIndex[n].upper = (m_items[i].pParam->sUpperLimit);
			MAVParameterIndex[n].lower = (m_items[i].pParam->sLowerLimit);						
			n++;
		}
	}
}

void MAV_Process(void) // requires cyclic call (with no excessive wait around)
{
	// Initialize the required buffers
	static mavlink_message_t mavRXmsg; // debug static
	uint8_t* pRXData = NULL;

	mavlink_status_t mavRXstate;
	mavRXstate.msg_received = 0;
		
	while(OS_QueueOut(&MavQueue,pRXData) == 0)
	{
		if(mavlink_parse_char(0,*pRXData,&mavRXmsg,&mavRXstate) == 1)
		{
			// process the message 
			mavlink_param_request_list_t prl;
			mavlink_param_request_read_t prr;
			mavlink_request_data_stream_t rds;
			mavlink_param_set_t psv;
			mavlink_command_long_t mcmd;
			
			switch (mavRXmsg.msgid)
			{
				case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
					// send parameter list
					mavlink_msg_param_request_list_decode(&mavRXmsg,&prl);
					if(prl.target_system == MAVSYSTEMID)
					{
						// iterate over parameters
						for(int i=0;i<MAVPARCOUNT;i++)
						{
							mavlink_param_union_t pu;
							pu.xx_u.param_int32 = *MAVParameterIndex[i].pValue; // fixme wrong type selected due to byte swap problem. should be int16.
							mavlink_msg_param_value_send(MAVLINK_COMM_0, MAVParameterIndex[i].name, pu.xx_u.param_float, MAV_PARAM_TYPE_INT16,MAVPARCOUNT,i);
						}
					}
					break;
				case MAVLINK_MSG_ID_PARAM_REQUEST_READ:
					// Request to read the onboard parameter with the param_id string id. 
					mavlink_msg_param_request_read_decode(&mavRXmsg,&prr);
					if(prr.target_system == MAVSYSTEMID && prr.param_index >=0) // no string-read implemented!
					{
						// iterate over parameters
						mavlink_param_union_t pu;
						pu.xx_u.param_int16 = *MAVParameterIndex[prr.param_index].pValue;
						mavlink_msg_param_value_send(MAVLINK_COMM_0,MAVParameterIndex[prr.param_index].name,pu.xx_u.param_float,MAV_PARAM_TYPE_INT16,MAVPARCOUNT,prr.param_index);
					}
					break;
				case MAVLINK_MSG_ID_PARAM_SET:
					// Set a parameter value TEMPORARILY to RAM
					mavlink_msg_param_set_decode(&mavRXmsg,&psv);
					if(psv.target_system == MAVSYSTEMID && psv.target_component == MAVCOMPONENT)
					{
						// iterate over parameters
						for(int i=0;i<MAVPARCOUNT;i++)
						{ 
							if(strncmp(psv.param_id,MAVParameterIndex[i].name,16)==0)
							{
								mavlink_param_union_t pu;
								pu.xx_u.param_float = psv.param_value;
								int16_t test = pu.xx_u.param_int32; // fixme dirty workaround, should be param_int16
								if(test >= MAVParameterIndex[i].lower && test <= MAVParameterIndex[i].upper)
								{
									// set parameter
									*MAVParameterIndex[i].pValue = test;
									
									// send answer
									mavlink_msg_param_value_send(MAVLINK_COMM_0,MAVParameterIndex[i].name,pu.xx_u.param_float,MAV_PARAM_TYPE_INT16,MAVPARCOUNT,i);
								}
								break; // stop comparing, if found.
							}
						}
					}
					break;
				case MAVLINK_MSG_ID_COMMAND_LONG:
					// execute command
					mavlink_msg_command_long_decode(&mavRXmsg,&mcmd);
					if(mcmd.target_system == MAVSYSTEMID)
					{
						uint8_t cmdresult = MAV_RESULT_FAILED;
						switch(mcmd.command)
						{
							case MAV_CMD_PREFLIGHT_STORAGE:
								if(mcmd.param1 == 1.0) // aua-aua-aua-aua-aua-aua-aua-aua-aua-aua-aua-aua-aua-aua-aua-aua-aua-aua Böse Schmerzen...
								{
									actSaveToFlash();
									cmdresult = MAV_RESULT_ACCEPTED;
								}
								break;
							default:
								cmdresult = MAV_RESULT_UNSUPPORTED;
								break;
						}
						// every command gets answered with the result:
						mavlink_msg_command_ack_send(MAVLINK_COMM_0,mcmd.command,cmdresult);

					}
					break;	
				case MAVLINK_MSG_ID_REQUEST_DATA_STREAM:
					// Data stream IDs. A data stream is not a fixed set of messages, but rather a recommendation to the autopilot software. Individual autopilots may or may not obey the recommended messages.
					mavlink_msg_request_data_stream_decode(&mavRXmsg,&rds);
					if(rds.target_system == MAVSYSTEMID) // is it for me?
					{
						switch (rds.req_stream_id)
						{
						case MAV_DATA_STREAM_ALL: // Enable all data streams
							_MAV_registerAbo(MAVLINK_MSG_ID_ATTITUDE_QUATERNION,1000/rds.req_message_rate,rds.start_stop);
							
							_MAV_registerAbo(MAVLINK_MSG_ID_SYS_STATUS,1000/rds.req_message_rate,rds.start_stop);
							_MAV_registerAbo(MAVLINK_MSG_ID_GPS_RAW_INT,1000/rds.req_message_rate,rds.start_stop);
							_MAV_registerAbo(MAVLINK_MSG_ID_GPS_STATUS,1000/rds.req_message_rate,rds.start_stop);
							_MAV_registerAbo(MAVLINK_MSG_ID_SCALED_IMU,1000/rds.req_message_rate,rds.start_stop);
							_MAV_registerAbo(MAVLINK_MSG_ID_RAW_IMU,1000/rds.req_message_rate,rds.start_stop);
							_MAV_registerAbo(MAVLINK_MSG_ID_RAW_PRESSURE,1000/rds.req_message_rate,rds.start_stop);
							_MAV_registerAbo(MAVLINK_MSG_ID_SCALED_PRESSURE,1000/rds.req_message_rate,rds.start_stop);
							_MAV_registerAbo(MAVLINK_MSG_ID_ATTITUDE,1000/rds.req_message_rate,rds.start_stop);
							_MAV_registerAbo(MAVLINK_MSG_ID_ATTITUDE_QUATERNION,1000/rds.req_message_rate,rds.start_stop);
							_MAV_registerAbo(MAVLINK_MSG_ID_LOCAL_POSITION_NED,1000/rds.req_message_rate,rds.start_stop);
							_MAV_registerAbo(MAVLINK_MSG_ID_GLOBAL_POSITION_INT,1000/rds.req_message_rate,rds.start_stop);
							_MAV_registerAbo(MAVLINK_MSG_ID_RC_CHANNELS_SCALED,1000/rds.req_message_rate,rds.start_stop);
							_MAV_registerAbo(MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN,1000/rds.req_message_rate,rds.start_stop);
							_MAV_registerAbo(MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT,1000/rds.req_message_rate,rds.start_stop);
							_MAV_registerAbo(MAVLINK_MSG_ID_HIGHRES_IMU,1000/rds.req_message_rate,rds.start_stop);
							_MAV_registerAbo(MAVLINK_MSG_ID_DEBUG,1000/rds.req_message_rate,rds.start_stop);
												
							break;
						case MAV_DATA_STREAM_RAW_SENSORS: // Enable IMU_RAW, GPS_RAW, GPS_STATUS packets.
							break;
						case MAV_DATA_STREAM_EXTENDED_STATUS: // Enable GPS_STATUS, CONTROL_STATUS, AUX_STATUS
							break;
						case MAV_DATA_STREAM_RC_CHANNELS: // Enable RC_CHANNELS_SCALED, RC_CHANNELS_RAW, SERVO_OUTPUT_RAW
							break;
						case MAV_DATA_STREAM_RAW_CONTROLLER: // Enable ATTITUDE_CONTROLLER_OUTPUT, POSITION_CONTROLLER_OUTPUT, NAV_CONTROLLER_OUTPUT.
							break;
						case MAV_DATA_STREAM_POSITION: // Enable LOCAL_POSITION, GLOBAL_POSITION/GLOBAL_POSITION_INT messages.
							break;
						default:
							break;
						}
						
						mavlink_msg_data_stream_send(MAVLINK_COMM_0,rds.req_stream_id,rds.req_message_rate,rds.start_stop);
					}
					break;					
				default:
						
					break;
			}
		}
	}
	
	// here produce all the requested messages
	uint32_t waittime = _MAV_processSendMessages();
	OS_WaitEventTimeout(OSEVT_MAVRX,OSALM_COMM0WAIT,waittime); // wait calculated time or wake up to process a received message
}

// in RX ISR copy the packet (or the single characters) to the MavLink task
void MAV_CopyPacket(uint8_t* ptr, uint32_t len) 
{
	for(uint32_t i = 0; i< len; i++)
	{
		if(OS_QueueIn(&MavQueue,&(ptr[i]))) break;
	}
	OS_SetEventFromISR(OSTSK_COMM0,OSEVT_MAVRX);
}

void MAV_SendAndWait(const uint8_t* buf, uint32_t len)
{
	USART_Send(0,(uint8_t*)buf, len); // fixme modularize !
	uint32_t n= (len / 10)+1; // special calculation for 115200 baud -> 11520 byte / s -> 11 byte / ms -> wait at least (len * 11 +1) ms
	OS_WaitTicks(OSALM_COMM0WAIT,n);
}

// register mavlink abo ; return 1 on error
int _MAV_registerAbo(uint8_t id, uint16_t rate, uint8_t state)
{
	uint32_t error = 1;
	for (int i = 0; i<MAVMAXABOS; i++)
	{
		if(state == 1 && MavLink_abos[i].active == 0) // switch on, find inactive entry
		{
			MavLink_abos[i].active = 1;
			MavLink_abos[i].stream_ID = id;
			MavLink_abos[i].rate = rate;
			error = 0;
			break;
		}
		else
		if(state == 0 && MavLink_abos[i].stream_ID == id) // switch off, find ID
		{
			MavLink_abos[i].active = 0;
			error = 0;
			break;
		}
		
	}
	return error;
}

#define ONBOARDSENSORS 0x00001;



// send the requested message and wait till it is away
void _MAV_ProduceStreamMessage(uint8_t id) 
{
	uint64_t time = OS_GetTicks()*1000;
	
	// Pack the message
	switch (id)
	{
		case MAVLINK_MSG_ID_HEARTBEAT:
			mavlink_msg_heartbeat_send(MAVLINK_COMM_0, MAV_TYPE_GENERIC, MAV_AUTOPILOT_GENERIC_WAYPOINTS_AND_SIMPLE_NAVIGATION_ONLY, MAV_MODE_MANUAL_ARMED, 0, MAV_STATE_ACTIVE);
			break;
		case MAVLINK_MSG_ID_SYS_STATUS:
			mavlink_msg_sys_status_send(MAVLINK_COMM_0,	0, 0, 0, 1, 1, -1, 1, 0, 0, 0, 0, 0, 0);
			break;
		case MAVLINK_MSG_ID_GPS_RAW_INT:
			mavlink_msg_gps_raw_int_send(MAVLINK_COMM_0,time,3,1000,10000,11,0,0,0,0,8);
			break;
		case MAVLINK_MSG_ID_GPS_STATUS:
			//mavlink_msg_gps_status_send(MAVLINK_COMM_0,8,1,0,0,0,0); // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!  pointers into the dark!
			break;
		case MAVLINK_MSG_ID_SCALED_IMU:
			mavlink_msg_scaled_imu_send(MAVLINK_COMM_0,time,0,0,0,0,0,0,0,0,0);
			break;
		case MAVLINK_MSG_ID_RAW_IMU:
			mavlink_msg_raw_imu_send(MAVLINK_COMM_0,time,0,0,0,0,0,0,0,0,0);
			break;
		case MAVLINK_MSG_ID_RAW_PRESSURE:
			mavlink_msg_raw_pressure_send(MAVLINK_COMM_0,time,1,1,1,1);
			break;
		case MAVLINK_MSG_ID_SCALED_PRESSURE:
			mavlink_msg_scaled_pressure_send(MAVLINK_COMM_0,time,1,1,1);
			break;
		case MAVLINK_MSG_ID_ATTITUDE:
			mavlink_msg_attitude_send(MAVLINK_COMM_0,time,0,0,0,0,0,0);
			break;
		case MAVLINK_MSG_ID_ATTITUDE_QUATERNION:
			mavlink_msg_attitude_quaternion_send(MAVLINK_COMM_0,time,1,0,0,0,0,0,0);
			break;
		case MAVLINK_MSG_ID_LOCAL_POSITION_NED:
			mavlink_msg_local_position_ned_send(MAVLINK_COMM_0,OS_GetTicks(),0,0,0,0,0,0);
			break;
		case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
			mavlink_msg_global_position_int_send(MAVLINK_COMM_0,OS_GetTicks(),11,11,2,10,1,1,1,180);
			break;
		case MAVLINK_MSG_ID_RC_CHANNELS_SCALED:
			mavlink_msg_rc_channels_scaled_send(MAVLINK_COMM_0,OS_GetTicks(),0,INT16_MAX,INT16_MAX,INT16_MAX,INT16_MAX,INT16_MAX,INT16_MAX,INT16_MAX,INT16_MAX,0);
			break;
		case MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN:
			mavlink_msg_gps_global_origin_send(MAVLINK_COMM_0,111,111,4 );
			break;
		case MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT:
			mavlink_msg_nav_controller_output_send(MAVLINK_COMM_0,0,0,0,0,0,0,0,0 );
			break;
		case MAVLINK_MSG_ID_HIGHRES_IMU:
			mavlink_msg_highres_imu_send(MAVLINK_COMM_0,time,0,0,0,0,0,0,0,0,0,0,0,0,0,0 );
			break;
		case MAVLINK_MSG_ID_DEBUG:
			mavlink_msg_debug_send(MAVLINK_COMM_0,OS_GetTicks(), 0, 7.7 ); // debug value with index
		
		default:
			break;
	}
}

uint32_t _MAV_processSendMessages(void) // return time to next due message
{
	// Is a response due, send this.
	// loop through all abos
	// If message is due, process during loop + break;
	// if no message is due, wait the remaining time in caller. (waked up by received message or the timeout)
	
	static uint32_t time;
	time = OS_GetTicks();
	static uint32_t waittime; 
	
	waittime = 5000; // minimum every 5s loop back here.
	
	static uint32_t remaining;
	
	for (int i = 0; i<MAVMAXABOS; i++)
	{
		if(MavLink_abos[i].active == 1) // send, if active
		{
			remaining = (MavLink_abos[i].timestamp + MavLink_abos[i].rate)-time;
			if(remaining < 3 || remaining > 60000) // or overflow
			{
				MavLink_abos[i].timestamp = time;
				_MAV_ProduceStreamMessage(MavLink_abos[i].stream_ID); // process the message
				waittime = min(waittime,MavLink_abos[i].rate);
			}
			else
			{
				// find shortest delay and forward this
				waittime = min(waittime,remaining);
			}
		}
		else
		{
			// just ignore inactives.
		}
		
	}
	return waittime;
}
