(* Helpful information:

https://mavlink.io/en/guide/serialization.html
https://github.com/mavlink/c_library_v2/tree/master/common

MsgID	MsgID	*	MAVlink Message
0	$000		HEARTBEAT
1	$001		SYS_STATUS
2	$002	x	SYSTEM_TIME
4	$004		PING
24	$018	x	GPS_RAW_INT
25	$019	x	GPS_STATUS
29      $01D            SCALED_PRESSURE
30	$01E		ATTITUDE
31	$01F		ATTITUDE_QUATERNION
32	$020		LOCAL_POSITION_NED
33	$021	*	GLOBAL_POSITION_INT
36	$024		SERVO_OUTPUT_RAW
65	$041		RC_CHANNELS
69	$045		MANUAL_CONTROL
70	$046		RC_CHANNELS_OVERRIDE
74	$04A		VFR_HUD
76	$04C		COMMAND_LONG
77	$04D		COMMAND_ACK
83	$053		ATTITUDE_TARGET
85	$055		POSITION_TARGET_LOCAL_NED
87	$057		POSITION_TARGET_GLOBAL_INT
105	$069		HIGHRES_IMU
111	$06F		TIMESYNC
140	$08C            ACTUATOR_CONTROL_TARGET
141	$08D		ALTITUDE
147	$093		BATTERY_STATUS
148	$094		AUTOPILOT_VERSION
230	$0E6		ESTIMATOR_STATUS
231	$0E7		WIND_COV
241	$0F1		VIBRATION
242	$0F2		HOME_POSITION
245	$0F5		EXTENDED_SYS_STATE
253	$0FD		STATUSTEXT
259	$103		CAMERA_INFORMATION
260	$104		CAMERA_SETTINGS
261	$105		STORAGE_INFORMATION
262	$106		CAMERA_CAPTURE_STATUS
264	$108		FLIGHT_INFORMATION
265	$109		MOUNT_ORIENTATION
322	$142		PARAM_EXT_VALUE
323	$143		PARAM_EXT_SET
324	$144		PARAM_EXT_ACK
340	$154		UTM_GLOBAL_POSITION

*)

unit mav_msg;                                       {Decode MAVlink messages}

{$mode objfpc}{$H+}

interface

uses
  sysutils, mav_def, DateUtils;

{Public functions and procedures}
procedure SYS_STATUS(const msg: TMAVmessage; offset: byte; var data: TGPSdata);
procedure SYS_TIME(const msg: TMAVmessage; offset: byte; var data: TGPSdata);
procedure GPS_RAW_INT(const msg: TMAVmessage; offset: byte; var data: TGPSdata);
procedure GPS_STATUS(const msg: TMAVmessage; offset: byte; var data: TGPSdata);  {Possibly never used by Yuneec or empty}
procedure SCALED_PRESSURE(const msg: TMAVmessage; offset: byte; var data: THWstatusData);
procedure ATTITUDE(const msg: TMAVmessage; offset: byte; var data: TAttitudeData);
procedure LOCAL_POSITION_NED(const msg: TMAVmessage; offset: byte; var data: TAttitudeData);
procedure GLOBAL_POSITION_INT(const msg: TMAVmessage; offset: byte; var data: TGPSdata);
procedure VRF_HUD(const msg: TMAVmessage; offset: byte; var data: TAttitudeData);
procedure MOUNT_ORIENTATION(const msg: TMAVmessage; offset: byte; var data: TAttitudeData);

function STATUSTEXT(const msg: TMAVmessage; offset: byte; SeveritySeparator: char='|'): string;
function YuneecTimeStampInSeconds(const msg: TMAVmessage): uint64;

implementation

procedure SYS_STATUS(const msg: TMAVmessage; offset: byte; var data: TGPSdata);
var
  SensorFlags: THWFlags;

begin
  with SensorFlags do begin
    sensor_present:=MavGetUInt32(msg, offset);
    sensor_enabled:=MavGetUInt32(msg, offset+4);
    sensor_healthy:=MavGetUInt32(msg, offset+8);

    if sensor_present>0 then begin
      data.gps_present:=((sensor_present and $04)>0) and           {mag}
                        ((sensor_present and $20)>0);              {gps}
      data.sensors_OK:=(sensor_present=sensor_enabled) and
                       (sensor_present=sensor_healthy);
    end;
  end;

  data.load:=MavGetUInt16(msg, offset+12);
  data.voltage:=MavGetUInt16(msg, offset+14);


end;

procedure SYS_TIME(const msg: TMAVmessage; offset: byte; var data: TGPSdata);
begin
  data.timeUTC:=UnixToDateTime(MavGetUInt64(msg, offset) div 1000000);  {us --> s};
  if msg.msglength=10 then
    data.boottime:=MilliSecondsToDateTime(MavGetUInt32(msg, offset+8) and $FFFF)
  else
    if msg.msglength=11 then
      data.boottime:=MilliSecondsToDateTime(MavGetUInt32(msg, offset+8) and $FFFFFF)
    else
      data.boottime:=MilliSecondsToDateTime(MavGetUInt32(msg, offset+8));
end;

procedure GPS_RAW_INT(const msg: TMAVmessage; offset: byte; var data: TGPSdata);
begin
  data.boottime:=MavGetUInt64(msg, offset)/MilliSecondsPerDay/1000;
  data.lat:=MavGetInt32(msg, offset+8);
  data.lon:=MavGetInt32(msg, offset+12);
  data.altMSL:=MavGetInt32(msg, offset+16);
  data.eph:=MavGetUInt16(msg, offset+20);
  data.epv:=MavGetUInt16(msg, offset+22);
  data.vel:=MavGetUInt16(msg, offset+24);
  data.cog:=MavGetUInt16(msg, offset+26);
  data.fix_type:=msg.msgbytes[offset+28];
  data.sats_visible:=msg.msgbytes[offset+29];

  if msg.msglength>30 then begin
    data.alt_ellipsoid:=MavGetInt32(msg, offset+30);
    data.h_acc:=MavGetUInt32(msg, offset+34);
    data.v_acc:=MavGetUInt32(msg, offset+38);
    data.vel_acc:=MavGetUInt32(msg, offset+42);

    if msg.msglength=49 then begin                       {Yuneec specific}
      data.hdg_acc:=MavGetUInt16(msg, offset+46);
      data.yaw:=msg.msgbytes[offset+48];
    end else begin                                       {msg length = 52}
      data.hdg_acc:=MavGetUInt32(msg, offset+46);
      data.yaw:=MavGetUInt16(msg, offset+50);
    end;
  end;
end;

procedure GPS_STATUS(const msg: TMAVmessage; offset: byte; var data: TGPSdata); {Possibly never used by Yuneec or empty}
var
  i: integer;

begin
  data.sats_visible:=msg.msgbytes[offset];
  for i:=0 to MAVsatCount do begin
    data.sat_prn[i]:=msg.msgbytes[i+offset+1];
    data.sat_used[i]:=msg.msgbytes[i+offset+21];
    data.sat_elevation[i]:=msg.msgbytes[i+offset+41];
    data.sat_azimuth[i]:=msg.msgbytes[i+offset+61];
    data.sat_snr[i]:=msg.msgbytes[i+offset+81];
  end;
end;

procedure SCALED_PRESSURE(const msg: TMAVmessage; offset: byte; var data: THWstatusData);
begin
  data.boottime:=MavGetUInt32(msg, offset)/MilliSecondsPerDay;
  data.pressure_abs:=MavGetFloat(msg, offset+4);
  data.pressure_diff:=MavGetFloat(msg, offset+8);
  data.baro_temp:=MavGetUInt16(msg, offset+12);
end;

procedure ATTITUDE(const msg: TMAVmessage; offset: byte; var data: TAttitudeData);
begin
  data.boottime:=MavGetUInt32(msg, offset)/MilliSecondsPerDay;

  data.roll:=RadToDegree180(MavGetFloat(msg, offset+4));
  data.pitch:=RadToDegree180(MavGetFloat(msg, offset+8));
  data.yaw:=RadToDegree360(MavGetFloat(msg, offset+12));

  data.rollspeed:=MavGetFloat(msg, offset+16);
  data.pitchspeed:=MavGetFloat(msg, offset+20);
  data.yawspeed:=MavGetFloat(msg, offset+24);
end;

procedure LOCAL_POSITION_NED(const msg: TMAVmessage; offset: byte; var data: TAttitudeData);
begin
  data.boottime:=MavGetUInt32(msg, offset)/MilliSecondsPerDay;

  data.posx:=MavGetFloat(msg, offset+4);
  data.posy:=MavGetFloat(msg, offset+8);
  data.posz:=MavGetFloat(msg, offset+12);

  data.vx:=MavGetFloat(msg, offset+16);
  data.vy:=MavGetFloat(msg, offset+20);
  data.vz:=MavGetFloat(msg, offset+24);
end;

procedure GLOBAL_POSITION_INT(const msg: TMAVmessage; offset: byte; var data: TGPSdata);
begin
  data.boottime:=MavGetUInt32(msg, offset)/MilliSecondsPerDay;

  data.lat:=MavGetInt32(msg, offset+4);
  data.lon:=MavGetInt32(msg, offset+8);
  data.altMSL:=MavGetInt32(msg, offset+12);
  data.alt_rel:=MavGetInt32(msg, offset+16);

  data.vx:=MavGetInt16(msg, offset+20);
  data.vy:=MavGetInt16(msg, offset+22);
  data.vz:=MavGetInt16(msg, offset+24);
  data.hdg:=MavGetUInt16(msg, offset+26);
end;

procedure VRF_HUD(const msg: TMAVmessage; offset: byte; var data: TAttitudeData);
begin
  data.airspeed:=MavGetFloat(msg, offset);
  data.groundspeed:=MavGetFloat(msg, offset+4);
  data.altmsl:=MavGetFloat(msg, offset+8);               {For Yuneec: altitude relative}
  data.climbrate:=MavGetFloat(msg, offset+12);

  if msg.msglength=20 then begin
    data.heading:=MavGetInt16(msg, offset+16);
    data.throttle:=MavGetUInt16(msg, offset+18);         {Throttle [%]}
  end else
    data.heading:=msg.msgbytes[offset+16];               {Yuneec specific}
end;

procedure MOUNT_ORIENTATION(const msg: TMAVmessage; offset: byte; var data: TAttitudeData);
begin
  data.boottime:=MavGetUInt32(msg, offset)/MilliSecondsPerDay;

  data.roll:=MavGetFloat(msg, offset+4);
  data.pitch:=MavGetFloat(msg, offset+8);
  data.yaw:=MavGetFloat(msg, offset+12);
  data.yaw_abs:=MavGetFloat(msg, offset+16);
end;


function STATUSTEXT(const msg: TMAVmessage; offset: byte; SeveritySeparator: char='|'): string;
var
  i: integer;
begin
  result:=SeverityToStr(msg.msgbytes[offset])+SeveritySeparator;
  for i:=1 to msg.msglength-1 do begin
    if msg.msgbytes[offset+i] in [10, 13, 32..127] then
      result:=result+chr(msg.msgbytes[offset+i]);
  end;
end;

function YuneecTimeStampInSeconds(const msg: TMAVmessage): uint64;
begin                                                    {+2 Time starts after! CRC}
  if (msg.msgformat=3) or (msg.msgformat=4) or (msg.msgformat=5) then
    result:=MavGetUInt64(msg, msg.msglength+LengthFixPartFD+2) div 1000
  else
    result:=MavGetUInt64Reverse(msg, msg.msglength+LengthFixPartFD+2) div 1000000;
end;

end.
