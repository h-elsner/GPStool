(* Helpful information:

https://mavlink.io/en/guide/serialization.html
https://github.com/mavlink/c_library_v2/tree/master/common

MAVLINK_LITTLE_ENDIAN
*)

unit mav_def;                                 {MAVlink definitions and variables}

{$mode objfpc}{$H+}

interface

uses
  sysutils, math;

const
  MAVsatCount=19;                                        {20 sats means 0..19}
  LengthFixPartBC=6;
  LengthFixPartFD=10;
  MagicBC=$BC;
  MagicFD=$FD;
  MilliSecondsPerDay=86400000;
  max8=255;
  max16=65535;
  maxLenMAVmsg=280;

  timefull='yyyy-mm-dd hh:nn:ss';
  timezzz='nn:ss.zzz';
  floatformat8='0.00000000';
  floatformat3='0.000';
  floatformat2='0.00';
  floatformat1='0.0';

  rsNotSpecified='Not specified';
  rsNotProvided='Not provided';
  rsUnknown='Unkown';

type
  TMAVmessage = record
    msglength, msgid: uint16;
    msgbytes: array[0..maxLenMAVmsg] of byte;
    sysid, targetid: byte;
    valid: boolean;
  end;

type

// Add here some data records for different analysis cases

  TGPSdata = record
// GPS data from GPS_raw_int (24)
    boottime, timeUTC, yuneectime: TDateTime;
    lat, lon: int32;               {[degE7] WGS84, EGM96 ellipsoid}
    altMSL, alt_rel: int32;        {[mm] Altitude. Positive for up}
    eph, epv, vel, cog, hdg: uint16;
    fix_type: byte;
    alt_ellipsoid: int32;          {[mm] Altitude (above WGS84, EGM96 ellipsoid)}
    h_acc, v_acc, vel_acc: uint32; {[mm] uncertainty}
    hdg_acc: uint32;               {[degE5] Heading / track uncertainty}
    yaw: uint32; {[cdeg] Yaw in earth frame from north. Use 36000 for north.}
    vx, vy, vz: int16;
    load, voltage: uint16;
    gps_present, sensors_OK: boolean;

// Satellites data from GPS_status (25)
    sats_visible: byte;
    sat_prn:       array[0..MAVsatCount] of byte;        {Global satellite ID}
    sat_used:      array[0..MAVsatCount] of byte;        {0 not, 1 used}
    sat_elevation: array[0..MAVsatCount] of byte;        {[deg] Elevation (0: right on top of receiver, 90: on the horizon) of satellite}
    sat_azimuth:   array[0..MAVsatCount] of byte;        {[deg] Direction of satellite}
    sat_snr:       array[0..MAVsatCount] of byte;        {[dB] Signal to noise ratio of satellite}
  end;

type
  TAttitudeData = record
    boottime: TDateTime;
    altMSL, alt_rel: single;                             {[m] Altitude. Positive for up}
    posx, posy, posz: single;                            {m}
    roll, pitch, yaw, yaw_abs: single;                   {deg}
    rollspeed, pitchspeed, yawspeed: single;             {rad/s}
    vx, vy, vz, airspeed, groundspeed, climbrate: single; {m/s}
    heading, throttle: int16;                            {°, %}
  end;

type
  THWstatusData = record
    boottime: TDateTime;
    rssi: byte;
    Vcc, current: UInt16;                                {mV, mA}
    load, droprate, batt: UInt16;                        {% or unitless}
    baro_temp, IMU_temp: Int16;                          {cdeg}
    pressure_abs, pressure_diff: float;                  {hPa}
  end;

type
  THWFlags = record
    sensor_present, sensor_enabled, sensor_healthy: UInt32;

  end;


{Public functions and procedures}
procedure ClearMAVmessage(var msg: TMAVmessage);
procedure ClearAttitudeData(var data: TAttitudeData);
procedure GPSdata_SetDefaultValues(var GPSvalues: TGPSdata);

function MavGetUInt64(const msg: TMAVmessage; pos: integer): uint64;
function MAVGetUINT64Reverse(const msg: TMAVmessage; pos: integer): uint64;  {Big endian}
function MavGetInt16(const msg: TMAVmessage; pos: integer): int16;
function MavGetUInt16(const msg: TMAVmessage; pos: integer): uint16;
function MavGetUInt32(const msg: TMAVmessage; pos: integer): uint32;
function MavGetInt32(const msg: TMAVmessage; pos: integer): int32;
function MavGetFloat(const msg: TMAVmessage; pos: integer): single;

function MicroSecondsToDateTime(const t: uint64): TDateTime;
function MilliSecondsToDateTime(const t: uint64): TDateTime;
function SatAzimuthToDeg(const azi: byte): single;       {[deg] Direction of satellite, 0: 0 deg, 255: 360 deg}
function SatElevationToDeg(const ele: byte): single;     {[deg] Elevation of satellite, 0: 0 deg, 255: 90 deg}

function FormatCoordinates(const coord: int32): shortstring;
function FormatAltitude(const alt: int32): shortstring;       {[mm]}
function FormatDOP(const dop: uint32): shortstring;
function FormatSpeed(const vel: uint32): shortstring;
function FormatXYZSpeed(const vel: int32): shortstring;
function FormatHdg(const hdg: uint32): shortstring;
function FormatDeziProcent(const prz: uint16): shortstring;
function FormatMilliVolt(const volt: uint16): shortstring;
function FixTypeToStr(const fixtype: byte): shortstring;      {MAVlink GPS fix type to string}

function BatteryTypeToStr(const batttype: byte): shortstring;
function BatteryFunctionToStr(const battfunc: byte): shortstring;
function BatteryChargeStateToStr(const battstate: byte): shortstring;

function FormatAcc(const acc: uint32): shortstring;           {[mm]}
function FormatVelAcc(const acc: uint32): shortstring;        {[mm]}
function FormatHdgAcc(const acc: uint32): shortstring;        {[degE5] Heading / track uncertainty}
function SeverityToStr(const severity: byte): shortstring;
function RadToDegree180(const radangle: single): single;      {rad to ° +/-180}
function RadToDegree360(const radangle: single): single;      {rad to ° 0..360, 0 is north}

implementation

procedure ClearMAVmessage(var msg: TMAVmessage);
begin
  msg:=default(TMAVmessage);
  msg.valid:=false;
end;

procedure ClearAttitudeData(var data: TAttitudeData);
begin
  data:=default(TAttitudeData);
end;

procedure GPSdata_SetDefaultValues(var GPSvalues: TGPSdata);
begin
  GPSvalues:=default(TGPSdata);                               {Set all to zero}
  with GPSvalues do begin
    sats_visible:=max8;
    eph:=max16;
    epv:=max16;
    vel:=max16;
    cog:=max16;
    hdg:=max16;
    voltage:=max16;
    sensors_OK:=true;
    gps_present:=true;
  end;
end;

function MavGetUInt64(const msg: TMAVmessage; pos: integer): uint64;
var
  i: integer;

begin
  result:=0;
  for i:=0 to 7 do begin
    result:=result+msg.msgbytes[pos+i]*(256**i);
  end;
end;

function MAVGetUINT64Reverse(const msg: TMAVmessage; pos: integer): uint64;
var
  i: integer;

begin
  result:=0;
  for i:=0 to 7 do begin
    result:=result+msg.msgbytes[pos+i]*(256**(7-i));
  end;
end;

function MavGetInt16(const msg: TMAVmessage; pos: integer): int16;
begin
  result:=msg.msgbytes[pos]+msg.msgbytes[pos+1]*256;
end;

function MavGetUInt16(const msg: TMAVmessage; pos: integer): uint16;
begin
  result:=msg.msgbytes[pos]+msg.msgbytes[pos+1]*256;
end;

function MavGetUInt32(const msg: TMAVmessage; pos: integer): uint32;
var
  i: integer;

begin
  result:=0;
  for i:=0 to 3 do begin
    result:=result+msg.msgbytes[pos+i]*(256**i);
  end;
end;

function MavGetInt32(const msg: TMAVmessage; pos: integer): int32;
var
  i: integer;

begin
  result:=0;
  for i:=0 to 3 do begin
    result:=result+msg.msgbytes[pos+i]*(256**i);
  end;
end;

function MavGetFloat(const msg: TMAVmessage; pos: integer): single;
var i: integer;
    wfl: packed array[0..3] of Byte;
    wx: Single absolute wfl;

begin
  result:=0;
  for i:=0 to 3 do                                       {Endianess prüfen (to/downto)}
    wfl[i]:=msg.msgbytes[i+pos];                         {4 byte aus Buffer ausschneiden}
  result:=wx;                                            {Typecast mittels absolute}
end;

function MicroSecondsToDateTime(const t: uint64): TDateTime;
begin
  result:=t/MilliSecondsPerDay/1000;
end;

function MilliSecondsToDateTime(const t: uint64): TDateTime;
begin
  result:=t/MilliSecondsPerDay;
end;

function SatAzimuthToDeg(const azi: byte): single;       {[deg] Direction of satellite, 0: 0 deg, 255: 360 deg}
begin
  result:=azi*360/255;
end;

function SatElevationToDeg(const ele: byte): single;     {[deg] Elevation of satellite, 0: 0 deg, 255: 90 deg}
begin
  result:=ele*90/255;
end;

function FormatCoordinates(const coord: int32): shortstring;
begin
  result:=FormatFloat(floatformat8, coord/10000000);     {[degE7]}
end;

function FormatAltitude(const alt: int32): shortstring;       {[mm]}
begin
  result:=FormatFloat(floatformat2, alt/1000)+'m';
end;

function FormatDOP(const dop: uint32): shortstring;
begin
  result:='';
  if dop<max16 then
    result:=FormatFloat(floatformat2, dop/100);
end;

function FormatSpeed(const vel: uint32): shortstring;
begin
  result:='';
  if vel<max16 then
    result:=FormatFloat(floatformat2, vel/100)+'m/s';
end;

function FormatXYZSpeed(const vel: int32): shortstring;
begin
  result:=FormatFloat(floatformat2, vel/100)+'m/s';
end;

function FormatHdg(const hdg: uint32): shortstring;
begin
  result:='';
  if hdg<max16 then
    result:=FormatFloat(floatformat2, hdg/100)+'°';
end;

function FormatDeziProcent(const prz: uint16): shortstring;
begin
  result:='';
  if prz<1001 then
    result:=FormatFloat(floatformat1, prz/10)+'%';
end;

function FormatMilliVolt(const volt: uint16): shortstring;
begin
  result:='';
  if (volt>0) and (volt<max16) then
    result:=FormatFloat(floatformat2, volt/1000)+'V';
end;


function FixTypeToStr(const fixtype: byte): shortstring; {MAVlink GPS fix type to string}
begin
  result:='';
  case fixtype of
    0:	Result:='No GPS connected';
    1:	Result:='No position information, GPS is connected';
    2:	Result:='2D position';
    3:	Result:='3D position';
    4:	Result:='DGPS/SBAS aided 3D position';
    5:	Result:='RTK float, 3D position';
    6:	Result:='RTK fixed, 3D position';
    7:	Result:='Static fixed, typically used for base stations';
    8:	Result:='PPP, 3D position';
  end;
end;

function BatteryTypeToStr(const batttype: byte): shortstring;
begin
  result:='';
  case batttype of
    0: result:=rsNotSpecified;
    1: result:='LiPo';
    2: result:='LiFe';
    3: result:='LiIon';
    4: result:='NiMH';
  end;
end;

function BatteryFunctionToStr(const battfunc: byte): shortstring;
begin
  result:='';
  case battfunc of
    0: result:=rsNotSpecified;
    1: result:='All flight systems';
    2: result:='Propulsion system';
    3: result:='Avionics';
    4: result:='Payload';
  end;
end;

function BatteryChargeStateToStr(const battstate: byte): shortstring;
begin
  result:='';
  case battstate of
    0: result:=rsNotProvided;
    1: result:='Normal operation';
    2: result:='Low';
    3: result:='Critical';
    4: result:='Emergency';
    5: result:='Failed';
    6: result:='Unhealthy';
    7: result:='Charging';
  end;
end;

{ENUMs see: https://github.com/mavlink/c_library_v2/blob/master/common/common.h}
function SeverityToStr(const severity: byte): shortstring;
begin
  result:=IntToStr(severity);  {Default if unknown}
  case severity of
    0: result:='EMERGENCY';    {System is unusable. This is a "panic" condition}
    1: result:='ALERT';        {Action should be taken immediately. Indicates error
                                in non-critical systems}
    2: result:='CRITICAL';     {Action must be taken immediately. Indicates failure
                                in a primary system}
    3: result:='ERROR';        {Indicates an error in secondary/redundant systems}
    4: result:='WARNING';      {Indicates about a possible future error if this
                                is not resolved within a given timeframe. Example
                                would be a low battery warning}
    5: result:='NOTICE';       {An unusual event has occurred, though not an error
                                condition. This should be investigated for the root cause.}
    6: result:='INFO';         {Normal operational messages. Useful for logging.
                                No action is required for these messages.}
    7: result:='DEBUG';        {Useful non-operational messages that can assist in
                                debugging. These should not occur during normal operation}
  end;
end;


function FormatAcc(const acc: uint32): shortstring;           {[mm]}
begin
  result:=FormatFloat(floatformat2, acc/10000)+'cm';
end;

function FormatVelAcc(const acc: uint32): shortstring;        {[mm/s]}
begin
  result:=FormatFloat(floatformat3, acc/100000)+'m/s';
end;

function FormatHdgAcc(const acc: uint32): shortstring;        {[degE5] Heading / track uncertainty}
begin
  result:=FormatFloat(floatformat3, acc/100000)+'°';
end;

function RadToDegree180(const radangle: single): single;      {rad to ° +/-180}
begin
  result:=radangle*180/pi;
end;

function RadToDegree360(const radangle: single): single;      {rad to ° 0..360, 0 is north}
begin
  result:=(RadToDegree180(radangle)+360) mod 360;
end;

end.
