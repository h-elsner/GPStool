(* Helpful information:

https://mavlink.io/en/guide/serialization.html
https://github.com/mavlink/c_library_v2/tree/master/common

*)

unit mav_gps;                                       {MAVlink definitions and variables}

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

type
  TMAVmessage = record
    msglength, msgid: integer;
    msgbytes: array[0..maxLenMAVmsg] of byte;
    sysid, targetid: byte;
    valid: boolean;
  end;

type
  TGPSdata = record
// GPS data from GPS_raw_int (24)
    boottime, timeUTC, yuneectime: TDateTime;
    lat, lon: int32;               {[degE7] WGS84, EGM96 ellipsoid}
    altMSL: int32;                 {[mm] Altitude (MSL). Positive for up}
    eph, epv, vel, cog: uint16;
    fix_type: byte;
    alt_ellipsoid: int32;          {[mm] Altitude (above WGS84, EGM96 ellipsoid)}
    h_acc, v_acc, vel_acc: uint32; {[mm] uncertainty}
    hdg_acc: uint32;               {[degE5] Heading / track uncertainty}
    yaw: uint32; {[cdeg] Yaw in earth frame from north. Use 36000 for north.}

// Satellites data from GPS_status (25)
    sats_visible: byte;
    sat_prn:       array[0..MAVsatCount] of byte;        {Global satellite ID}
    sat_used:      array[0..MAVsatCount] of byte;        {0 not, 1 used}
    sat_elevation: array[0..MAVsatCount] of byte;        {[deg] Elevation (0: right on top of receiver, 90: on the horizon) of satellite}
    sat_azimuth:   array[0..MAVsatCount] of byte;        {[deg] Direction of satellite}
    sat_snr:       array[0..MAVsatCount] of byte;        {[dB] Signal to noise ratio of satellite}
  end;

{Public functions and procedures}
procedure ClearMAVmessage(var msg: TMAVmessage);
procedure GPSdata_SetDefaultValues(var GPSvalues: TGPSdata);
function MavGetUInt64(const msg: TMAVmessage; pos: integer): uint64;
function MAVGetUINT64Reverse(const msg: TMAVmessage; pos: integer): uint64;
function MavGetInt16(const msg: TMAVmessage; pos: integer): int16;
function MavGetUInt16(const msg: TMAVmessage; pos: integer): uint16;
function MavGetUInt32(const msg: TMAVmessage; pos: integer): uint32;
function MavGetInt32(const msg: TMAVmessage; pos: integer): int32;
function MicroSecondsToDateTime(const t: uint64): TDateTime;
function MilliSecondsToDateTime(const t: uint64): TDateTime;
function YuneecTimeStampInSeconds(const msg: TMAVmessage): uint64;
function SatAzimuthToDeg(const azi: byte): single;       {[deg] Direction of satellite, 0: 0 deg, 255: 360 deg}
function SatElevationToDeg(const ele: byte): single;     {[deg] Elevation of satellite, 0: 0 deg, 255: 90 deg}
function FormatCoordinates(const coord: int32): string;
function FormatAltitude(const alt: int32): string;       {[mm]}
function FormatDOP(const dop: uint32): string;
function FormatSpeed(const vel: uint32): string;
function FormatHdg(const hdg: uint32): string;
function FixTypeToStr(const fixtype: byte): string;      {MAVlink GPS fix type to string}
function FormatAcc(const acc: uint32): string;           {[mm]}
function FormatVelAcc(const acc: uint32): string;        {[mm]}
function FormatHdgAcc(const acc: uint32): string;        {[degE5] Heading / track uncertainty}

implementation

procedure ClearMAVmessage(var msg: TMAVmessage);
var
  i: integer;

begin
  with msg do begin
    msglength:=0;
    sysid:=0;
    targetid:=0;
    valid:=false;
    for i:=0 to maxLenMAVmsg do
      msgbytes[i]:=0;                                    {Fix part empty}
  end;
end;

procedure GPSdata_SetDefaultValues(var GPSvalues: TGPSdata);
begin
  with GPSvalues do begin
    timeUTC:=0;
    boottime:=0;
    yuneectime:=0;
    sats_visible:=max8;
    eph:=max16;
    epv:=max16;
    vel:=max16;
    cog:=max16;
    yaw:=0;
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
  b: byte;

begin
  result:=0;
  for i:=0 to 3 do begin
    result:=result+msg.msgbytes[pos+i]*(256**i);
  end;
end;

function MicroSecondsToDateTime(const t: uint64): TDateTime;
begin
  result:=t/MilliSecondsPerDay/1000;
end;

function MilliSecondsToDateTime(const t: uint64): TDateTime;
begin
  result:=t/MilliSecondsPerDay;
end;

function YuneecTimeStampInSeconds(const msg: TMAVmessage): uint64;
begin                                                    {+2 Time starts after! CRC}
  result:=MavGetUInt64Reverse(msg, msg.msglength+LengthFixPartFD+2) div 1000000;
end;

function SatAzimuthToDeg(const azi: byte): single;       {[deg] Direction of satellite, 0: 0 deg, 255: 360 deg}
begin
  result:=azi*360/255;
end;

function SatElevationToDeg(const ele: byte): single;     {[deg] Elevation of satellite, 0: 0 deg, 255: 90 deg}
begin
  result:=ele*90/255;
end;

function FormatCoordinates(const coord: int32): string;
begin
  result:=FormatFloat(floatformat8, coord/10000000);     {[degE7]}
end;

function FormatAltitude(const alt: int32): string;       {[mm]}
begin
  result:=FormatFloat(floatformat2, alt/1000)+'m';
end;

function FormatDOP(const dop: uint32): string;
begin
  result:='';
  if dop<max16 then
    result:=FormatFloat(floatformat2, dop/100);
end;

function FormatSpeed(const vel: uint32): string;
begin
  result:='';
  if vel<max16 then
    result:=FormatFloat(floatformat2, vel/100)+'m/s';
end;

function FormatHdg(const hdg: uint32): string;
begin
  result:='';
  if hdg<max16 then
    result:=FormatFloat(floatformat2, hdg/100)+'°';
end;

function FixTypeToStr(const fixtype: byte): string;      {MAVlink GPS fix type to string}
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

function FormatAcc(const acc: uint32): string;           {[mm]}
begin
  result:=FormatFloat(floatformat1, acc/10)+'cm';
end;

function FormatVelAcc(const acc: uint32): string;        {[mm/s]}
begin
  result:=FormatFloat(floatformat2, acc/1000)+'m/s';
end;

function FormatHdgAcc(const acc: uint32): string;        {[degE5] Heading / track uncertainty}
begin
  result:=FormatFloat(floatformat3, acc/100000)+'°';
end;

end.
