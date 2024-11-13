(*
btnLoad.Tag misused for file type. 0 for invalid file or file magic for known file types


References
----------
Polar diagram:
https://www.lazarusforum.de/viewtopic.php?p=66139#p66139

MAVlink messages:


 2: https://github.com/mavlink/c_library_v2/blob/master/common/mavlink_msg_system_time.h
24: https://github.com/mavlink/c_library_v2/blob/master/common/mavlink_msg_gps_status.h
25: https://github.com/mavlink/c_library_v2/blob/master/common/mavlink_msg_gps_raw_int.h
*)

unit GPStool_main;

{$mode objfpc}{$H+}

interface

uses
  Classes, SysUtils, Forms, Controls, Graphics, Dialogs, ActnList, StdCtrls,
  ExtCtrls, Buttons, ComCtrls, Grids, FileUtil, DateUtils, TAGraph, TATypes,
  TASeries, TAChartUtils, TAGeometry, TARadialSeries, TASources, mav_gps;

const
  clSatUsed=clLime;
  clSatVisible=clMoneyGreen;
  clPolarLabel=clSkyBlue;
  PolarSatSize=8;
  tab1=' ';
  tab2='  ';
  sep=';';                                               {CSV separator}

type

  { TForm1 }

  TForm1 = class(TForm)
    actClose: TAction;
    actContinue: TAction;
    actEnd: TAction;
    actHalt: TAction;
    actAbout: TAction;
    actSaveChart: TAction;
    actSaveGrid: TAction;
    actSaveCSV: TAction;
    actLoadFile: TAction;
    ActionList1: TActionList;
    BarSatSNR: TBarSeries;
    btnClose: TBitBtn;
    btnLoad: TBitBtn;
    cbSaveCSV: TCheckBox;
    lblTime: TLabel;
    SatPolar: TChart;
    SatPolarSeries: TPolarSeries;
    ChartSatSNR: TChart;
    gbPolar: TGroupBox;
    gbData: TGroupBox;
    gbSatSNR: TGroupBox;
    ImageList1: TImageList;
    SatPolarSource: TListChartSource;
    SatSNRBarSource: TListChartSource;
    OpenDialog: TOpenDialog;
    plUser: TPanel;
    plData: TPanel;
    btnContinue: TSpeedButton;
    btnHalt: TSpeedButton;
    btnEnd: TSpeedButton;
    StatusBar: TStatusBar;
    gridGPSdata: TStringGrid;
    barSleepTime: TTrackBar;
    procedure actCloseExecute(Sender: TObject);
    procedure actContinueExecute(Sender: TObject);
    procedure actEndExecute(Sender: TObject);
    procedure actHaltExecute(Sender: TObject);
    procedure actLoadFileExecute(Sender: TObject);
    procedure actSaveCSVExecute(Sender: TObject);
    procedure FormClose(Sender: TObject; var CloseAction: TCloseAction);
    procedure SatPolarAfterDrawBackWall(ASender: TChart; ACanvas: TCanvas;
      const ARect: TRect);
    procedure FormCreate(Sender: TObject);
  private
    procedure CreateSatPolarDiagram(const sats: TGPSdata);
    procedure CreateSatSNRBarChart(const sats: TGPSdata);
    procedure WriteGridGPSdataValues(const sats: TGPSdata);
    procedure PrepareSatSNRBarChart;
    procedure PrepareSatPolarDiagram;
    procedure ClearPositioningData;
    procedure WriteGridGPSdataHeader;
    procedure ResetGlobalVariables;
    procedure WriteHeaderCSVlist;
    procedure WriteDataCSVlist(const sats: TGPSdata);
    procedure InfoMessageToStatusbar(info: string);
  public
    function  DecodeOneSensorMessage(const msg: TMAVmessage; offset: byte; var data: TGPSdata): boolean;
    procedure ProcessMAVFile(const UsedMagic: byte);
    procedure PreparePolarAxes(AChart: TChart; AMax: Double);
    procedure DrawPolarAxes(AChart: TChart; AMax, ADelta: Double; MeasurementUnit: string);
  end;

var
  Form1: TForm1;
  stop: boolean;
  NumMsgUsed, NumMsgTotal: integer;
  csvlist: TStringList;
  inputstream: TMemoryStream;
  PositionInStream: uint64;

{$I GPStool_en.inc}

implementation

{$R *.lfm}

procedure TForm1.FormCreate(Sender: TObject);
begin
  Caption:=capApplication;
  gbSatSNR.Caption:=capSatSNR;
  gbSatSNR.Hint:=hntSatSNR;
  gbData.Caption:=capData;
  gbData.Hint:=hntData;
  gbPolar.Caption:=capPolar;
  gbPolar.Hint:=hntPolar;
  cbSaveCSV.Caption:=capSaveCSV;
  cbSaveCSV.Hint:=hntSaveCSV;
  actClose.Caption:=capClose;
  actClose.Hint:=hntClose;
  actLoadFile.Caption:=capLoadFile;
  actLoadFile.Hint:=hntLoadFile;
  StatusBar.Hint:=hntStatusbar;
  OpenDialog.Title:=capOpenDialog;
  lblTime.Caption:=capTime;

  WriteGridGPSdataHeader;
  PrepareSatSNRBarChart;
  BarSatSNR.Clear;
  PrepareSatPolarDiagram;
  PreparePolarAxes(SatPolar, 90);
  SatPolarSeries.Clear;
  barSleepTime.Hint:=hntSleepTime;

  barSleepTime.Position:=100;
  cbSaveCSV.Checked:=false;

  stop:=false;
  NumMsgUsed:=0;
  NumMsgTotal:=0;

  csvlist:=TStringList.Create;
  inputstream:=TMemoryStream.Create;
  inputstream.Position:=0;
end;

procedure TForm1.FormClose(Sender: TObject; var CloseAction: TCloseAction);
begin
  if assigned(csvlist) then
    csvlist.free;
  if assigned(inputstream) then
    inputstream.Free;
end;

procedure TForm1.ResetGlobalVariables;
begin
  NumMsgUsed:=0;
  NumMsgTotal:=0;
  csvlist.Clear;
  inputstream.Position:=0;
  inputstream.Size:=0;
  PositionInStream:=0;                                   {Position of processing in input stream}
  stop:=false;
end;

procedure TForm1.PrepareSatPolarDiagram;                 {Settings for the polar diagram}
begin
  with SatPolarSeries do begin
    Source:=SatPolarSource;
    Marks.Style:=smsLabel;
    LinePen.Style:=psClear;
    ShowPoints:=true;
    Marks.LabelBrush.Color:=clPolarLabel;
    Pointer.Style:=psHexagon;
    Pointer.HorizSize:=PolarSatSize;
    Pointer.VertSize:=PolarSatSize;
    Pointer.Pen.Style:=psClear;
    Pointer.Visible:=true;
  end;
end;

procedure TForm1.PrepareSatSNRBarChart;                  {Settings for the SNR chart}
begin
  with ChartSatSNR do begin                              {for the whole chart}
    Title.Visible:=false;
    LeftAxis.Title.Caption:=capSatSNR+' [db]';
    LeftAxis.Title.Visible:=true;
    BottomAxis.Marks.Source:=SatSNRBarSource;
    BottomAxis.Marks.Style:=smsLabel;
    BottomAxis.Grid.Visible:=false;
    BottomAxis.Marks.LabelFont.Orientation := 900;       {gedreht}
  end;

  with BarSatSNR do begin                                {For the bar serie}
    Source:=SatSNRBarSource;
    SeriesColor:=clSatUsed;
  end;
end;

procedure TForm1.WriteGridGPSdataHeader;
begin
  gridGPSdata.BeginUpdate;
  try
    gridGPSdata.RowCount:=18;
    gridGPSdata.Cells[0, 0]:='Time since boot';
    gridGPSdata.Cells[0, 1]:='SYSTEM TIME UTC';
    gridGPSdata.Cells[0, 2]:='Time UTC per Message';
    gridGPSdata.Cells[0, 3]:='Latitude (WGS84 EGM96 ellipsoid)';
    gridGPSdata.Cells[0, 4]:='Longitude (WGS84 EGM96 ellipsoid)';
    gridGPSdata.Cells[0, 5]:='Altitude (MSL)';
    gridGPSdata.Cells[0, 6]:='Number of satellites visible';
    gridGPSdata.Cells[0, 7]:='GPS HDOP';
    gridGPSdata.Cells[0, 8]:='GPS VDOP';
    gridGPSdata.Cells[0, 9]:='GPS ground speed';
    gridGPSdata.Cells[0, 10]:='Course over ground';
    gridGPSdata.Cells[0, 11]:='GPS fix type';
    gridGPSdata.Cells[0, 12]:='Altitude (above WGS84 EGM96 ellipsoid)';
    gridGPSdata.Cells[0, 13]:='Position uncertainty';
    gridGPSdata.Cells[0, 14]:='Altitude uncertainty';
    gridGPSdata.Cells[0, 15]:='Speed uncertainty';
    gridGPSdata.Cells[0, 16]:='Heading / track uncertainty';
    gridGPSdata.Cells[0, 17]:='Yaw in earth frame from north';
    gridGPSdata.AutoSizeColumn(0);
  finally
    gridGPSdata.EndUpdate;
  end;
end;

procedure TForm1.WriteHeaderCSVlist;
var
  i: integer;
  header: string;

begin
  csvlist.Clear;
  header:=gridGPSdata.Cells[0, 0];
  for i:=1 to gridGPSdata.RowCount-1 do
    header:=header+sep+gridGPSdata.Cells[0, i];
// Add additional items at the end (possibly for checking something
// + sep + header item
  csvlist.Add(header);                                   {Add header to CSV file}
end;

/////////////////////////////// Actions ////////////////////////////////////////

procedure TForm1.actCloseExecute(Sender: TObject);
begin
  stop:=true;
  Close;
end;

procedure TForm1.InfoMessageToStatusbar(info: string);
begin
  StatusBar.Panels[2].Text:=info;
end;

procedure TForm1.actContinueExecute(Sender: TObject);
begin
  stop:=false;
  if btnLoad.Tag>0 then begin                            {Means, there was is a valid file type in progress}
    ProcessMAVfile(btnLoad.Tag);
  end;
end;

procedure TForm1.actEndExecute(Sender: TObject);
begin
  stop:=true;
  PositionInStream:=0;
  StatusBar.Panels[0].Text:=IntToStr(NumMsgTotal);
  StatusBar.Panels[1].Text:=IntToStr(NumMsgUsed);
  actSaveCSVExecute(self);
end;

procedure TForm1.actHaltExecute(Sender: TObject);
begin
  stop:=true;
  StatusBar.Panels[0].Text:=IntToStr(NumMsgTotal);
  StatusBar.Panels[1].Text:=IntToStr(NumMsgUsed);
end;

procedure TForm1.actLoadFileExecute(Sender: TObject);
var
  PureFileName: string;

  procedure SetForValidFile;
  begin
    InfoMessageToStatusbar(OpenDialog.FileName);
    Caption:=capApplication+tab2+PureFileName;
  end;

  procedure SetForInvalidFile(ErrorMessage: string);
  begin
    Caption:=capApplication;
    StatusBar.Panels[0].Text:='0';
    StatusBar.Panels[1].Text:='0';
    InfoMessageToStatusbar(ErrorMessage);
  end;

begin
  if OpenDialog.Execute then begin
    btnLoad.Tag:=0;                                       {Invalid file; Tag misused for file type}
    ResetGlobalVariables;
    BarSatSNR.Clear;
    SatPolarSeries.Clear;
    ClearPositioningData;
    WriteHeaderCSVlist;

    if FileSize(OpenDialog.FileName)<500 then begin
      SetForInvalidFile(errSmallFile);
      exit;
    end;
    PureFileName:=ExtractFileName(OpenDialog.FileName);
    if (pos('.tlog', LowerCase(PureFileName))>1) or
       (pos('.log', LowerCase(PureFileName))>1)then begin
      SetForValidFile;
      btnLoad.Tag:=MagicFD;

    end else begin
      if pos('Sensor_', PureFileName)=1 then begin
        SetForValidFile;
        btnLoad.Tag:=MagicBC;
      end else begin
        SetForInvalidFile(errInvalidFile);
      end;
    end;
    inputstream.LoadFromFile(OpenDialog.FileName);
    ProcessMAVfile(btnLoad.Tag);

    StatusBar.Panels[0].Text:=IntToStr(NumMsgTotal);
    StatusBar.Panels[1].Text:=IntToStr(NumMsgUsed);
    actSaveCSVExecute(self);
  end;
end;

procedure TForm1.actSaveCSVExecute(Sender: TObject);
var
  ChangedFileName: string;

begin
  if cbSaveCSV.Checked and (csvlist.Count>1) then begin
    ChangedFileName:=ChangeFileExt(OpenDialog.FileName, '')+'_GPStool.csv';
    csvlist.SaveToFile(ChangedFileName);
    InfoMessageToStatusbar(ExtractFileName(ChangedFileName+tab1+rsSaved));
  end;
end;

///////////////////////////// Processing ///////////////////////////////////////

function TForm1.DecodeOneSensorMessage(const msg: TMAVmessage; offset: byte; var data: TGPSdata): boolean;

  procedure SYS_TIME;
  begin
    result:=true;
    data.timeUTC:=UnixToDateTime(MavGetUInt64(msg, offset) div 1000000);  {us --> s};
    data.yuneectime:=UnixToDateTime(YuneecTimeStampInSeconds(msg));
    if msg.msglength=10 then
      data.boottime:=MilliSecondsToDateTime(MavGetUInt32(msg, offset+8) and $FFFF)
    else
      if msg.msglength=11 then
        data.boottime:=MilliSecondsToDateTime(MavGetUInt32(msg, offset+8) and $FFFFFF)
      else
        data.boottime:=MilliSecondsToDateTime(MavGetUInt32(msg, offset+8));
  end;

  procedure GPS_RAW_INT;
  begin
    result:=true;
    data.boottime:=MavGetUInt64(msg, offset)/MilliSecondsPerDay/1000;
    data.yuneectime:=UnixToDateTime(YuneecTimeStampInSeconds(msg));
    data.lat:=MavGetInt32(msg, offset+8);
    data.lon:=MavGetInt32(msg, offset+12);
    data.altMSL:=MavGetInt32(msg, offset+16);
    data.eph:=MavGetUInt16(msg, offset+20);
    data.epv:=MavGetUInt16(msg, offset+22);
    data.vel:=MavGetUInt16(msg, offset+24);
    data.cog:=MavGetUInt16(msg, offset+26);
    data.fix_type:=msg.msgbytes[offset+28];
    data.sats_visible:=msg.msgbytes[offset+29];
    data.alt_ellipsoid:=MavGetInt32(msg, offset+30);
    data.h_acc:=MavGetUInt32(msg, offset+34);
    data.v_acc:=MavGetUInt32(msg, offset+38);
    data.vel_acc:=MavGetUInt32(msg, offset+42);

//  Yuneec specific
    data.hdg_acc:=MavGetUInt16(msg, offset+46);
    data.yaw:=msg.msgbytes[offset+48];
  end;

// On each message is a trailer after the CRC with a time stamp 8 bytes UTG in µs

  procedure GPS_STATUS;                                  {Possibly never used by Yuneec or empty}
  var
    i: integer;

  begin
    result:=true;
    data.yuneectime:=UnixToDateTime(YuneecTimeStampInSeconds(msg));
    data.sats_visible:=msg.msgbytes[offset];
    for i:=0 to MAVsatCount do begin
      data.sat_prn[i]:=msg.msgbytes[i+offset+1];
      data.sat_used[i]:=msg.msgbytes[i+offset+21];
      data.sat_elevation[i]:=msg.msgbytes[i+offset+41];
      data.sat_azimuth[i]:=msg.msgbytes[i+offset+61];
      data.sat_snr[i]:=msg.msgbytes[i+offset+81];
    end;
    sleep(barSleepTime.Position);
  end;

begin
  result:=false;
  data.sats_visible:=max8;
  case msg.msgid of
     2: SYS_TIME;
    24: GPS_RAW_INT;
    25: GPS_STATUS;
  end;
end;

procedure TForm1.ProcessMAVFile(const UsedMagic: byte);
var
  b: byte;
  msg: TMAVmessage;
  data: TGPSdata;

  procedure SetFixPartValuesForMsgType;
  begin
    case UsedMagic of
      MagicFD: begin
        // Read all message bytes from stream: Fix part + Payload + CRC + 8 bytes YuneeTimestamp [µs]
        inputstream.ReadBuffer(msg.msgbytes, msg.msglength+LengthFixPartFD+10);
        msg.sysid:=msg.msgbytes[5];
        msg.targetid:=msg.msgbytes[6];
        msg.msgid:=MavGetInt32(msg, 7) and $FFFFFF;      {MAVlink Message ID has 3 bytes}
      end;

      MagicBC: begin
        // Read all message bytes from stream: Fix part + Payload + CRC
        inputstream.ReadBuffer(msg.msgbytes, msg.msglength+LengthFixPartBC+2);
        msg.sysid:=msg.msgbytes[3];
        msg.targetid:=msg.msgbytes[4];
        msg.msgid:=msg.msgbytes[5];
      end;
    end;
  end;

  function CheckMessage: boolean;
  begin
    result:=false;
    case UsedMagic of
      MagicFD: begin
        result:=msg.msgid>1;

      end;
      MagicBC: begin
        result:=msg.msgid>1;

      end;
    end;
  end;

begin
  inputstream.Position:=PositionInStream;                {Position of processing in input stream}
  ClearMAVmessage(msg);
  GPSdata_SetDefaultValues(data);
  while inputstream.Position<inputstream.Size-100 do begin
    repeat                                               {Try to find next UsedMagic in stream}
      b:=inputstream.ReadByte;
    until (b=UsedMagic) or (inputstream.Position>=inputstream.Size-LengthFixPartFD-12);

    msg.msgbytes[0]:=b;
    msg.msglength:=inputstream.ReadByte;                 {Lenght payload is without CRC}
    msg.msgbytes[1]:=msg.msglength;
    inputstream.Position:=inputstream.Position-2;

    SetFixPartValuesForMsgType;
    inc(NumMsgTotal);
    if CheckMessage then begin
      if DecodeOneSensorMessage(msg, LengthFixPartBC, data) then begin
        inc(NumMsgUsed);
        if data.boottime>0 then begin
          if data.sats_visible<>max8 then begin
            CreateSatPolarDiagram(data);
            CreateSatSNRBarChart(data);
            WriteGridGPSdataValues(data);
            WriteDataCSVlist(data);
          end;
        end;
        Application.ProcessMessages;
        if stop then begin
          PositionInStream:=inputstream.Position;
          exit;
        end;
      end;
    end;
  end;
end;


procedure TForm1.ClearPositioningData;
var
  i: integer;

begin
  lblTime.Caption:=capTime;
  for i:=0 to gridGPSdata.RowCount-1 do
    gridGPSdata.Cells[1, i]:='';
end;

procedure TForm1.CreateSatPolarDiagram(const sats: TGPSdata);
var
  azi, ele: single;
  IndicatorColor: TColor;
  i: integer;

begin
  SatPolar.DisableRedrawing;
  try
    SatPolarSeries.Clear;
    for i:=0 to MAVsatCount do begin
      if (sats.sat_used[i]=1) and (sats.sats_visible>0) then
        IndicatorColor:=clSatUsed
      else
        IndicatorColor:=clSatVisible;
      azi:=SatAzimuthToDeg(sats.sat_azimuth[i]);
      ele:=SatElevationToDeg(sats.sat_elevation[i]);
      SatPolarSource.Add(azi, ele, 'ID'+IntToStr(sats.sat_prn[i]), IndicatorColor);
    end;
    PreparePolarAxes(SatPolar, 90);  {Sat elevation 0: right on top of receiver, 90: on the horizon}
  finally
    SatPolar.EnableRedrawing;
    SatPolar.Repaint;
  end;
end;

procedure TForm1.CreateSatSNRBarChart(const sats: TGPSdata);
var
  i, NumSatsInUse: integer;
  IndicatorColor: TColor;

begin
  ChartSatSNR.DisableRedrawing;
  BarSatSNR.Clear;
  NumSatsInUse:=0;
  ChartSatSNR.Title.Visible:=true;
  try
    for i:=0 to MAVsatCount do begin
      if (sats.sat_used[i]=1) and (sats.sats_visible>0) then begin
        IndicatorColor:=clSatUsed;
        inc(NumSatsInUse);
      end else
        IndicatorColor:=clSatVisible;
      SatSNRBarSource.Add(i, sats.sat_snr[i], 'ID'+IntToStr(sats.sat_prn[i]), IndicatorColor);
    end;
    ChartSatSNR.Title.Text[0]:=IntToStr(sats.sats_visible)+tab1+rsVisible+tab2+
                               IntToStr(NumSatsInUse)+tab1+rsInUse;
  finally
    ChartSatSNR.EnableRedrawing;
    ChartSatSNR.Repaint;
  end;
end;

procedure TForm1.WriteGridGPSdataValues(const sats: TGPSdata);
begin
  if sats.boottime>0 then begin
    gridGPSdata.BeginUpdate;
    try
      gridGPSdata.Cells[1, 0]:=FormatDateTime(timezzz, sats.boottime);
      if sats.timeUTC>1 then begin;
        lblTime.Caption:=FormatDateTime(timefull, sats.timeUTC);
        gridGPSdata.Cells[1, 1]:=lblTime.Caption;
      end;

      gridGPSdata.Cells[1, 2]:=FormatDateTime(timefull, sats.yuneectime);
      gridGPSdata.Cells[1, 3]:=FormatCoordinates(sats.lat);
      gridGPSdata.Cells[1, 4]:=FormatCoordinates(sats.lon);
      gridGPSdata.Cells[1, 5]:=FormatAltitude(sats.altMSL);
      gridGPSdata.Cells[1, 6]:=IntToStr(sats.sats_visible);
      gridGPSdata.Cells[1, 7]:=FormatDOP(sats.eph);
      gridGPSdata.Cells[1, 8]:=FormatDOP(sats.epv);
      gridGPSdata.Cells[1, 9]:=FormatSpeed(sats.vel);
      gridGPSdata.Cells[1, 10]:=FormatHdg(sats.cog);
      gridGPSdata.Cells[1, 11]:=FixTypeToStr(sats.fix_type);
      gridGPSdata.Cells[1, 12]:=FormatAltitude(sats.alt_ellipsoid);
      gridGPSdata.Cells[1, 13]:=FormatAcc(sats.h_acc);
      gridGPSdata.Cells[1, 14]:=FormatAcc(sats.v_acc);
      gridGPSdata.Cells[1, 15]:=FormatVelAcc(sats.vel_acc);
      gridGPSdata.Cells[1, 16]:=FormatHdgAcc(sats.hdg_acc);
      gridGPSdata.Cells[1, 17]:=IntToStr(sats.yaw);
    finally
      gridGPSdata.EndUpdate;
    end;
  end;
end;

procedure TForm1.WriteDataCSVlist(const sats: TGPSdata);
var
  i: integer;
  CSVrow: string;

begin
  if sats.boottime>0 then begin
    CSVrow:=gridGPSdata.Cells[1, 0];
    for i:=1 to gridGPSdata.RowCount-1 do                 {Add data from GPS_RAW_INT}
      CSVrow:=CSVrow+sep+gridGPSdata.Cells[1, i];
// Add additional values at the end (possibly for checking something
// + sep + value
    csvlist.Add(CSVrow);
  end;
end;


//////////////////////// Polar coordinates diagram /////////////////////////////

{https://www.lazarusforum.de/viewtopic.php?p=66139#p66139
 wp_xyz

PreparePolarAxes rufst du auf, nachdem deine Daten geladen sind und du weißt,
wie groß der maximale Radiuswert ist.
Den brauchst du als Parameter AMax in dieser Prozedur (hier fix auf 90°).
Damit werden die x- und y-Achsen auf gleichen Wertebereich eingestellt und
insgesamt komplett ausgeblendet.

DrawPolarAxes wird im OnAfterDrawBackwall-Ereignis des Charts aufgerufen.
Zu diesem Zeitpunkt sind die Daten noch nicht ausgegeben - es würde sich auch
OnAfterPaint anbieten, aber damit würden die Achsenkreise über die
Datenkurven gezeichnet und in der hier gezeigten Implementierung komplett
übermalt, weil das Hintergrundrechteck mit eingefärbt wird.
DrawPolarAxes erhält als Parameter wieder den maximalen Radius und den
Abstand der Kreise. Die komplette Zeichenausgabe ist etwas ungewohnt
("Chart.Drawer"), weil TAChart eine Zwischenschicht für die Ausgabe
eingeführt hat, so dass man verschiedene Ausgabe"geräte"
(BGRABitmap, Vektorformate, Drucker) mit demselben Code ansprechen kann.
}

procedure TForm1.PreparePolarAxes(AChart: TChart; AMax: Double);
var
  ex: TDoubleRect;

begin
  ex.a.x := -AMax;
  ex.a.y := -AMax;
  ex.b.x :=  AMax;
  ex.b.y :=  AMax;
  with AChart do begin
    Extent.FixTo(ex);
    Proportional := true;
    Frame.Visible := false;
    with LeftAxis do begin
      AxisPen.Visible := false;
      Grid.Visible := false;
      PositionUnits := cuGraph;
      Marks.Visible := false;
    end;
    with BottomAxis do begin
      AxisPen.Visible := false;
      Grid.Visible := false;
      PositionUnits := cuGraph;
      Marks.Visible := false;
    end;
  end;
end;

procedure TForm1.DrawPolarAxes(AChart: TChart; AMax, ADelta: Double; MeasurementUnit: string);
var
  xRadius, theta: Double;
  P1, P2: TPoint;
  i, h, w: Integer;
  AxisLabels: string;

begin
  with AChart do begin
// Background
    Drawer.SetBrushParams(bsSolid, Color);
    Drawer.FillRect(0, 0, Width, Height);

// Radial lines for direction
    Drawer.SetBrushParams(bsClear, clNone);
    Drawer.SetPenParams(psDot, clGray);
    for i:=0 to 5 do begin
      theta := i * pi/6;
      P1 := GraphToImage(DoublePoint(AMax*sin(theta), AMax*cos(theta)));
      P2 := GraphToImage(DoublePoint(-AMax*sin(theta), -AMax*cos(theta)));
      Drawer.MoveTo(P1);
      Drawer.Lineto(P2);
    end;

// Circles
    xRadius := ADelta;
    while xRadius <= AMax do begin
      P1 := GraphToImage(DoublePoint(-xRadius, -xRadius));
      P2 := GraphToImage(DoublePoint(+xRadius, +xRadius));
      Drawer.SetPenParams(psDot, clGray);
      Drawer.SetBrushParams(bsClear, clNone);
      Drawer.Ellipse(P1.x, P1.y, P2.x, P2.y);
      xRadius := xRadius + ADelta;
    end;

// Axis labels
    Drawer.Font := BottomAxis.Marks.LabelFont;
    h := Drawer.TextExtent('0').y;
    xRadius := 0;
    while xRadius <= AMax do begin
      AxisLabels := FloatToStr(xRadius)+MeasurementUnit;
      w := Drawer.TextExtent(AxisLabels).x;
      P1 := GraphToImage(DoublePoint(0, xRadius));
      Drawer.TextOut.Pos(P1.X - w div 2, P1.y - h div 2).Text(AxisLabels).Done;
      xRadius := xRadius + ADelta;
    end;
  end;
end;

{DrawPolarAxes is called in the OnAfterDrawBackwall event of the chart.
 OnAfterPaint would also be an option, but this would draw the axis circles over the
 data curves because the background rectangle is also colored.}

procedure TForm1.SatPolarAfterDrawBackWall(ASender: TChart; ACanvas: TCanvas;
  const ARect: TRect);
begin
  DrawPolarAxes(ASender, 90, 30, '°');
end;

end.

