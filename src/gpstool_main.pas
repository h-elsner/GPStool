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
  ExtCtrls, Buttons, ComCtrls, Grids, XMLPropStorage, Menus,
  FileUtil, DateUtils, TAGraph, TATypes, TASeries, TAChartUtils, TAGeometry,
  TARadialSeries, TASources, TAIntervalSources, mav_gps;

const
  clSatUsed=clLime;
  clSatVisible=clMoneyGreen;
  clPolarLabel=clSkyBlue;
  clDataSerie1=clRed;
  clTabs=$00F7F7F7;
  PolarSatSize=8;
  tab1=' ';
  tab2='  ';
  sep=';';                                               {CSV separator}
  digits=['0'..'9', '.', ',', ':'];

  meinname='Helmut Elsner';
  appVersion='V0.2 build 2024-11-14';

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
    cbFast: TCheckBox;
    ChartData: TChart;
    ChartDataLineSeries1: TLineSeries;
    DateTimeIntervalDataSource: TDateTimeIntervalChartSource;
    lblTime: TLabel;
    MainMenu1: TMainMenu;
    mnClose: TMenuItem;
    barStream: TProgressBar;
    PanelChart: TPanel;
    Separator3: TMenuItem;
    mnMainSaveCSV: TMenuItem;
    Separator2: TMenuItem;
    mnMainSaveChart: TMenuItem;
    mnMainSaveGrid: TMenuItem;
    Separator1: TMenuItem;
    mnOpen: TMenuItem;
    mnAbout: TMenuItem;
    mnInfo: TMenuItem;
    mnFile: TMenuItem;
    mnSaveSatSNR: TMenuItem;
    mnSaveCSV: TMenuItem;
    mnSaveGrid: TMenuItem;
    pmnSatSNR: TPopupMenu;
    pmnData: TPopupMenu;
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
    SaveDialog: TSaveDialog;
    gridGPSdata: TStringGrid;
    barSleepTime: TTrackBar;
    Splitter1: TSplitter;
    StatusBar: TStatusBar;
    XMLPropStorage1: TXMLPropStorage;
    procedure actAboutExecute(Sender: TObject);
    procedure actCloseExecute(Sender: TObject);
    procedure actContinueExecute(Sender: TObject);
    procedure actEndExecute(Sender: TObject);
    procedure actHaltExecute(Sender: TObject);
    procedure actLoadFileExecute(Sender: TObject);
    procedure actSaveChartExecute(Sender: TObject);
    procedure actSaveCSVExecute(Sender: TObject);
    procedure actSaveGridExecute(Sender: TObject);
    procedure cbSaveCSVChange(Sender: TObject);
    procedure FormActivate(Sender: TObject);
    procedure FormClose(Sender: TObject; var CloseAction: TCloseAction);
    procedure FormDropFiles(Sender: TObject; const FileNames: array of string);
    procedure gridGPSdataHeaderClick(Sender: TObject; IsColumn: Boolean;
      Index: Integer);
    procedure plDataResize(Sender: TObject);
    procedure SatPolarAfterDrawBackWall(ASender: TChart; ACanvas: TCanvas;
      const ARect: TRect);
    procedure FormCreate(Sender: TObject);
  private
    procedure DoLoadFile(aFileName: string);
    procedure CreateSatPolarDiagram(const sats: TGPSdata);
    procedure CreateSatSNRBarChart(const sats: TGPSdata);
    procedure WriteGridGPSdataValues(const sats: TGPSdata);
    procedure PrepareSatSNRBarChart;
    procedure PrepareSatPolarDiagram;
    procedure PrepareChartData(const dataindex: integer);
    procedure CreateChartDataLineSeries(const dataindex: integer);
    procedure ClearPositioningData;
    procedure WriteGridGPSdataHeader;
    procedure ResetGlobalVariables;
    procedure WriteHeaderCSVlist;
    procedure WriteGPSDataCSVlist(const sats: TGPSdata);
    procedure InfoMessageToStatusbar(info: string);
    procedure SetForValidFile(aFileName: string);
    procedure SetForInvalidFile(ErrorMessage: string);
  public
    function  DecodeOneSensorMessage(const msg: TMAVmessage; offset: byte; var data: TGPSdata): boolean;
    procedure ProcessMAVFile(const UsedMagic: byte);
    procedure PreparePolarAxes(AChart: TChart; AMax: Double);
    procedure DrawPolarAxes(AChart: TChart; AMax, ADelta: Double; MeasurementUnit: string);
  end;

var
  Form1: TForm1;
  stopping: boolean;
  NumMsgUsed, NumMsgTotal: integer;
  csvlist: TStringList;
  inputstream: TMemoryStream;

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
  cbFast.Caption:=capFast;
  cbFast.Hint:=hntFast;

  actSaveCSV.Caption:=capSaveCSV;
  actSaveCSV.Hint:=hntSaveCSV;
  actClose.Caption:=capClose;
  actClose.Hint:=hntClose;
  actLoadFile.Caption:=capLoadFile;
  actLoadFile.Hint:=hntLoadFile;
  actSaveGrid.Caption:=capSaveGrid;
  actSaveGrid.Hint:=hntSaveGrid;
  actSaveChart.Caption:=capSaveChart;
  actSaveChart.Hint:=hntSaveChart;
  actAbout.Caption:=capAbout+tab1+capApplication;

  mnInfo.Caption:=capMainMenuInfo;
  mnFile.Caption:=capMainMenuFile;
  StatusBar.Hint:=hntStatusbar;
  OpenDialog.Title:=capOpenDialog;
  lblTime.Caption:=capTime;
  lblTime.Hint:=hntTime;

  WriteGridGPSdataHeader;
  PrepareSatSNRBarChart;
  BarSatSNR.Clear;
  PrepareSatPolarDiagram;
  PreparePolarAxes(SatPolar, 90);
  SatPolarSeries.Clear;
  barSleepTime.Hint:=hntSleepTime;

  barSleepTime.Position:=100;
  cbSaveCSV.Checked:=false;
  gridGPSdata.AlternateColor:=clTabs;

  stopping:=false;
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

procedure TForm1.FormDropFiles(Sender: TObject; const FileNames: array of string);
begin
  DoLoadFile(FileNames[0]);
end;

procedure TForm1.gridGPSdataHeaderClick(Sender: TObject; IsColumn: Boolean;
  Index: Integer);
begin
  if index>2 then begin
    if (index=3) or (index=4) then begin                 {Coordinates}
      // distance to do
    end else begin
      PrepareChartData(index);
      CreateChartDataLineSeries(index);
    end;
  end;
end;

procedure TForm1.plDataResize(Sender: TObject);
begin
  gbData.Height:=(plData.Height div 2)-24;
  if gbData.Height>=440 then
    gbData.Height:=440;                                  {Limit seizing of data table}
  gbSatSNR.Top:=gbData.Top+gbData.Height+12;
  gbSatSNR.Height:=plData.Height-gbData.Height-54;
end;

procedure TForm1.cbSaveCSVChange(Sender: TObject);
begin
  actSaveCSV.Enabled:=not cbSaveCSV.Checked;
end;

procedure TForm1.FormActivate(Sender: TObject);
begin
  actSaveCSV.Enabled:=not cbSaveCSV.Checked;
end;

procedure TForm1.ResetGlobalVariables;
begin
  NumMsgUsed:=0;
  NumMsgTotal:=0;
  inputstream.Position:=0;
  inputstream.Size:=0;
  barStream.Position:=0;
  stopping:=false;
end;

procedure TForm1.WriteGridGPSdataHeader;
begin
  gridGPSdata.BeginUpdate;
  try
    gridGPSdata.RowCount:=23;
    gridGPSdata.Cells[0, 0]:='Time since boot';
    gridGPSdata.Cells[0, 1]:='SYSTEM TIME UTC';
    gridGPSdata.Cells[0, 2]:='Time UTC per Message';
    gridGPSdata.Cells[0, 3]:='Latitude (WGS84 EGM96 ellipsoid)';
    gridGPSdata.Cells[0, 4]:='Longitude (WGS84 EGM96 ellipsoid)';
    gridGPSdata.Cells[0, 5]:='Altitude (MSL)';
    gridGPSdata.Cells[0, 6]:='Altitude (above WGS84 EGM96 ellipsoid)';
    gridGPSdata.Cells[0, 7]:='Altitude above home';
    gridGPSdata.Cells[0, 8]:='GPS ground speed';
    gridGPSdata.Cells[0, 9]:='Ground X Speed';
    gridGPSdata.Cells[0, 10]:='Ground Y Speed';
    gridGPSdata.Cells[0, 11]:='Ground Z Speed';
    gridGPSdata.Cells[0, 12]:='Course over ground';
    gridGPSdata.Cells[0, 13]:='Heading';
    gridGPSdata.Cells[0, 14]:='Yaw in earth frame from north';
    gridGPSdata.Cells[0, 15]:='Number of satellites visible';
    gridGPSdata.Cells[0, 16]:='GPS fix type';
    gridGPSdata.Cells[0, 17]:='GPS HDOP';
    gridGPSdata.Cells[0, 18]:='GPS VDOP';
    gridGPSdata.Cells[0, 19]:='Position uncertainty';
    gridGPSdata.Cells[0, 20]:='Altitude uncertainty';
    gridGPSdata.Cells[0, 21]:='Speed uncertainty';
    gridGPSdata.Cells[0, 22]:='Heading / track uncertainty';
  finally
    gridGPSdata.EndUpdate;
  end;
end;

function IndexToMeasurementUnit(index: integer): shortstring;
begin
  result:='';
  case index of
    5..7: result:=' [m]';
    8..11, 21: result:=' [m/s]';
    19, 20: result:=' [cm]';
    12..14, 22: result:=' [°]';
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

procedure TForm1.SetForValidFile(aFileName: string);
begin
  InfoMessageToStatusbar(aFileName);
  Caption:=capApplication+tab2+ExtractFileName(aFileName);
end;

procedure TForm1.SetForInvalidFile(ErrorMessage: string);
begin
  Caption:=capApplication;
  StatusBar.Panels[0].Text:='0';
  StatusBar.Panels[1].Text:='0';
  InfoMessageToStatusbar(ErrorMessage);
end;

procedure TForm1.DoLoadFile(aFileName: string);
var
  PureFileName: string;

begin
  btnLoad.Tag:=0;                                       {Invalid file; Tag misused for file type}
  ResetGlobalVariables;
  BarSatSNR.Clear;
  SatPolarSeries.Clear;
  ClearPositioningData;

  if FileSize(OpenDialog.FileName)<500 then begin
    SetForInvalidFile(errSmallFile);
    exit;
  end;

  PureFileName:=ExtractFileName(OpenDialog.FileName);
  if (pos('.tlog', LowerCase(PureFileName))>1) or
     (pos('.log', LowerCase(PureFileName))>1)then begin
    SetForValidFile(aFileName);
    btnLoad.Tag:=MagicFD;
  end else begin
    if pos('Sensor_', PureFileName)=1 then begin
      SetForValidFile(aFileName);
      btnLoad.Tag:=MagicBC;
    end else begin
      SetForInvalidFile(errInvalidFile);
    end;
  end;

  inputstream.LoadFromFile(OpenDialog.FileName);
  barStream.Max:=inputstream.Size-1;
  ProcessMAVfile(btnLoad.Tag);

  StatusBar.Panels[0].Text:=IntToStr(NumMsgTotal);
  StatusBar.Panels[1].Text:=IntToStr(NumMsgUsed);
  if cbSaveCSV.Checked then
    actSaveCSVExecute(self);
end;

/////////////////////////////// Actions ////////////////////////////////////////

procedure TForm1.actCloseExecute(Sender: TObject);
begin
  stopping:=true;
  Close;
end;

procedure TForm1.actAboutExecute(Sender: TObject);
begin
  MessageDlg(capApplication+sLineBreak+AppVersion+
             sLineBreak+sLineBreak+meinname,
             mtInformation,[mbOK],0);
end;

procedure TForm1.InfoMessageToStatusbar(info: string);
begin
  StatusBar.Panels[2].Text:=info;
end;

procedure TForm1.actContinueExecute(Sender: TObject);
begin
  stopping:=false;
  if btnLoad.Tag>0 then begin                            {Means, there was is a valid file type in progress}
    ProcessMAVfile(btnLoad.Tag);
  end;
end;

procedure TForm1.actEndExecute(Sender: TObject);
begin
  stopping:=true;
  inputstream.Position:=0;
  barStream.Position:=0;
  StatusBar.Panels[0].Text:=IntToStr(NumMsgTotal);
  StatusBar.Panels[1].Text:=IntToStr(NumMsgUsed);
  if cbSaveCSV.Checked then
    actSaveCSVExecute(self);
end;

procedure TForm1.actHaltExecute(Sender: TObject);
begin
  stopping:=true;
  StatusBar.Panels[0].Text:=IntToStr(NumMsgTotal);
  StatusBar.Panels[1].Text:=IntToStr(NumMsgUsed);
end;

procedure TForm1.actLoadFileExecute(Sender: TObject);
begin
  if OpenDialog.Execute then begin
    DoLoadFile(OpenDialog.FileName);
  end;
end;

procedure TForm1.actSaveChartExecute(Sender: TObject);
begin
  SaveDialog.Title:=hntSaveGrid;
  SaveDialog.FileName:=ChangeFileExt(OpenDialog.FileName, '')+'_chart.png';
  if SaveDialog.Execute then begin
    ChartSatSNR.SaveToBitmapFile(SaveDialog.FileName);
    InfoMessageToStatusbar(ExtractFileName(SaveDialog.FileName)+tab1+rsSaved);
  end;
end;

procedure TForm1.actSaveCSVExecute(Sender: TObject);
var
  ChangedFileName: string;

begin
  if (csvlist.Count>1) then begin
    ChangedFileName:=ChangeFileExt(OpenDialog.FileName, '')+'_GPStool.csv';
    if cbSaveCSV.Checked then begin
      csvlist.SaveToFile(ChangedFileName);
      InfoMessageToStatusbar(ExtractFileName(ChangedFileName)+tab1+rsSaved);
    end else begin
      SaveDialog.Title:=capSaveCSV;
      SaveDialog.FileName:=ChangedFileName;
      if SaveDialog.Execute then begin
        csvlist.SaveToFile(SaveDialog.FileName);
        InfoMessageToStatusbar(ExtractFileName(SaveDialog.FileName)+tab1+rsSaved);
      end;
    end;
  end else
    InfoMessageToStatusbar(errEmptyFile);
end;

procedure TForm1.actSaveGridExecute(Sender: TObject);
begin
  SaveDialog.Title:=hntSaveGrid;
  SaveDialog.FileName:=ChangeFileExt(OpenDialog.FileName, '')+'_table.csv';
  if SaveDialog.Execute then begin
    gridGPSdata.SaveToCSVFile(SaveDialog.FileName);
    InfoMessageToStatusbar(ExtractFileName(SaveDialog.FileName)+tab1+rsSaved);
  end;
end;

///////////////////////////// Processing ///////////////////////////////////////

function TForm1.DecodeOneSensorMessage(const msg: TMAVmessage; offset: byte; var data: TGPSdata): boolean;

  procedure SYS_TIME;
  begin
    result:=true;
    data.timeUTC:=UnixToDateTime(MavGetUInt64(msg, offset) div 1000000);  {us --> s};
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

  procedure GLOBAL_POSITION_INT;
  begin
    result:=true;
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

begin
  result:=false;
  data.sats_visible:=max8;
  if btnLoad.Tag=MagicFD then                            {Trailing time stamp for all FD messages}
    data.yuneectime:=UnixToDateTime(YuneecTimeStampInSeconds(msg));
  case msg.msgid of
     2: SYS_TIME;
    24: GPS_RAW_INT;
    25: GPS_STATUS;
    33: GLOBAL_POSITION_INT;
  end;
end;

procedure TForm1.ProcessMAVFile(const UsedMagic: byte);
var
  b, offset: byte;
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
        offset:=LengthFixPartFD;
      end;

      MagicBC: begin
        // Read all message bytes from stream: Fix part + Payload + CRC
        inputstream.ReadBuffer(msg.msgbytes, msg.msglength+LengthFixPartBC+2);
        msg.sysid:=msg.msgbytes[3];
        msg.targetid:=msg.msgbytes[4];
        msg.msgid:=msg.msgbytes[5];
        offset:=LengthFixPartBC;
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
  ClearMAVmessage(msg);
  GPSdata_SetDefaultValues(data);
  if inputstream.Position=0 then begin
    csvlist.Clear;
    WriteHeaderCSVlist;
  end;
  while inputstream.Position<inputstream.Size-100 do begin
    repeat                                               {Try to find next UsedMagic in stream}
      b:=inputstream.ReadByte;
    until (b=UsedMagic) or
          (inputstream.Position>=inputstream.Size-LengthFixPartFD-12);

    msg.msgbytes[0]:=b;
    msg.msglength:=inputstream.ReadByte;                 {Lenght payload is without CRC}
    msg.msgbytes[1]:=msg.msglength;
    inputstream.Position:=inputstream.Position-2;

    SetFixPartValuesForMsgType;
    inc(NumMsgTotal);
    if CheckMessage then begin
      if DecodeOneSensorMessage(msg, offset, data) then begin
        inc(NumMsgUsed);
        if data.boottime>0 then begin
          if (data.sats_visible<>max8) and (data.sats_visible<>0) then begin
            if (not cbFast.Checked) and (data.fix_type>0) then begin
              CreateSatPolarDiagram(data);
              CreateSatSNRBarChart(data);
            end;
            WriteGridGPSdataValues(data);
            WriteGPSDataCSVlist(data);
          end;
        end;
        barStream.Position:=inputstream.Position;
        if not cbFast.Checked then
          Application.ProcessMessages;
        if stopping then begin
          exit;
        end;
      end;
    end;
  end;
  if  cbFast.Checked then begin
    CreateSatPolarDiagram(data);
    CreateSatSNRBarChart(data);
  end;
  inputstream.Position:=0;
  barStream.Position:=barStream.Max;
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
      if (sats.sat_used[i]=1) then
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
  i, NumSatsInUse, NumSatsVisible: integer;
  IndicatorColor: TColor;

begin
  ChartSatSNR.DisableRedrawing;
  BarSatSNR.Clear;
  NumSatsInUse:=0;
  ChartSatSNR.Title.Visible:=true;
  try
    for i:=0 to MAVsatCount do begin
      if (sats.sat_used[i]=1) then begin
        IndicatorColor:=clSatUsed;
        inc(NumSatsInUse);
      end else
        IndicatorColor:=clSatVisible;
      SatSNRBarSource.Add(i, sats.sat_snr[i], 'ID'+IntToStr(sats.sat_prn[i]), IndicatorColor);
    end;
    NumSatsVisible:=sats.sats_visible;
    if NumSatsVisible=max8 then
      NumSatsVisible:=0;
    ChartSatSNR.Title.Text[0]:=IntToStr(NumSatsVisible)+tab1+rsVisible+tab2+
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

      if btnLoad.Tag=MagicFD then
        gridGPSdata.Cells[1, 2]:=FormatDateTime(timefull, sats.yuneectime);
      if sats.fix_type>0 then begin
//  From GPS_RAW_INT (24)
        gridGPSdata.Cells[1, 3]:=FormatCoordinates(sats.lat);
        gridGPSdata.Cells[1, 4]:=FormatCoordinates(sats.lon);
        gridGPSdata.Cells[1, 5]:=FormatAltitude(sats.altMSL);
        gridGPSdata.Cells[1, 6]:=FormatAltitude(sats.alt_ellipsoid);
        gridGPSdata.Cells[1, 8]:=FormatSpeed(sats.vel);
        gridGPSdata.Cells[1, 12]:=FormatHdg(sats.cog);
        gridGPSdata.Cells[1, 14]:=IntToStr(sats.yaw);
        gridGPSdata.Cells[1, 17]:=FormatDOP(sats.eph);
        gridGPSdata.Cells[1, 18]:=FormatDOP(sats.epv);
        gridGPSdata.Cells[1, 19]:=FormatAcc(sats.h_acc);
        gridGPSdata.Cells[1, 20]:=FormatAcc(sats.v_acc);
        gridGPSdata.Cells[1, 21]:=FormatVelAcc(sats.vel_acc);
        gridGPSdata.Cells[1, 22]:=FormatHdgAcc(sats.hdg_acc);
      end;

//  From GLOBAL_POSITION_INT (33)
      gridGPSdata.Cells[1, 7]:=FormatAltitude(sats.alt_rel);
      gridGPSdata.Cells[1, 9]:=FormatXYZSpeed(sats.vx);
      gridGPSdata.Cells[1, 10]:=FormatXYZSpeed(sats.vy);
      gridGPSdata.Cells[1, 11]:=FormatXYZSpeed(sats.vz);
      gridGPSdata.Cells[1, 13]:=FormatHdg(sats.hdg);

//  From GPS_RAW_INT (24)
      gridGPSdata.Cells[1, 15]:=IntToStr(sats.sats_visible);
      gridGPSdata.Cells[1, 16]:=FixTypeToStr(sats.fix_type);
    finally
      gridGPSdata.EndUpdate;
    end;
  end;
end;

procedure TForm1.WriteGPSDataCSVlist(const sats: TGPSdata);
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

function GetFloatFromTable(const ValueFromTable: string): single;
var
  s: string;
  i: integer;

begin
  s:=ValueFromTable;
  for i:=length(s) downto 1 do
    if not (s[i] in digits) then
      delete(s, i, 1);
  result:=StrToFloatDef(s, 0);
end;

procedure TForm1.CreateChartDataLineSeries(const dataindex: integer);
var
  i: integer;
  value: single;
  timestamp: TDateTime;

begin
  if csvlist.Count>10 then begin
    ChartData.DisableRedrawing;
    ChartDataLineSeries1.Clear;
    try
      for i:=1 to csvlist.Count-1 do begin
        value:=GetFloatFromTable(csvlist[i].Split([sep])[dataindex]);
        timestamp:=ScanDateTime(timezzz, csvlist[i].Split([sep])[0]);
        ChartDataLineSeries1.AddXY(timestamp, value);
      end;
    finally
      ChartData.EnableRedrawing;
    end;
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

procedure TForm1.PrepareSatPolarDiagram;                 {My settings for the polar diagram}
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

procedure TForm1.PrepareSatSNRBarChart;                  {My settings for the SNR chart}
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

procedure TForm1.PrepareChartData(const dataindex: integer);
begin
  with ChartData do begin
    Title.Visible:=false;
    LeftAxis.Title.Caption:=gridGPSdata.Cells[0, dataindex]+IndexToMeasurementUnit(dataindex);
    LeftAxis.Title.Visible:=true;
    BottomAxis.Marks.Source:=DateTimeIntervalDataSource;
    BottomAxis.Marks.Style:=smsLabel;
    BottomAxis.Marks.Format:='%2:s';
  end;
  DateTimeIntervalDataSource.DateTimeFormat:='nn:ss';
  DateTimeIntervalDataSource.Params.NiceSteps:='5|10';   {x-axis labels wider steps}
  ChartDataLineSeries1.SeriesColor:=clDataSerie1;
end;


end.

