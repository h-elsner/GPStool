(*

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
  ExtCtrls, Buttons, ComCtrls, Grids, XMLPropStorage, Menus, math,
  FileUtil, DateUtils, TAGraph, TATypes, TASeries, TAChartUtils, TAGeometry,
  TARadialSeries, TASources, TAIntervalSources, mav_def, mav_msg, Types;

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
//  Limit the message ID number for validation regarding the current use case of the analysis
  MaxMsgID=266;

  meinname='Helmut Elsner';
  appVersion='V0.3 build 2024-11-16';

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
    ChartSatSNR: TChart;
    DateTimeIntervalDataSource: TDateTimeIntervalChartSource;
    gbData: TGroupBox;
    gbPolar: TGroupBox;
    gbSatSNR: TGroupBox;
    gridGPSdata: TStringGrid;
    lblTime: TLabel;
    MainMenu1: TMainMenu;
    mnClose: TMenuItem;
    barStream: TProgressBar;
    PageControl: TPageControl;
    PanelChart: TPanel;
    plData: TPanel;
    SatPolar: TChart;
    SatPolarSeries: TPolarSeries;
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
    ImageList1: TImageList;
    SatPolarSource: TListChartSource;
    SatSNRBarSource: TListChartSource;
    OpenDialog: TOpenDialog;
    plUser: TPanel;
    btnContinue: TSpeedButton;
    btnHalt: TSpeedButton;
    btnEnd: TSpeedButton;
    SaveDialog: TSaveDialog;
    barSleepTime: TTrackBar;
    Splitter1: TSplitter;
    StatusBar: TStatusBar;
    StatusTextList: TMemo;
    gridCSVdata: TStringGrid;
    tsEKFtool: TTabSheet;
    tsGPSTool: TTabSheet;
    tsData: TTabSheet;
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
    procedure gridGPSdataPrepareCanvas(Sender: TObject; aCol, aRow: Integer;
      aState: TGridDrawState);
    procedure PageControlChange(Sender: TObject);
    procedure plDataResize(Sender: TObject);
    procedure SatPolarAfterDrawBackWall(ASender: TChart; ACanvas: TCanvas;
      const ARect: TRect);
    procedure FormCreate(Sender: TObject);
    procedure StatusTextListMouseWheelDown(Sender: TObject; Shift: TShiftState;
      MousePos: TPoint; var Handled: Boolean);
    procedure StatusTextListMouseWheelUp(Sender: TObject; Shift: TShiftState;
      MousePos: TPoint; var Handled: Boolean);
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
    procedure ProcessMAVFile(const FormatType: byte);
    procedure PreparePolarAxes(AChart: TChart; AMax: Double);
    procedure DrawPolarAxes(AChart: TChart; AMax, ADelta: Double; MeasurementUnit: string);
  end;

var
  Form1: TForm1;
  stopping: boolean;
  NumMsgUsed, NumMsgTotal: integer;
  MsgFormatType: byte;
  csvlist: TStringList;
  inputstream: TMemoryStream;
  gpsPresent, SensorsHealthy: boolean;

{$I GPStool_en.inc}

implementation

{$R *.lfm}

procedure TForm1.FormCreate(Sender: TObject);
begin
  {https://forum.lazarus.freepascal.org/index.php?topic=34510.0}
  {$IFDEF WINDOWS}
      Application.MainFormOnTaskBar := True;
  {$ENDIF}

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
  PrepareSatPolarDiagram;
  PreparePolarAxes(SatPolar, 90);
  PrepareChartData(0);
  BarSatSNR.Clear;
  SatPolarSeries.Clear;
  ChartDataLineSeries1.Clear;
  StatusTextList.Clear;
  barSleepTime.Hint:=hntSleepTime;

  barSleepTime.Position:=100;
  cbSaveCSV.Checked:=false;
  gridGPSdata.AlternateColor:=clTabs;

  stopping:=false;
  gpsPresent:=true;
  SensorsHealthy:=true;
  NumMsgUsed:=0;
  NumMsgTotal:=0;
  MsgFormatType:=0;

  csvlist:=TStringList.Create;
  inputstream:=TMemoryStream.Create;
  inputstream.Position:=0;
end;

procedure TForm1.StatusTextListMouseWheelDown(Sender: TObject;
  Shift: TShiftState; MousePos: TPoint; var Handled: Boolean);
begin
  if ssCtrl in Shift then
    StatusTextList.Font.Size:=StatusTextList.Font.Size-1;
end;

procedure TForm1.StatusTextListMouseWheelUp(Sender: TObject;
  Shift: TShiftState; MousePos: TPoint; var Handled: Boolean);
begin
  if ssCtrl in Shift then
    StatusTextList.Font.Size:=StatusTextList.Font.Size+1;
end;

procedure TForm1.FormClose(Sender: TObject; var CloseAction: TCloseAction);
begin
  Screen.Cursor:=crDefault;
  if assigned(csvlist) then
    csvlist.free;
  if assigned(inputstream) then
    inputstream.Free;
end;

procedure TForm1.FormDropFiles(Sender: TObject; const FileNames: array of string);
begin
  OpenDialog.FileName:=FileNames[0];
  Application.BringToFront;
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

procedure TForm1.gridGPSdataPrepareCanvas(Sender: TObject; aCol, aRow: Integer;
  aState: TGridDrawState);
var
  gridtextstyle: TTextStyle;

begin
  if aCol=1 then begin
    gridtextstyle:=gridGPSdata.Canvas.TextStyle;
    gridtextstyle.Alignment:=taCenter;
    gridGPSdata.Canvas.TextStyle:=gridtextstyle;
  end;
end;

procedure TForm1.PageControlChange(Sender: TObject);
var
  tmpStream: TStringStream;

begin
  if PageControl.ActivePage=tsData then begin
    if csvlist.Count>1 then begin
      gridCSVdata.RowCount:=1;
      Application.ProcessMessages;
      tmpStream:=TStringStream.Create;
      try
        csvlist.SaveToStream(tmpStream);
        tmpStream.Position:=0;
        gridCSVdata.LoadFromCSVStream(tmpStream, sep);
      finally
        tmpStream.Free;
      end;
    end;
  end;
end;

procedure TForm1.plDataResize(Sender: TObject);
const
  listhight=640;

begin
  gbData.Height:=(plData.Height div 2)-12;
  if gbData.Height>=listhight then
    gbData.Height:=listhight;                            {Limit seizing of data table}
  gbSatSNR.Top:=gbData.Top+gbData.Height+8;
  gbSatSNR.Height:=plData.Height-gbData.Height-40;
end;

procedure TForm1.cbSaveCSVChange(Sender: TObject);
begin
  actSaveCSV.Enabled:=not cbSaveCSV.Checked;
end;

procedure TForm1.FormActivate(Sender: TObject);
begin
  actSaveCSV.Enabled:=not cbSaveCSV.Checked;
  if PageControl.ActivePage=tsData then
    PageControl.ActivePage:=tsGPStool;
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
    gridGPSdata.RowCount:=25;
    gridGPSdata.Cells[0, 0]:='Time since boot';                    {2, 24, 33}
    gridGPSdata.Cells[0, 1]:='SYSTEM TIME UTC';                    {2}
    gridGPSdata.Cells[0, 2]:='Time UTC per Message';               {all $FD messages}
    gridGPSdata.Cells[0, 3]:='Latitude (WGS84 EGM96 ellipsoid)';   {24, 33}
    gridGPSdata.Cells[0, 4]:='Longitude (WGS84 EGM96 ellipsoid)';  {24, 33}
    gridGPSdata.Cells[0, 5]:='Altitude (MSL)';                     {24, 33}
    gridGPSdata.Cells[0, 6]:='Altitude (above WGS84 EGM96 ellipsoid)'; {24}
    gridGPSdata.Cells[0, 7]:='Altitude above home';                {33}
    gridGPSdata.Cells[0, 8]:='GPS ground speed';                   {33}
    gridGPSdata.Cells[0, 9]:='Ground X Speed';                     {33}
    gridGPSdata.Cells[0, 10]:='Ground Y Speed';                    {33}
    gridGPSdata.Cells[0, 11]:='Ground Z Speed';                    {33}
    gridGPSdata.Cells[0, 12]:='Course over ground';                {24}
    gridGPSdata.Cells[0, 13]:='Heading';                           {33}
    gridGPSdata.Cells[0, 14]:='Yaw in earth frame from north';     {24}
    gridGPSdata.Cells[0, 15]:='Number of satellites visible';      {24, 25}
    gridGPSdata.Cells[0, 16]:='GPS fix type';                      {24}
    gridGPSdata.Cells[0, 17]:='GPS HDOP';                          {24}
    gridGPSdata.Cells[0, 18]:='GPS VDOP';                          {24}
    gridGPSdata.Cells[0, 19]:='Position uncertainty';              {24}
    gridGPSdata.Cells[0, 20]:='Altitude uncertainty';              {24}
    gridGPSdata.Cells[0, 21]:='Speed uncertainty';                 {24}
    gridGPSdata.Cells[0, 22]:='Heading / track uncertainty';       {24}
    gridGPSdata.Cells[0, 23]:='Battery voltage';                   {1}
    gridGPSdata.Cells[0, 24]:='Usage of the mainloop time';        {1}
  finally                     {Sad: Yuneec does not deliver data in 25}
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
    23: result:=' [V]';
    24: result:=' [%]';
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
  StatusBar.Panels[2].Text:=MsgFormatTypeToStr(MsgFormatType);
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

{   0: rInvalid;
    1: MAVlink V2 common            ToDo
    2: Yuneec H520 TLOG
    3: Yuneec Mantis LOG
    4: Yuneec Mantis FlyLog
    5: Yuneec HPlus Sensor
    6: Yuneec H480 Sensor
}

function GetMsgFormatType(fn: string): byte;
var
  shortfn: string;

begin
  result:=0;                                              {default: Invalid file}
  shortfn:=LowerCase(ExtractFileName(fn));
  if ExtractFileExt(shortfn)<>'.csv' then begin
    if (pos('20', shortfn)=1) then begin
      if (pos('.tlog', shortfn)>18) then  exit(2);
    end;
    if pos('sensor_', shortfn)=1 then begin
      if pos('.bin', shortfn)=13 then exit(6);
      if pos('.txt', shortfn)=13 then exit(5);
      if pos('.txt', shortfn)>13 then exit(4);
    end;
    if (pos('yuneec_20', shortfn)=1) and
       (pos('.log', shortfn)>20) then exit(3);
    if pos('flylog_20', shortfn)=1 then exit(4);
  end;
end;

procedure TForm1.DoLoadFile(aFileName: string);
begin
  StatusBar.Panels[2].Text:='';
  ResetGlobalVariables;
  BarSatSNR.Clear;
  SatPolarSeries.Clear;
  ChartDataLineSeries1.Clear;
  ClearPositioningData;
  Application.ProcessMessages;

  if FileSize(aFileName)<500 then begin
    SetForInvalidFile(errSmallFile);
    exit;
  end;

  MsgFormatType:=GetMsgFormatType(aFileName);
  if MsgFormatType>1 then begin
    Screen.Cursor:=crHourGlass;
    SetForValidFile(aFileName);
    inputstream.LoadFromFile(aFileName);
    barStream.Max:=inputstream.Size-1;
    ProcessMAVfile(MsgFormatType);

    StatusBar.Panels[0].Text:=IntToStr(NumMsgTotal);
    StatusBar.Panels[1].Text:=IntToStr(NumMsgUsed);
    if cbSaveCSV.Checked then
      actSaveCSVExecute(self);
    Screen.Cursor:=crDefault;

  end else
    SetForInvalidFile(errInvalidFile);
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
  StatusBar.Panels[3].Text:=info;
end;

procedure TForm1.actContinueExecute(Sender: TObject);
begin
  stopping:=false;
  if MsgFormatType>1 then begin                            {Means, there was is a valid file type in progress}
    ProcessMAVfile(MsgFormatType);
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
var
  stext: string;

const
  textsep='|';

begin
  result:=true;
  data.gps_present:=true;
  SensorsHealthy:=true;
  data.sats_visible:=max8;                               {If unknown, set to UINT8_MAX}

// Yuneec: On each $FD message is a trailer after the CRC with a time stamp 8 bytes UTG in µs
  if (msg.msgformat<6) and (msg.msgformat>1) then begin
    data.yuneectime:=UnixToDateTime(YuneecTimeStampInSeconds(msg));
  end;

  case msg.msgid of
     1: SYS_STATUS(msg, offset, data);
     2: SYS_TIME(msg, offset, data);
     4: PING(msg, offset, data);
    24: GPS_RAW_INT(msg, offset, data);
    25: if msg.msglength>1 then begin
          GPS_STATUS(msg, offset, data);
          sleep(barSleepTime.Position);
        end;
//    32: LOCAL_POSITION_NED(msg, offset, floatdata);
    33: GLOBAL_POSITION_INT(msg, offset, data);
    253: begin
           stext:=STATUSTEXT(msg, offset, textsep);
           StatusTextList.Lines.Add(FormatDateTime(timezzz, data.boottime)+tab2+
                                    stext.Split([textsep])[1]+tab2+'['+
                                    stext.Split([textsep])[0]+']');
         end;
  else
    result:=false;                                       {Messages not decoded}
  end;
  if not data.gps_present then
    gpsPresent:=false;
  if not data.sensors_OK then
    SensorsHealthy:=false;
end;

procedure TForm1.ProcessMAVFile(const FormatType: byte);
var
  b, offset: byte;
  msg: TMAVmessage;
  data: TGPSdata;
  UsedMagic: byte;

  procedure SetFixPartValuesForMsgType;
  begin
    case FormatType of
      2..5: begin
        // Read all message bytes from stream: Fix part + Payload + CRC + 8 bytes YuneeTimestamp [µs]
        inputstream.ReadBuffer(msg.msgbytes, msg.msglength+LengthFixPartFD+10);
        msg.sysid:=msg.msgbytes[5];
        msg.targetid:=msg.msgbytes[6];
        msg.msgid:=MavGetInt32(msg, 7) and $FFFFFF;      {MAVlink Message ID has 3 bytes}
        offset:=LengthFixPartFD;
        UsedMagic:=$FD;
      end;

      6: begin
        // Read all message bytes from stream: Fix part + Payload + CRC
        inputstream.ReadBuffer(msg.msgbytes, msg.msglength+LengthFixPartBC+2);
        msg.sysid:=msg.msgbytes[3];
        msg.targetid:=msg.msgbytes[4];
        msg.msgid:=msg.msgbytes[5];
        offset:=LengthFixPartBC;
        UsedMagic:=$BC;
      end;
    end;
  end;

  function CheckMessage: boolean;      {ToDo depending on use case}
  begin
    result:=false;
    case FormatType of
      1..5: begin
        result:=(msg.msgid<MaxMsgID) and
                (msg.msgbytes[2]=0) and
                (msg.msgbytes[3]=0);
//        result:=CheckCRC16MAV(msg, LengthFixPartFD);

      end;
      6: result:=CheckCRC16X25(msg, LengthFixPartBC);
    end;
    msg.valid:=result;
  end;

begin
  ClearMAVmessage(msg);
  GPSdata_SetDefaultValues(data);
  if inputstream.Position=0 then begin
    StatusTextList.Clear;
    WriteHeaderCSVlist;
    gpsPresent:=true;
    SensorsHealthy:=true;
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
      msg.msgformat:=MsgFormatType;
      if DecodeOneSensorMessage(msg, offset, data) then begin
        inc(NumMsgUsed);
        if (data.boottime>0) and msg.valid then begin
          if (not cbFast.Checked) and (data.fix_type>0) then begin
            CreateSatPolarDiagram(data);
            CreateSatSNRBarChart(data);
          end;
          WriteGridGPSdataValues(data);
          WriteGPSDataCSVlist(data);
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
  if not gpsPresent then
    lblTime.Caption:=rsGPSmissing
  else
    if not SensorsHealthy then
      lblTime.Caption:=errSensorsNotHealthy;
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

      if (MsgFormatType>1) and (MsgFormatType<6) then
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

//  From SYS_STATUS (1)
      gridGPSdata.Cells[1, 23]:=FormatMilliVolt(sats.voltage);
      gridGPSdata.Cells[1, 24]:=FormatDeziProcent(sats.load);

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
  result:=NaN;                                           {TAChart skips NaN}
  s:=trim(ValueFromTable);
  for i:=length(s) downto 1 do
    if not (s[i] in digits) then
      delete(s, i, 1);
  if s<>'' then
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
         if trim(csvlist[i].Split([sep])[0])<>'' then begin
           value:=GetFloatFromTable(csvlist[i].Split([sep])[dataindex]);
           timestamp:=ScanDateTime(timezzz, csvlist[i].Split([sep])[0]);
           ChartDataLineSeries1.AddXY(timestamp, value);
         end;
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
    BottomAxis.Marks.Source:=DateTimeIntervalDataSource;
    BottomAxis.Marks.Style:=smsLabel;
    BottomAxis.Marks.Format:='%2:s';
    if dataindex>2 then begin
      LeftAxis.Title.Caption:=gridGPSdata.Cells[0, dataindex]+IndexToMeasurementUnit(dataindex);
      LeftAxis.Title.Visible:=true;
    end;
  end;
  DateTimeIntervalDataSource.DateTimeFormat:='nn:ss';
  DateTimeIntervalDataSource.Params.NiceSteps:='5|10';   {x-axis labels wider steps}
  ChartDataLineSeries1.SeriesColor:=clDataSerie1;
end;


end.

