function [ data, errmsg ] = readPosprocFile( varargin )
%
% Project : NAVLIB library
%
% Copyright (2010) EPFL-TOPO. All Rights Reserved.
%
% File : readPosprocFile.m
% Author : Yannick Stebler
%
% Usage :
%
% [ data ] = readPosprocFile()
% [ data, errmsg ] = readPosprocFile( filename, 'Argument', value, ... )
% [ data, errmsg ] = readPosprocFile( 'Argument', value, ... )
% 
% Purpose :
%   This function decodes POSPROC files.
%
% Input:
%   filename : string containing file name
%   Argument:
%       - 'Filename' : string containing file name to decode
%       - 'Format' : posproc file format which can take the following
%          values: 
%               * 'IMU' : IMU data file (imu_xxx.dat)
%               * 'SNV' : Strapdown navigation data file (sbet/snv_xxx.out)
%               * 'IDX' : Filter state index file (idx_xxx.txt)
%               * 'ERS' : Filter error state file (ers_xxx.out)
%               * 'GPS' : GPS position file (gps_xxx.dat)
%       - 'Frequency' : frequency of the data (required for 'IMU' files)
%       - 'IMUType' : IMU type which can take the following values (option
%         for 'IMU' files):
%               * 'IMAR' : FSAS-IMAR device
%               * 'AIRINS' : IXSEA-AIRINS device
%               * 'LN200' : Grumman LN200 device
%               * 'XSENS' : XSens MTi-G, MTi and MTx device
%               * 'NAVLIB' : Converted IMU file obtained with NAVCON
%       - 'Type' : Recorded format for 'IMU' files which can take the
%          following values (option for 'IMU' files):
%               * 'Long' : Integer IMU data record (default)
%               * 'Float' : Floated IMU data record
%       - 'AngleFormat' : angle format which can take the following values
%          (option for 'SNV' files):
%               * 'rad' : radian angles (default)
%               * 'grad' : grad angles
%               * 'deg' : degree angles
%       - 'StartTime' : starting time comprised within [0,999999] interval
%          from which data should be extracted (option for 'SNV' files).
%       - 'EndTime' : ending time comprised within [0,999999] interval from
%          until which data should be extracted (option for 'SNV' files).
%       - 'IdxData' : structure containing IDX data which can be obtained
%          by decoding 'IDX' file (required for 'ERS' files).
%       - 'Version' : posproc version which can take the following values
%          (option for 'ERS' files):
%               * 2.0 : posproc version 2.0
%       - 'AccScale' : accelerometer scale factor to ouput observation in 
%         m/s (required for 'IMU' files)
%       - 'GyrScale' : gyroscope scale factor to ouput observation in rad 
%         (required for 'IMU' files)
%       - 'Verbose' : display messages if set true
%   
% Output:
%   data : returns decoded data if success, else -1
%   errmsg : if data is -1, errmsg contains error message
%
% Important note:
%   If 'ERS' files have to be decoded, the corresponding 'IDX' file must
%   first be decoded. This will provide the 'IdxData' structure required
%   when decoding 'ERS' files.
%
% History:
%   01-2010 - Y.Stebler - Implementation
%   11-2010 - Y.Stebler - Correction
%   01-2013 - J.Skaloud - NavChip (NC_INT, NC_FLT) format
%
% -------------------------------------------------------------------------



% -------------------------------------------------------------------------
% ARGUMENT PARSING
% -------------------------------------------------------------------------

if nargin == 0 || isempty(varargin)

    % Provide file name and path
    [ fname, PATHNAME ] = uigetfile('*.*', 'Open PosProc FLOAT binary format');
    settings.filename = [PATHNAME fname];
    
end

i = 1;
errmsg = [];
data = -1;

% Don't display messages
settings.verbose = false;

if (nargin < 2) || ( ~isValidHandle(varargin{1}) && ischar(varargin{1}) )
    settings.filename = varargin{1};
    i = 2;
end

if nargin > 1
    
    % Loop through arguments
    while i <= nargin
        
        % Check if it's a valid handle
        if isValidHandle(varargin{i})
            
            % If valid handle, parse value
            switch lower(varargin{i})
                
                case 'filename'
                    if ( i+2 <= nargin && ischar(varargin{i+1}) )
                        settings.filename = varargin{i+1};
                        i = i+2;
                    else
                        i = i+1;
                    end
                    
                 case 'imutype'
                    if ( i+1 <= nargin && ischar(varargin{i+1}) )
                        settings.imutype = varargin{i+1};
                        i = i+2;
                    else
                        i = i+1;
                    end
    
                case 'frequency'
                    if ( i+1 <= nargin && isscalar(varargin{i+1}) )
                        settings.frequency = varargin{i+1};
                        i = i+2;
                    else
                        i = i+1;
                    end
                    
                case 'format'
                    if ( i+1 <= nargin && ischar(varargin{i+1}) )
                        settings.format = varargin{i+1};
                        i = i+2;
                    else
                        i = i+1;
                    end
                    
                case 'type'
                    if ( i+1 <= nargin && ischar(varargin{i+1}) )
                        settings.type = varargin{i+1};
                        i = i+2;
                    else
                        i = i+1;
                    end
                    
                case 'angleformat'
                    if ( i+1 <= nargin && ischar(varargin{i+1}) )
                        settings.angle = varargin{i+1};
                        i = i+2;
                    else
                        i = i+1;
                    end
                    
                case 'starttime'
                    if ( i+1 <= nargin && isscalar(varargin{i+1}) )
                        settings.starttime = varargin{i+1};
                        i = i+2;
                    else
                        i = i+1;
                    end
                    
                case 'endtime'
                    if ( i+1 <= nargin && isscalar(varargin{i+1}) )
                        settings.endtime = varargin{i+1};
                        i = i+2;
                    else
                        i = i+1;
                    end
                    
                case 'idxdata'
                    if ( i+1 <= nargin && isstruct(varargin{i+1}) )
                        settings.idx = varargin{i+1};
                        i = i+2;
                    else
                        i = i+1;
                    end
                    
                case 'ppimutype'
                    if ( i+1 <= nargin && ischar(varargin{i+1}) )
                        settings.type = varargin{i+1};
                        i = i+2;
                    else
                        i = i+1;
                    end
                    
                case 'accscale'
                    if ( i+1 <= nargin && isscalar(varargin{i+1}) )
                        settings.accscale = varargin{i+1};
                        i = i+2;
                    else
                        i = i+1;
                    end
                    
                 case 'gyrscale'
                    if ( i+1 <= nargin && isscalar(varargin{i+1}) )
                        settings.gyrscale = varargin{i+1};
                        i = i+2;
                    else
                        i = i+1;
                    end
                    
                  case 'verbose'
                    if ( i+1 <= nargin && isscalar(varargin{i+1}) )
                        settings.verbose = varargin{i+1};
                        i = i+2;
                    else
                        i = i+1;
                    end
                    
                    
                otherwise
                    i = i+1;
                    
            end
            
        else
            
            i = i+1;

        end
        
    end
    
end



% -------------------------------------------------------------------------
% ARGUMENT CHECK
% -------------------------------------------------------------------------

% Check format
if ~isfield(settings, 'format') || ~isValidFormat(settings.format)
    settings.format = 'imu';
    if settings.verbose
        fprintf('Warning [readPosprocFile]: Format set to %s\n',...
            settings.format);
    end
end

if strcmpi(settings.format, 'imu') || strcmpi(settings.format, 'snv')

    if ~isfield(settings, 'angle') || ~isValidAngleFormat( settings.angle )
        settings.angle = 'rad';
        if settings.verbose
            fprintf('Warning [readPosprocFile]: Angle format set to "%s"\n',...
                settings.angle);
        end
    end
    
end

if strcmpi(settings.format, 'imu')
    % Check IMU type
    if ~isfield(settings, 'imutype') || ~isValidIMU(settings.imutype)
        settings.imutype = 'custom';
        if settings.verbose
            fprintf('Warning [readPosprocFile]: IMU type set to "%s"\n', ...
                settings.type);
        end
    end
    
    % Check type
    if ~isfield(settings, 'type') || ~isValidType(settings.type)
        settings.type = 'long';
        if settings.verbose
            fprintf('Warning [readPosprocFile]: Encoding format type set to %s\n', ...
                settings.type);
        end
    end

    % Check frequency
    if isfield(settings, 'frequency') && settings.frequency < 0
        settings.frequency = [];
        if settings.verbose
            fprintf('Warning [readPosprocFile]: Frequency set to %d Hz\n',...
                settings.frequency);
        end
    end
end

if strcmpi(settings.format, 'snv')

    % Check start time
    if ~isfield(settings, 'starttime') || settings.starttime < 0 
        settings.starttime = 0;
        if settings.verbose
            fprintf('Warning [readPosprocFile]: Start time set to %d (s)\n',...
                settings.starttime);
        end
    end

    % Check end time
    if ~isfield(settings, 'endtime') || settings.endtime < 0 
        settings.endtime = 999999;
        if settings.verbose
            fprintf('Warning [readPosprocFile]: End time set to %d (s)\n',...
                settings.endtime);
        end
    end
end

if strcmpi(settings.format,'ers')
    % Check idxdata structure
    if ~isfield(settings, 'idx') || ~isValidIdxData( settings.idx )
        errmsg = 'Invalid IDX data structure';
        return;
    end
    
    % Check Posproc version
    if ~isfield(settings,'version') || ~isValidVersion( settings.version )
        settings.version = 2.0;
        if settings.verbose
            fprintf('Warning [readPosprocFile]: Posproc version set to %.1f\n',...
                settings.version);
        end
    end
end



% -------------------------------------------------------------------------
% ADAPT IMU SPECIFIC SETTINGS
% ------------------------------------------------------------------------- 


if strcmpi(settings.format, 'imu')
    
    if strcmpi( settings.imutype, 'IMAR' )
        
        settings.accscale = 0.00152588/1000;         % Scale acc to m/s
        settings.gyrscale = 0.10000000*pi/180/3600;  % Scale gyro to rad
        settings.type = 'long';
        
    elseif strcmpi( settings.imutype, 'AIRINS' )
        
        settings.accscale = 1/1000;                 % scale acc to m/s
        settings.gyrscale = pi/180/3600;            % Scale gyro to rad
        settings.type = 'float';
        
    elseif strcmpi( settings.imutype, 'LN200' )
        
        settings.accscale = 1/16384.0;              % scale acc to m/s
        settings.gyrscale = 1/2097152.0;            % Scale gyro to rad
        
        settings.type = 'long';
        
    elseif strcmpi( settings.imutype, 'XSENS' )
        
        settings.accscale = 1/1000.0;               % scale acc to m/s
        settings.gyrscale = pi/180/3600;            % Scale gyro to rad
        settings.type = 'float';
        
    elseif strcmpi( settings.imutype, 'NAVLIB' )
        
        settings.accscale = 1/1000.0;               % scale acc to m/s
        settings.gyrscale = pi/180/3600;            % Scale gyro to rad
        settings.type = 'float';
        
    elseif strcmpi( settings.imutype, 'NC_INT' )
        
        settings.accscale = 39.0625e-6;             % scale acc to m/s
        settings.gyrscale = 0.00000625;             % Scale gyr to rad
        settings.type = 'long' ; 
        
   elseif strcmpi( settings.imutype, 'NC_FLT' )
        
        settings.accscale = 1/1000.0;               % scale acc to m/s
        settings.gyrscale = pi/180/3600;            % Scale gyr to rad
        settings.type = 'float' ; 
        
    else
        
        settings.accscale = 1;
        settings.gyrscale = 1;
        
    end
        
end


% -------------------------------------------------------------------------
% READ FILE
% -------------------------------------------------------------------------

% Decode file according to its format
switch ( lower(settings.format) )
    
    case 'imu'
        
        if settings.verbose
            fprintf('Info [readPosprocFile]: Decoding IMU file from %s...\n',...
                settings.filename);
        end
        [data, errmsg] = decodeIMUFile( settings );
        
    case 'snv'
        
        if settings.verbose
            fprintf('Info [readPosprocFile]: Decoding Standard Navigation Record file from %s...\n',...
                settings.filename);
        end
        [data, errmsg] = decodeSNVFile( settings );
        
    case 'idx'
        
        if settings.verbose
            fprintf('Info [readPosprocFile]: Decoding Filter State ID file from %s...\n',...
                settings.filename);
        end
        [data, errmsg] = decodeIDXFile( settings );
        
    case 'ers'
        
        if settings.verbose
            fprintf('Info [readPosprocFile]: Decoding Estimated Error State file from %s...\n',...
                settings.filename);
        end
        [data, errmsg] = decodeERSFile( settings );
        
    case 'gps'
        
        if settings.verbose
            fprintf('Info [readPosprocFile]: Decoding GPS file from %s...\n',...
                settings.filename);
        end
        [data, errmsg] = decodeGPSFile( settings );
        
    otherwise
        
        errmsg = 'Invalid format!';
        data = [];
        
end

if ( ~isstruct(data) && isempty(data) < 0 ) || ~isempty(errmsg)
    if settings.verbose
        fprintf('Error [readPosprocFile]: %s\n', errmsg);
    end
    return;
else
    if settings.verbose
        fprintf(rewind(4));
        fprintf(' -- DONE!\n');
    end
end

return;




% -------------------------------------------------------------------------
% AUXILIARY FUNCTIONS
% -------------------------------------------------------------------------


function ret = isValidHandle( arg )

validHandles = [ 'frequency', 'format', 'type', 'angleformat', 'endtime',...
    'idxdata', 'version', 'ppimutype', 'verbose' ];

if isempty(findstr(lower(arg), validHandles))
    ret = false;
else
    ret = true;
end

return;


function ret = isValidFormat( arg )

validFormat = [ 'imu', 'snv', 'idx', 'ers', 'gps' ];

if isempty(findstr(lower(arg), validFormat))
    ret = false;
else
    ret = true;
end

return;


function ret = isValidIMU( arg )

validIMU = [ 'airins', 'imar', 'ln200', 'xsens', 'navlib', ...
             'nc_int', 'nc_flt','unknown' ];

if isempty(findstr(lower(arg), validIMU))
    ret = false;
else
    ret = true;
end

return;


function ret = isValidType( arg )

validType = [ 'long', 'float' ];

if isempty(findstr(lower(arg), validType))
    ret = false;
else
    ret = true;
end

return;


function ret = isValidAngleFormat( arg )

validAngleFormat = [ 'rad', 'grad', 'deg' ];

if isempty(findstr(lower(arg), validAngleFormat))
    ret = false;
else
    ret = true;
end

return;


function ret = isValidIdxData( arg )

ret = true;

if ~isfield(arg,'filter') || ~isfield(arg, 'measurement') ...
    || ~isfield(arg, 'ijx') || ~isfield(arg, 'ijm')
    ret = false;
end


return;


function ret = isValidVersion( arg )

ret = true;

if arg ~= 2.0
    ret = false;
end

return;


function [data, errmsg] = decodeIMUFile( settings )
%
%
% File Description:
%
%   Floated IMU Data Record (FLOAT)
%       time (seconds)
%       x incremental angle (arc-seconds)
%       y incremental angle (arc-seconds)
%       z incremental angle (arc-seconds)
%       x incremental velocity (mm/seconds)
%       y incremental velocity (mm/seconds)
%       z incremental velocity (mm/seconds)
%
%   Integer IMU Data Record (LONG)
%       time (seconds)
%       x incremental angle (gyro pulses)
%       y incremental angle (gyro pulses)
%       z incremental angle (gyro pulses)
%       x incremental velocity (acc pulses)
%       y incremental velocity (acc pulses)
%       z incremental velocity (acc pulses)
 
% Initialize values
data = -1;
errmsg = [];

% open and read file 
fid = fopen( settings.filename, 'r'); 

% Check file
if fid < 0
    errmsg = 'Cannot open file!';
    return;
end

% Read data
switch lower(settings.type)
    
    case 'long'

        [time, ~ ] = fread( fid, [1,inf], 'double', 24 );
        fseek( fid, 8, 'bof');
        [data, ~ ] = fread( fid, [6,inf], '6*long', 8);
        
    case 'float'
        
        [time, ~ ] = fread( fid, [1,inf], 'double', 48 );
        fseek( fid, 8, 'bof');
        [data, ~ ] = fread( fid, [6,inf], '6*double', 8);
        
    otherwise
        
        errmsg = 'Unknown Posproc IMU file type!';
        return;
        
end

% Close file
fclose(fid); 

data = [ time ; data ];

if ~isfield(settings, 'frequency') || isempty(settings.frequency)
    % Data Rate
    f = round( mean(1./diff(data(1,:))) );
    if settings.verbose
        fprintf('Info [readPosprocFile] : Detected sampling frequency: %.2f [Hz]\n',f)
    end
else
    f = settings.frequency;
end

% Scale factors
sg = f * settings.gyrscale;
sa = f * settings.accscale;

data(2:4,:) = sg.* data(2:4,:);
data(5:7,:) = sa.* data(5:7,:);

if strcmpi(settings.angle, 'deg')
    data(:,2:4) = 180/pi .* data(:,2:4);
end
return;


function [data, errmsg] = decodeGPSFile( settings )
%
%
% File Description:
%
%   GPS position file format
%       time (seconds)
%       Latitude (decimal degrees)
%       Longitude (decimal degrees)
%       Height (m)
%       East velocity (m/sec)
%       North velocity (m/sec)
%       Up velocity (m/sec)
%       East position variance (m*m)
%       North position variance (m*m)
%       Up position variance (m*m)
%    

% Initialize values
data = -1;
errmsg = [];

% open and read file 
fid = fopen( settings.filename, 'r'); 

% Check file
if fid < 0
    errmsg = 'Cannot open file!';
    return;
end

% Read data     
[time, ~ ] = fread( fid, [1,inf], 'double', 72 );
fseek( fid, 8, 'bof');
[data, ~ ] = fread( fid, [9,inf], '9*double', 8);


% Close file
fclose(fid); 

data = [ time ; data ];

return;


function [data, errmsg] = decodeSNVFile( settings )
%  y = possnv(file_in, startime, endtime, aformat)
%
%  Reads APPLANIX nav/smoth solution file_in('SNV_xxx.out' or SBET_xxx.out')  
%  data from startime to endtime and outputs its subset(10) to y
%
%  INPUT: FILE_IN: APPLANIX SNV/SBET_xxx.out, 17xdouble=(136 bytes)
%                  [t lat long ht xwv ywv zwv r p wh xa ya za xar yar zar]
%			 TIMEs:   start/end times in GPS sec of the week
%         AFORMAT: angular format on output: 'DEG'(default) 'GRAD' or 'RAD'       
%
%  OUTPUT: y vector(10,:) of format: 
%              time  lat lon  ht ve vn vu rl pt head 
%              1      2   3   4   5  6  7  8  9  10 
%              [sec] -[deg]- [m] --[m/s]- ---[deg]--
%
%      function used:  [f count] = fread(fid,[17,inf],'double')
%      Jan Skaloud, Oct01
% ----------------------------------------------------------
%  Input record: 17xdouble=(136 bytes)
%                     	1  time  			sec_of_week 
%                      	2  latitude   		rad
%                       3  longitude  		rad
%                       4  altitude       meters
%                       5  x_wander_vel   m/s
%                       6  y_wander_vel   m/s
%                       7  z_wander_vel  	m/s
%                       8  roll          	radians
%                       9  pitch         	radians
%                       10 wander_heading radians
%                       11 wander angle   radians
%                       12 x body accel   m/s^2
%                       13 y body accel   m/s^2
%                       14 z body accel   m/s^2
%                       15 x angular rate rad/s
%                       16 y angular rate rad/s
%						17 z angular rate rad/s					
% This is what is written in the ouput record:
%                       1   time            sec_of_week
%                       2   latitude        rad
%                       3   longitude       rad
%                       4   altitude        m
%                       5   north velocity  m/s
%                       6   east velocity   m/s
%                       7   down velocity   m/s
%                       8   roll            rad
%                       9   pitch           rad
%                       10  heading         rad
%                       11  x body accel    m/s^2
%                       12  y body accel    m/s^2
%                       13  z body accel    m/s^2
%                       14  x angular rate  rad/s
%                       15  y angular rate  rad/s
%						16  z angular rate  rad/s	
errmsg = [];
data = -1;

% Switch among angle types
switch  lower(settings.angle)
    
    case 'deg',
        SCALE = 180/pi; 
      
    case 'grad',
        SCALE = 200/pi; 
      
    otherwise,
        
        SCALE = 1; % default is RAD
  
end

rec_len=17;

% Open file and read file
fid = fopen(settings.filename, 'r');
if fid < 0
    errmsg = 'Cannot open file!';
    return;
end
[f, ~] = fread(fid,[rec_len,inf],'double');
fclose(fid);

% Find time range
[si, ei] = findstrt(f(1,:),settings.starttime,settings.endtime);

% GPS time conversion (in sow)
[time,~,~] = timemode(f(1,si:ei),'r');

% Transform the Wander frame to local ENU frame
vxi=5;  % wander_x vel
vyi=6;  % wander_y vel
vzi=7;  % wander_z vel
whi=10; % wander heading
wai=11; % wander angel

hd = f(whi,:)-f(wai,:); 

vn = f(vxi,:).*cos(f(wai,:)) - f(vyi,:).*sin(f(wai,:));
ve = (-1).*f(vxi,:).*sin(f(wai,:)) - f(vyi,:).*cos(f(wai,:));
vd = f(vzi,:);

% write the ouptut
data = [time;f(2,si:ei).*SCALE; f(3,si:ei).*SCALE; f(4,si:ei); vn(si:ei);...
    ve(si:ei); vd(si:ei); f(8,si:ei).*SCALE; f(9,si:ei).*SCALE; ...
    hd(si:ei).*SCALE; f(12,si:ei); f(13,si:ei); f(14,si:ei); ...
    f(15,si:ei).*SCALE; f(16,si:ei).*SCALE; f(17,si:ei).*SCALE];

return;


function [time,lbl,first] = timemode(gpsec,modeT)
%function [time,lbl,first] = timemode(gpsec,modeT)
%
%TIME conversion from gps seconds of the week
%
%input:  gpsec - gps time in seconds of the week
%        modeT - 's' = seconds 
%              - 'm' = minutes
%              - 'h' = hours
%                 default = seconds of the week
%output: time  - converted time vector with subtracted startime
%        label - printed label 
%        first - gps time in seconds of the week of the first epoch

TimeMode=0 ;

if nargin > 1, 
   if modeT == 's',
      TimeMode = 1; 
   elseif modeT == 'm',
      TimeMode = 2; 
   elseif modeT == 'h'
      TimeMode = 3;
   end
end

first = gpsec(1); 

switch TimeMode
case 0,
   time = gpsec;
   lbl=sprintf('GPS Time(sec)');
case 1,
   time = gpsec - first;
   lbl=sprintf('GPS Time(sec), 0=%8.1f sec of GPS week',first);
case 2,
   time = (gpsec-first)./60;
   lbl=sprintf('Time(min), 0=%8.1f sec of GPS week',first); 
case 3,
   time = (gpsec-first)./3600;
   lbl=sprintf('Time(hrs), 0=%8.1f sec of GPS week',first);
end


return;


function [data, errmsg] = decodeIDXFile( settings )
%
%[ijx,ijm] = posidx(project)
%
% Reads posproc indexing file of filter+measurement setup  
% Returns  Filt- [Ftotal ; Fnow ] 
%          Meas  [Fnow Fmax]
%           ijx- vector of filter setup (indexes)
%           ijm- vector of measurement setup (indexes)

nBLOCK = 5;  % size of filter block 
             % 1: system states (SS), 
             % 2: Gauss_Markov 2nd order (GM2)
             % 3: Gauss_Markov 1st order (GM1)
             % 4: ?empty 
             % 5: Ranom Constant (RC)
nMAX = 90; 

errmsg = [];
if nargin ~= 1
    data = -1;
    errmsg = 'Wrong input argument!';
    return; 
end

msg1 = 'Current size of KF is not compatible with ppview';

%filename = strcat('IDX_', project, '.txt');
[fid,~] = fopen(settings.filename, 'r'); 
if fid<0
    errmsg = 'Cannot open index file'; 
    return; 
end

temp = zeros(1,nMAX); 
last = 0; 

while 1
 tline = fgetl(fid);
 if ~ischar(tline), break; end                      % checks EOF

 k=findstr(tline,'*');
 if ~isempty(k),                                    % checks line for comment
     tnumber = tline(1:k(1)-1);                     % disects the comment
 else
     tnumber = tline;
 end

 [v, count, ~] = sscanf(tnumber,'%d', inf);     % scanfs the line
 if count > nMAX
     errmsg = 'Formating mistmatch in IDX_*.txt'; 
     return; 
 end
 if count < 1
     continue; 
 end

 if ~isempty( findstr(tline,'Total block dimensions') ),
     if count ~= nBLOCK, errmsg = msg1; return; end
     Ftot = v'; 
 elseif ~isempty( findstr(tline,'Selected block dimensions') ),
     if count ~= nBLOCK, errmsg = msg1; return; end
     Fcur = v';   
 elseif~isempty( findstr(tline,'Current, max no of meas') ),
     if count ~= 2, errmsg = msg1; return; end
     Meas = v';   
 elseif ~isempty( findstr(tline,'JX')),                 % reads last line of JX
     temp(last+1:last+count)=v';                        
     JX = temp(1,1:last+count);
     if length(JX)~=sum(Ftot), errmsg = msg1; return; end
     last = 0;
 elseif ~isempty( findstr(tline,'JM')),                 % reads last line of JM
     temp(last+1:last+count)=v';
     JM = temp(1,1:last+count);
     if length(JM)~=Meas(2), errmsg = msg1; return; end
     last = 0;
 elseif last+count<=nMAX, 
     temp(last+1:last+count)=v';                        % stack up line in a temp vector
     last = last+count; 
 else
     errmsg = msg1; return;
 end              
end
fclose(fid);

n = sum(Fcur);
ijx = zeros(n,1);
for i=1:n,
    ijx(i) = find(JX(:)==i);                              %finds filter indx
end

m = Meas(1);
ijm = zeros(m,1);
for i=1:m,
    ijm(i) = find(JM(:)==i);
end

Filt = [Ftot; Fcur];

% Build output structure
data.filter = Filt;
data.measurement = Meas;
data.ijx = ijx;
data.ijm = ijm;


return;


function [data, errmsg] = decodeERSFile( settings )

data = -1;
errmsg = [];


% open and read file 
fid = fopen( settings.filename, 'r'); 

% Check file
if fid < 0
    errmsg = 'Cannot open file!';
    return;
end

rec_len = 1 + size(settings.idx.ijx,1);

[f, ~] = fread(fid,[rec_len,inf],'double');
fclose(fid);

[~, data.labels, errmsg] = getLabels( settings );

if isempty(errmsg)
    data.stateErrors = f';
end

return;


function [out, labels, errmsg] = getLabels( settings )
%
%[T,S,Sl, M,D] = pplabels(version)
%
% Reads posproc label file used in ppview 'ppvlbls.txt' 
% Returns T- cell(:,1) of trajectory labels
%         S- cell(:,1) of system states labels (system 2nd_order 1st_order constat)
%         M- cell(:,1) of measurement labels
%         D- cell(:,1) of difference labels
%

out = -1;
labels = -1;
errmsg = [];

if settings.version == 2.0, 
    out.t = 21;
    out.T = { 'Top View: Latitude (degrees)'
'View from East: Altitude (meters)'
'View from South: Altitude (meters)'
'Latitude (degrees)'
'Longitude (degrees)'
'Altitude (meters)'
'Roll (degrees)'
'Pitch (degrees)'
'True Heading (degrees)'
'North Velocity (meters/second)'
'East Velocity (meters/second)'
'Down Velocity (meters/second)'
'Total Speed (meters/second)'
'Ground Speed (meters/s)'
'X Body Acceleration (g)'
'Y Body Acceleration (g)'
'Z Body Acceleration (g)'
'Total Acceleration (g)'
'X Body Angular Rate (degrees/second)'
'Y Body Angular Rate (degrees/second)'
'Z Body Angular Rate (degrees/second)'
};
    out.s = [10 6 47 0 27];
    out.S = { 'S/D navigator x position error (meters)'
'S/D navigator y position error (meters)'
'S/D navigator z position error (meters)'
'S/D navigator x velocity error (meters/second)'
'S/D navigator y velocity error (meters/second)'
'S/D navigator z velocity error (meters/second)'
'S/D navigator x misalignment (arc-min)'
'S/D navigator y misalignment (arc-min)'
'S/D navigator sin(z misal) (arc-min)'
'S/D navigator cos(z misal)-1 (radians)'
'S/D navigator x position RMS error (meters)'
'S/D navigator y position RMS error (meters)'
'S/D navigator z position RMS error (meters)'
'S/D navigator x velocity RMS error (meters/second)'
'S/D navigator y velocity RMS error (meters/second)'
'S/D navigator z velocity RMS error (meters/second)'
'S/D navigator x misalignment RMS error (arc min)'
'S/D navigator y misalignment RMS error (arc min)'
'S/D navigator sin(psi_z) RMS (arc-min)'
'S/D navigator cos(psi_z)-1 RMS (radians)'
'GPS N position error (meters)'
'GPS N velocity error (meters/second)'
'GPS E position error (meters)'
'GPS E velocity error (meters/second)'
'GPS D position error (meters)'
'GPS D velocity error (meters/second)'
'x accelerometer drift (micro-g)'
'y accelerometer drift (micro-g)'
'z accelerometer drift (micro-g)'
'x accelerometer scale factor drift (ppm)'
'y accelerometer scale factor drift (ppm)'
'z accelerometer scale factor drift (ppm)'
'yz accelerometer nonorthogonality (arc-sec)'
'xz accelerometer nonorthogonality (arc-sec)'
'xy accelerometer nonorthogonality (arc-sec)'
'x gyro drift (degrees/hour)'
'y gyro drift (degrees/hour)'
'z gyro drift (degrees/hour)'
'x gyro scale factor drift (ppm)'
'y gyro scale factor drift (ppm)'
'z gyro scale factor drift (ppm)'
'xx gyro g-sensitive drift (degrees/hour/g)'
'xz gyro g-sensitive drift (degrees/hour/g)'
'yy gyro g-sensitive drift (degrees/hour/g)'
'yz gyro g-sensitive drift (degrees/hour/g)'
'zz gyro g-sensitive drift (degrees/hour/g)'
'zy gyro g-sensitive drift (degrees/hour/g)'
'xy gyro nonorthogonality (arc-sec)'
'xz gyro nonorthogonality (arc-sec)'
'yx gyro nonorthogonality (arc-sec)'
'yz gyro nonorthogonality (arc-sec)'
'zx gyro nonorthogonality (arc-sec)'
'zy gyro nonorthogonality (arc-sec)'
'GPS N position error (meters)'
'GPS E position error (meters)'
'GPS D position error (meters)'
'GPS N velocity error (meters/second)'
'GPS E velocity error (meters/second)'
'GPS D velocity error (meters/second)'
'x IMU->GPS lever arm error (meters)'
'y IMU->GPS lever arm error (meters)'
'z IMU->GPS lever arm error (meters)'
'x DVS/DMI scale factor error (%)'
'y DVS/DMI boresight error (degrees)'
'z DVS/DMI boresight error (degrees)'
'x body velocity error (meters/second)'
'y body velocity error (meters/second)'
'z body velocity error (meters/second)'
'N wind or sea current (meters/second)'
'E wind or sea current (meters/second)'
'Baro bias(meters)'
'Baro scale factor error (%)'
'heading offset(degrees)'
'x accelerometer bias (micro-g)'
'y accelerometer bias (micro-g)'
'z accelerometer bias (micro-g)'
'x accelerometer scale factor error (ppm)'
'y accelerometer scale factor error (ppm)'
'z accelerometer scale factor error (ppm)'
'yz accelerometer nonorthogonality (arc-sec)'
'xz accelerometer nonorthogonality (arc-sec)'
'xy accelerometer nonorthogonality (arc-sec)'
'x gyro bias (degrees/hour)'
'y gyro bias (degrees/hour)'
'z gyro bias (degrees/hour)'
'x gyro scale factor error (ppm)'
'y gyro scale factor error (ppm)'
'z gyro scale factor error (ppm)'
'xx gyro g-sensitive bias (degrees/hour/g)'
'xz gyro g-sensitive bias (degrees/hour/g)'
'yy gyro g-sensitive bias (degrees/hour/g)'
'yz gyro g-sensitive bias (degrees/hour/g)'
'zz gyro g-sensitive bias (degrees/hour/g)'
'zy gyro g-sensitive bias (degrees/hour/g)'
'xy gyro nonorthogonality (arc-sec)'
'xz gyro nonorthogonality (arc-sec)'
'yx gyro nonorthogonality (arc-sec)'
'yz gyro nonorthogonality (arc-sec)'
'zx gyro nonorthogonality (arc-sec)'
'zy gyro nonorthogonality (arc-sec)' 
    };
     out.m = 23;
     out.M = {'SNV-BAR altitude (meters)'
'SNV-GPS roll (radians)'
'SNV-GPS pitch (radians)'
'sin(SNV-GPS heading) (unitless)'
'cos(SNV-GPS heading)-1 (unitless)'
'N SNV-GPS position (meters)'
'E SNV-GPS position (meters)'
'D SNV-GPS position (meters)'
'N SNV-GPS velocity (meters/second)'
'E SNV-GPS velocity (meters/second)'
'D SNV-GPS velocity (meters/second)'
'x SNV-DVS velocity (meters/second)'
'y SNV-DVS velocity (meters/second)'
'z SNV-DVS velocity (meters/second)'
'x SNV-DMI integrated velocity (meters)'
'y SNV-DMI integrated velocity (meters)'
'z SNV-DMI integrated velocity (meters)'
'N SNV-position fix (meters)'
'E SNV-position fix (meters)'
'D SNV-position fix (meters)'
'x SNV-zero velocity (meters/second)'
'y SNV-zero velocity (meters/second)'
'z SNV-zero velocity (meters/second)'
    };
    out.d =9; 
    out.D = { 'North position difference (meters)'
'East position difference (meters)'
'Down position difference (meters)'
'North velocity difference (meters/second)'
'East velocity difference (meters/second)'
'Down velocity difference (meters/second)'
'Roll difference (arc-min)'
'Pitch difference (arc-min)'
'Heading difference (arc-min)'
    };

else
    errmsg = 'Unsupported version';
    return;
end

mask = [ settings.idx.ijx(1:settings.idx.filter(1,1)) 
    settings.idx.ijx(settings.idx.filter(1,1) + 1 : end) + settings.idx.filter(2,1) ]';

labels = cell(length(mask),1);
for i = 1:length(mask)
    labels{i} = out.S{mask(i)};
end

return;