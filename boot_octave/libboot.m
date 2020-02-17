%----------------------------------------------------------------------------------------------------
% Simple bootloader code for some device(s) developed in scope of LiBforSecUse EMPIR project.
% Supported devices: Z-Simulator Bias Source
%                    CMI Clock Divider v2.0
%                    LiBforSecUse Bias Subtractor
%
% Expects simple AVR (see assembly source code for details).
%
% How does it work:
% It can load the firmware when MCU contains only bootloader or during approx 1s after power up:
%   1) Set port, baudrate (9600) and either binary or Intel HEX firmware file
%   2) Start this m-file
%   3) Power up the device within some 1s after 2)
%   4) It should do the business or say why it failed
% It can also start bootloader when application is already running by sending SCPI command
% 'SYST:BOOT <passcode>' to the device:  
%   1) Set port, baudrate (9600) and either binary or Intel HEX firmware file
%   3) Power up the device, wait till ready to operate 
%   2) Start this m-file
%   4) It should do the business or say why it failed
%
% AVR MCU must be programmed with the bootloader first! Fuse bits are to be set so
% bootsection is 256 Words long (BOOTSZ1 = 1, BOOTSZ0 = 0). BOOTRST = 0 will force MCU to start
% at bootloader code address after powerup, but it is not necessary.    
%                     
% (c) Stanislav Maslan, 2020, s.maslan@seznam.cz
% The script is distributed under MIT license, https://opensource.org/licenses/MIT.
%
% Simulator details: 
%   https://github.com/smaslan/Z-sim-mutual
%
% "Lithium Batteries for Second Life Applications" project url:
%   https://www.ptb.de/empir2018/libforsecuse/home/  
%----------------------------------------------------------------------------------------------------
clc;
clear all;
close all;
warning('off');

% current path
rpth = fileparts(mfilename('fullpath'));
cd(rpth);

% RS232 communication package
pkg load instrument-control;

% firmware path (Intel HEX or raw binary file) 
fw_path = 'zsim.hex';

% COM port setup
port = 'COM5';
baud = 9600;

% bootloader passcode
passcode = '17IND10';

% empty MCU boot (first load)?
%  note: Non-zero only if MCU contains only bootloader and no application!
%        When app is already there, it is expected bootloader is invoked via SCPI command 'SYST:BOOT <passcode>'.
empty_mcu = 0;


% load FW data
[dd,nn,ext] = fileparts(fw_path);
if strcmpi(ext,'.hex')
    data = hex2bin(fw_path);
    fprintf('Firmware loaded from Intel HEX file (%d Bytes).\n',numel(data));
else
    fr = fopen(fw_path,'r');
    data = fread(fr, [1 inf], 'uint8');    
    fclose(fr);
    fprintf('Firmware loaded from binary file (%d Bytes).\n',numel(data));
end
count = numel(data);

% check port existance
comlist = instrhwinfo('serial');
if ~any(strcmpi(comlist,port))
    % port not found
    error(sprintf('Com port ''%s'' not found!',port));
endif

% timeout [in 100ms multiples]
timeout = 1;

% open port
com = serial(port, baud, timeout);

if ~empty_mcu
    
    % identify
    srl_flush(com);
    srl_write(com, "*IDN?\n");
    idn = char(srl_read(com, 128));
    idn(idn==10) = 0;
    fprintf('Instrument IDN string: %s\n',idn);
    [a,b,c,d,e] = regexp(idn,'LiBforSecUse Bias Subtractor|CMI Clock Divider|Z-Simulator Bias Source');
    if isempty(d)
        fclose(com);
        error('Instrument reported invalid IDN!');    
    endif
    
    % enter boot mode
    srl_flush(com);
    srl_write(com, sprintf('SYST:BOOT %s\n',passcode));
    ack = str2num(char(srl_read(com, 2)));
    if isempty(ack) || ~ack
        fclose(com);
        error('Entering bootloader failed! Possibly wrong passcode.');
    endif
    
endif

% longer timeout
srl_timeout(com,20);

% check bootloader ready mark
srl_flush(com);  
bcode = char(srl_read(com, 1));
if isempty(bcode) || bcode ~= 'B'
    fclose(com);
    error('Bootloader did not responded as expected!');
endif

% read bootloader message
bootmsg = char(srl_read(com, numel('Schmutzig Bootloader Ready!')+1));
sprintf('Bootloader message: %s\n',bootmsg);

% send start mark 
srl_write(com, 'R');

% check ACK  
acode = char(srl_read(com, 1));
if isempty(acode) || acode ~= 'A'
    fclose(com);
    error('Bootloader did not responded as expected!');
endif

% send passcode again
srl_write(com, sprintf('%s%c',passcode,0));

% check ACK  
acode = char(srl_read(com, 1));
if isempty(acode) || acode ~= 'A'
    fclose(com);
    error('Passcode not accepted!');
endif

% page size [Words] 
page_size = 2*double(srl_read(com, 1));
fprintf('Page size: %d Bytes\n',page_size);

% memory size [Words] 
mem_size = 2*double(typecast(srl_read(com, 2),'uint16'));
fprintf('Memory size: %d Bytes\n',mem_size);

if count > mem_size
    fclose(com);
    error(sprintf('Selected file (%d Bytes) will not fit to device memory (%d Bytes)!',count,mem_size));
endif

% page count to write
page_count = ceil(count/page_size);

% expand fw data to multiple of page size
temp = data;
data = repmat(uint8(0xFF),[page_count*page_size 1]);
data(1:count,1) = temp;
% split data per pages
data = reshape(data,[page_size page_count]);

% --- send fw data ---
for page = 1:page_count

    fprintf('Writting page #%d of #%d...              \r',page,page_count);
    
    % write page command
    srl_write(com, 'W');
    
    % write page
    srl_write(com, data(:,page));
    
    % check ACK  
    acode = char(srl_read(com, 1));
    if isempty(acode) || acode ~= 'P'
        fprintf('\n');
        fclose(com);
        error(sprintf('Page #%d verification failed!',page));
    endif   

endfor
fprintf('\n');

% write done flag
srl_write(com, 'D');

% close port and pray for success
fclose(com);

fprintf('Done!\n');
