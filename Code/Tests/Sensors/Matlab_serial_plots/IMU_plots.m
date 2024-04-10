close all; clear all;

% Serial object
s = serial('COM5', 'BaudRate', 500000);

% Open serial port
fopen(s);

