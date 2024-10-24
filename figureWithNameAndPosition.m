% figureWithNameAndPosition.m
% function to create a figure with name and position
% Usage:
%   figureWithNameAndPosition(h,name,pos)
% where:
%   h : figure handle
%   name : string with figure name
%   pos = [posx, posy, width, height] : position and size of the figure (normalized values between 0 and 1) 
%
% Author:
%   Luis Benigno Gutierrez Zea
%   luis.gutierrez@upb.edu.co
%

function figureWithNameAndPosition(h,name,pos)

xini = 0.003;
yini = 0.06;
width = 0.661;
height = 0.843;
if isOctave()
  x = get(0,"screensize");
  h = figure(h,"name",name,"position",x([3,4,3,4]).*[xini+width*pos(1) yini+height*pos(2)  width*pos(3) height*pos(4)]);
else
  h = figure('Name',name,'Units','normalized','OuterPosition',pos);
end
