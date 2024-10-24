% function to check wether the m code is running in Octave
% Usage:
%   y = isOctave()
% where
%   y : y=1 if it is on octave, y=0 otherwise 
%
% Author:
%   Luis Benigno Gutierrez Zea
%   luis.gutierrez@upb.edu.co
%

function y = isOctave()

y = exist('OCTAVE_VERSION','builtin')==5;
