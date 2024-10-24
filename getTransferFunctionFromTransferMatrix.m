% getTransferFunctionFromTransferMatrix.m
% function to get transfer function from transfer matrix_type
% Usage:
%   [transferFunction] = getTransferFunctionFromTransferMatrix(modeltf,in,out,inputSign)
% where:
%   modeltf : transfer matrix model
%   in : input number
%   out : output number
%   inputSign : input sign
%   transferFunction : transfer function from input in (with sign inputSign) to output out
%
% Author:
%   Luis Benigno Gutierrez Zea
%   luis.gutierrez@upb.edu.co
%

function [transferFunction] = getTransferFunctionFromTransferMatrix(modeltf,in,out,inputSign)
  [num,den] = tfdata(modeltf);
  if isOctave()
    if inputSign==1
      transferFunction = tf(num{out,in},den{out,in},'inname',{modeltf.inname{in,1}},'outname',{modeltf.outname{out,1}});
    elseif inputSign==-1
      transferFunction = tf(-num{out,in},den{out,in},'inname',{['-',modeltf.inname{in,1}]},'outname',{modeltf.outname{out,1}});
    end
  else
    if inputSign==1
      transferFunction = tf(num{out,in},den{out,in},'InputName',{modeltf.InputName{in,1}},'OutputName',{modeltf.OutputName{out,1}});
    elseif inputSign==-1
      transferFunction = tf(-num{out,in},den{out,in},'InputName',{['-',modeltf.InputName{in,1}]},'OutputName',{modeltf.OutputName{out,1}});
    end
  end
end
