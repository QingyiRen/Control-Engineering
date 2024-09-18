function [real,img] = poles_location(ratio,na_fre)
real=ratio*na_fre;
img=na_fre*sqrt(1-ratio*ratio);
end