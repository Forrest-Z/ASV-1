% function to perform los algorithm 

close all;
clear;

M=[25.8 0 0;
    0 33.8 1;
    0 1 2.76];
D=[2 0 0;
    0 7 0.1;
    0 0.1 0.5];

V=[1 2 3];
C=computeCb(M(1,1),M(2,2),M(2,3),V);


function C=computeCb(m11,m22,m23,velocity)
C=zeros(3);
C(1,3)=-m22*velocity(2)-m23*velocity(3);
C(3,1)=m22*velocity(2)+m23*velocity(3);
C(2,3)=m11*velocity(1);
C(3,2)=-m11*velocity(1);
end