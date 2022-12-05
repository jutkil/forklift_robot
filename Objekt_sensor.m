%Dette skriptet skal beskrive leddvinkler som robotarmen må ha for å nå et
%gitt objekt detektert av kamerasystemet. 
clear all;
clc;

L(1) = Link ( [0 0.4 0 pi/2] );
L(2) = Link ( [0 0 1 0] );
L(3) = Link ( [0 0 0.4 0] );
L(4) = Link ( [0 0 0.6 0] );
L(5) = Link ( [0 0 0.4 -pi/2] );
L(6) = Link ( [0 0 0.2 0]);
RA = SerialLink(L);
RA.name = 'PHUT T22';
% RA.base = transl(0.85,0,0.602); %flytter arm base 

% Halverer lengden på gaflene for å simulere at vekten på lasten er i 
%  senter på end-effector
RA.tool = [0,0,1,1.2;0,-1,0,0;1,0,0,-0.2;0,0,0,1];

%Transformasjons matriser
rTc2 = [1,0,0,2.9; 0,1,0,0; 0,0,1,1.8; 0,0,0,1]; %Fra robotbase til kamera 2 

c2ToObject = [1,0,0,1; 0,1,0,0; 0,0,1,1; 0,0,0,1]; %Fra kamera 2 til object 

tool = [0,0,1,1.2;0,-1,0,0;1,0,0,-0.2;0,0,0,1];
toolToObject = inv(tool) * rTc2 * c2ToObject

%Beregner vinkler som robotarm trenger i forhold til objektet
qRobotarmToObject = RA.ikine(toolToObject, 'mask', [1 1 1 0 0 0])