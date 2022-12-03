close, clear all
clc;

% Dette scriptet viser eksempel på arbeids bevegelser som kan utføres av
% roboten vår. 

% Definere link lengths
d1 = 0.2; a1 = 1; a2 = 0.4; a3 = 0.6; a4 = 0.4; a5 = 0.2;

% Definerer joint angles
%q1 = 0; q2 = 0; q3 = 0; q4 = 0; q5 = 0; q6 = 0; %q7 = 0; 
qi = [0 0 0 0 0 0];

alpha1 = pi/2;
alpha2 = 0;
alpha3 = 0;
alpha4 = 0;
alpha5 = -pi/2;
alpha6 = 0;

% %Link/Joint specification
L(1) = Link([0 d1 0 alpha1]);
L(2) = Link([0 0 a1 alpha2]);
L(3) = Link([0 0 a2 alpha3]);
L(4) = Link([0 0 a3 alpha4]);
L(5) = Link([0 0 a4 alpha5]);
L(6) = Link([0 0 a5 alpha6]);

RA = SerialLink(L);
RA.name = 'PHUT T22';
RA.base = transl(0.85,0,0.602); %flytter arm base 
RA.tool = [0,0,1,1.2;0,-1,0,0;1,0,0,-0.2;0,0,0,1];
%RA.tool = trotx(0)*transl(1.2,0,0); %offset til GAFFEL ish center

t = 0:0.05:2;

% Startpos - preturn
q0 = [pi -pi/12 0 pi/12 0 0];
q1 = [pi pi/6 0 -pi/6 0 0];
Q1 = jtraj(q0,q1,t);

TRAJ1 = fkine(RA,Q1);
for i = 1:1:length(t)
    T1 = TRAJ1(i);
    trs1 = transl(T1);
    xx1(i) = trs1(1);
    yy1(i) = trs1(2);
    zz1(i) = trs1(3);
end

q2 = [0 pi/6 0 -pi/6 0 0];
Q2 = jtraj(q1,q2,t);

TRAJ2 = fkine(RA,Q2);
for i = 1:1:length(t)
    T2 = TRAJ2(i);
    trs2 = transl(T2);
    xx2(i) = trs2(1);
    yy2(i) = trs2(2);
    zz2(i) = trs2(3);
end

q3 = [0 -pi/3 -pi/3 pi/3 pi/3 0];
Q3 = jtraj(q2,q3,t);

TRAJ3 = fkine(RA,Q3);
for i = 1:1:length(t)
    T3 = TRAJ3(i);
    trs3 = transl(T3);
    xx3(i) = trs3(1);
    yy3(i) = trs3(2);
    zz3(i) = trs3(3);
end


q4 = [0 -pi/3 -pi/3 pi/3 5*pi/18 0];
Q4 = jtraj(q3,q4,t);

TRAJ4 = fkine(RA,Q4);
for i = 1:1:length(t)
    T4 = TRAJ4(i);
    trs4 = transl(T4);
    xx4(i) = trs4(1);
    yy4(i) = trs4(2);
    zz4(i) = trs4(3);
end

q5 = [0 pi/3 -pi/3 -pi/3 7*pi/18 0];
Q5 = jtraj(q4,q5,t);

TRAJ5 = fkine(RA,Q5);
for i = 1:1:length(t)
    T5 = TRAJ5(i);
    trs5 = transl(T5);
    xx5(i) = trs5(1);
    yy5(i) = trs5(2);
    zz5(i) = trs5(3);
end

q6 = [pi pi/6 0 -pi/6 0 0];
Q6 = jtraj(q5,q6,t);

TRAJ6 = fkine(RA,Q6);
for i = 1:1:length(t)
    T6 = TRAJ6(i);
    trs6 = transl(T6);
    xx6(i) = trs6(1);
    yy6(i) = trs6(2);
    zz6(i) = trs6(3);
end

q7 = [pi -pi/12 0 pi/12 0 0];
Q7 = jtraj(q6,q7,t);

TRAJ7 = fkine(RA,Q7);
for i = 1:1:length(t)
    T7 = TRAJ7(i);
    trs7 = transl(T7);
    xx7(i) = trs7(1);
    yy7(i) = trs7(2);
    zz7(i) = trs7(3);
end

q8 = [pi pi/6 0 -pi/6 0 0];
Q8 = jtraj(q7,q8,t);

TRAJ8 = fkine(RA,Q8);
for i = 1:1:length(t)
    T8 = TRAJ8(i);
    trs8 = transl(T8);
    xx8(i) = trs8(1);
    yy8(i) = trs8(2);
    zz8(i) = trs8(3);
end

q8 = [pi pi/6 0 -pi/6 0 0];
Q8 = jtraj(q7,q8,t);

TRAJ8 = fkine(RA,Q8);
for i = 1:1:length(t)
    T8 = TRAJ8(i);
    trs8 = transl(T8);
    xx8(i) = trs8(1);
    yy8(i) = trs8(2);
    zz8(i) = trs8(3);
end

q9 = [pi/2 pi/3 -pi/3 -pi/3 pi/3 0];
Q9 = jtraj(q8,q9,t);

TRAJ9 = fkine(RA,Q9);
for i = 1:1:length(t)
    T9 = TRAJ9(i);
    trs9 = transl(T9);
    xx9(i) = trs9(1);
    yy9(i) = trs9(2);
    zz9(i) = trs9(3);
end

q10 = [pi/2 pi/6 -pi/6 -pi/12 pi/12 0];
Q10 = jtraj(q9,q10,t);

TRAJ10 = fkine(RA,Q10);
for i = 1:1:length(t)
    T10 = TRAJ10(i);
    trs10 = transl(T10);
    xx10(i) = trs10(1);
    yy10(i) = trs10(2);
    zz10(i) = trs10(3);
end

q11 = [2*pi/9 pi/3 -pi/3 -pi/3 pi/3 -2*pi/9];
Q11 = jtraj(q10,q11,t);

TRAJ11 = fkine(RA,Q11);

for i = 1:1:length(t)
    T11 = TRAJ11(i);
    trs11 = transl(T11);
    xx11(i) = trs11(1);
    yy11(i) = trs11(2);
    zz11(i) = trs11(3);
end

q12 = [pi pi/6 0 -pi/6 0 0];
Q12 = jtraj(q11,q12,t);

TRAJ12 = fkine(RA,Q12);
for i = 1:1:length(t)
    T12 = TRAJ12(i);
    trs12 = transl(T12);
    xx12(i) = trs12(1);
    yy12(i) = trs12(2);
    zz12(i) = trs12(3);
end

q13 = [pi -pi/12 0 pi/12 0 0];
Q13 = jtraj(q12,q13,t);

TRAJ13 = fkine(RA,Q13);
for i = 1:1:length(t)
    T13 = TRAJ13(i);
    trs13 = transl(T13);
    xx13(i) = trs13(1);
    yy13(i) = trs13(2);
    zz13(i) = trs13(3);
end

hold on
% Tegner linjer av bevegelsene
% plot3(xx1,yy1,zz1,'Color',[1 0 0],'LineWidth',1);
% plot3(xx2,yy2,zz2,'Color',[1 0 0],'LineWidth',1);
% plot3(xx3,yy3,zz3,'Color',[1 0 0],'LineWidth',1);
% plot3(xx4,yy4,zz4,'Color',[1 0 0],'LineWidth',1);
% plot3(xx5,yy5,zz5,'Color',[1 0 0],'LineWidth',1);
% plot3(xx6,yy6,zz6,'Color',[1 0 0],'LineWidth',1);
% plot3(xx7,yy7,zz7,'Color',[1 0 0],'LineWidth',1);
% plot3(xx8,yy8,zz8,'Color',[1 0 0],'LineWidth',1);
% plot3(xx9,yy9,zz9,'Color',[1 0 0],'LineWidth',1);
% plot3(xx10,yy10,zz10,'Color',[1 0 0],'LineWidth',1);
plot(RA,Q1); plot(RA,Q2); plot(RA,Q3);
plot(RA,Q4); plot(RA,Q5); plot(RA,Q6);
plot(RA,Q7); plot(RA,Q8); plot(RA,Q9);
plot(RA,Q10); plot(RA,Q11); plot(RA,Q12);
plot(RA,Q13);
%plot(RA,Q11);