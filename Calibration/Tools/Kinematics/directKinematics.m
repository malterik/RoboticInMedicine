% should be replaced by your own direct kinematics!

function [ Pose ] = directKinematics( q )
% direct kinematics for UR5
DH.a=[0,-0.4250,-0.39225,0,0,0];
DH.d=[0.089159, 0 , 0 , 0.10915, 0.09465, 0.0823];
DH.alpha=[pi/2, 0,0, pi/2, -pi/2, 0];
DH.q=q;

A=eye(4);
for i=1:6
    Tz=eye(4);
    Tz(3,4)=DH.d(i);
    Rz=eye(4);
    Rz(1:2,1:2)=[cos(DH.q(i)),-sin(DH.q(i));sin(DH.q(i)),cos(DH.q(i))];
    Tx=eye(4);
    Tx(1,4)=DH.a(i);
    Rx=eye(4);
    Rx(2:3,2:3)=[cos(DH.alpha(i)),-sin(DH.alpha(i));sin(DH.alpha(i)),cos(DH.alpha(i))];
    A=A*Tz*Rz*Tx*Rx;

end
Pose=A;

end
