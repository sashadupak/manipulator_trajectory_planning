clear()

function [x, y, z] = forward_kinematics(q)
    T1=[cos(q(1)-%pi),0,sin(q(1)-%pi),a1*cos(q(1)-%pi);
        sin(q(1)-%pi),0,-cos(q(1)-%pi),a1*sin(q(1)-%pi);
        0,1,0,d1;
        0,0,0,1]
    T2=[cos(q(2)+%pi/2),-sin(q(2)+%pi/2),0,a2*cos(q(2)+%pi/2);
        sin(q(2)+%pi/2),cos(q(2)+%pi/2),0,a2*sin(q(2)+%pi/2);
        0,0,1,0;
        0,0,0,1]
    T3=[cos(q(3)),-sin(q(3)),0,a3*cos(q(3));
        sin(q(3)),cos(q(3)),0,a3*sin(q(3));
        0,0,1,0;
        0,0,0,1]
    T = T1*T2*T3
    x = T(1,4)
    y = T(2,4)
    z = T(3,4)
endfunction


file_name = "pi_traj.txt"
path = "/home/aleksandr/ITMO_lab2/ev3/lab3/X/"
res = read(path + file_name, -1, 13)
ex(:, 1) = res(:, 1)
ex(:, 2) = res(:, 5)
ex(:, 3) = res(:, 9)
time = res(:, 13)

d1=0.14
a2=0.12
a3=0.12
a1=0

for i = 1:length(time)
    q(1) = ex(i, 1)*%pi/180
    q(2) = ex(i, 2)*%pi/180
    q(3) = ex(i, 3)*%pi/180
        [x2(i),y2(i),z2(i)]=forward_kinematics(q)
        D1(i)=d1
        x1(i) = -a2*cos(q(1))*sin(-q(2))
        y1(i) = -a2*sin(q(1))*sin(-q(2))
        z1(i) = a2*cos(q(2)) + d1
end

zer = zeros(length(x1), 1)
X1=[0, 0]
Y1=[0, 0]
Z1=[0, d1]
X2=[zer'; x1']
Y2=[zer'; y1']
Z2=[D1'; z1']
X3=[x1'; x2']
Y3=[y1'; y2']
Z3=[z1'; z2']
f1=figure()
f1.background=8
f1.children.thickness=3
xsegs(X1, Y1, Z1)
xsegs(X2(:, length(x1)), Y2(:, length(x1)), Z2(:, length(x1)))
xsegs(X3(:, length(x1)), Y3(:, length(x1)), Z3(:, length(x1)))
f1.children.thickness=1
comet3d(x2, y2, z2)
