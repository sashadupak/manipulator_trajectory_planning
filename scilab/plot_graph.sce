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

file_name = "sqr_traj.txt"
path = "/home/aleksandr/ITMO_lab2/ev3/lab3/X/"
res = read(path + file_name, -1, 13)
q(:, 1) = res(:, 1)
q(:, 2) = res(:, 5)
q(:, 3) = res(:, 9)
q1(:, 1) = res(:, 2)
q1(:, 2) = res(:, 6)
q1(:, 3) = res(:, 10)
t = res(:, 13)

a1=0
a2=0.12
a3=0.12
d1=0.14

for i = 1:length(t)
    [x(i), y(i), z(i)]= forward_kinematics(q(i, :)'*%pi/180)
    [x1(i), y1(i), z1(i)]= forward_kinematics(q1(i, :)'*%pi/180)
end

plot2d(t, y1, 0)
plot2d(t, y, 2)
xtitle('', 'Time, s', 'Y, m')
legend("Desired trajectory", "Model", "Experiment")
xs2png(0, "/home/aleksandr/ITMO_lab2/ev3/lab3/photos/2dek_3.png")
