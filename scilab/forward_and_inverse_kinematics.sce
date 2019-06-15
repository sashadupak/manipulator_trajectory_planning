clear()

a1=0
a2=0.12
a3=0.12
d1=0.14

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

function q = inverse_kinematics(x, y, z)
    r1=sqrt(x^2+y^2)
    r3=sqrt(x^2+y^2+(z-d1)^2)
    if r3 > a2 + a3
        return [0; 0; 0]
    end
    if x == 0
        if y == 0
            q(1) = 0
        elseif y > 0
            q(1) = %pi/2
        else
            q(1) = -%pi/2
        end
    elseif y == 0
        if x > 0
            q(1) = 0
        else
            q(1) = %pi
        end
    else
        if x>0
            if y>0
                q(1) = atan(y/x)
            else
                q(1) = atan(y/x)
            end
        else
            if y>0
                q(1) = %pi+atan(y/x)
            else
                q(1) = -%pi+atan(y/x)
            end
        end
    end
    if z - d1 == 0
        q(2) = %pi/2 - acos((a2**2+r3**2-a3**2)/(2*a2*r3))
    elseif z - d1 > 0
        q(2) = atan(r1/(z-d1)) - acos((a2**2+r3**2-a3**2)/(2*a2*r3))
    else
        q(2) = %pi + atan(r1/(z-d1)) - acos((a2**2+r3**2-a3**2)/(2*a2*r3))
    end
    q(3) = %pi - acos((a2**2+a3**2-r3**2)/(2*a2*a3))
endfunction
