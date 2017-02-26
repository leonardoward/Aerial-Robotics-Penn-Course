function R = zyz_representation(u)
    %u=[u1 u2 u3]=[psi theta phi]=[?, ?, ?] 
    %Calculates the rotation matrix for a ZYZ euler angle representation,(?, ?, ?)
    A=[cos(u(1)) -sin(u(1)) 0; sin(u(1)) cos(u(1)) 0; 0 0 1];
    B=[cos(u(2)) 0 sin(u(2)); 0 1 0; -sin(u(2)) 0 cos(u(2))];
    C=[cos(u(3)) -sin(u(3)) 0; sin(u(3)) cos(u(3)) 0; 0 0 1];
    R=A*B*C;
    
end