function R = generic_rotation(u,phi)
    I=[1 0 0; 0 1 0; 0 0 1];
    R=I.*cos(phi)+u*u'.*(1-cos(phi))+skew(u)*sin(phi); 
end