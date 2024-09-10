function dx = pend_cart(x,I,m,M,L,g,d,F)
    %X1 = x ; cart position
    %X2 = x_dot ; cart velocity
    %X3 = theta; pend angular
    %X4 = theta_dot; pend angular velocity

    theta = x(3);
    sx = sin(theta);
    cx = cos(theta);
    C = (M+m)*(I+m*L^2)-m^2*L^2*cx^2;

    dx(1,1)= x(2);
    dx(2,1) = (1/C)*(-(I+m*L^2)*d*x(2) + m*L*(I+m*L^2)*sx*x(4)^2 + (I + m*L^2)*F-m^2*L^2*g*cx*sx);
    dx(3,1) = x(4);
    dx(4,1) = (1/C)*(m*L*cx*d*x(2)+m^2*L^2*sx*cx*x(4)^2 - m*L*cx*F + (M+m)*m*g*L*sx);
