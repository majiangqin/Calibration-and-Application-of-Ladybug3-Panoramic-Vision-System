
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 相机检校中的误差方程式系数和常数项
% 误差方程加入控制点改正
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [B,L,C,W2] = calibration_SetErrorEquation2(x,y,X,Y,Z,Xs,Ys,Zs,a1,a2,a3,b1,b2,b3,c1,c2,c3,x0,y0,fx,fy,k1,k2,p1,p2)

[number,aaaaa] = size(X);


    for i=1:number
        XX = a1 * (X(i)-Xs) + b1 * (Y(i)-Ys) + c1 * (Z(i)-Zs);
        YY = a2 * (X(i)-Xs) + b2 * (Y(i)-Ys) + c2 * (Z(i)-Zs);
        ZZ = a3 * (X(i)-Xs) + b3 * (Y(i)-Ys) + c3 * (Z(i)-Zs);
        r_pingfang = (x(i)-x0)^2 + (y(i)-y0)^2;
        deta_x = (x(i) - x0) * (k1 * r_pingfang + k2 * (r_pingfang^2)) + p1 * (r_pingfang + 2 * ((x(i) - x0)^2)) + 2 * p2 * (x(i) - x0) * (y(i) - y0);
        deta_y = (y(i) - y0) * (k1 * r_pingfang + k2 * (r_pingfang^2)) + p2 * (r_pingfang + 2 * ((y(i) - y0)^2)) + 2 * p1 * (x(i) - x0) * (y(i) - y0);

% p134 V1 = Ac * t + Bc * Xc - L1 <近景摄影测量>
        B(2*(i-1)+1,1) = (a1 * fx + a3 * (x(i)-x0 - deta_x)) / ZZ;
        B(2*(i-1)+1,2) = (b1 * fx + b3 * (x(i)-x0 - deta_x)) / ZZ;
        B(2*(i-1)+1,3) = (c1 * fx + c3 * (x(i)-x0 - deta_x)) / ZZ;
        B(2*(i-1)+1,4) = -fx * (X(i)-Xs) / ZZ;
        B(2*(i-1)+1,5) = 0;
        B(2*(i-1)+1,6) = -(x(i)-x0 - deta_x) * (X(i)-Xs) / ZZ;
        B(2*(i-1)+1,7) = -fx * (Y(i)-Ys) / ZZ;
        B(2*(i-1)+1,8) = 0;
        B(2*(i-1)+1,9) = -(x(i)-x0 - deta_x) * (Y(i)-Ys) / ZZ;
        B(2*(i-1)+1,10) = -fx * (Z(i)-Zs) / ZZ;
        B(2*(i-1)+1,11) = 0;
        B(2*(i-1)+1,12) = -(x(i)-x0 - deta_x) * (Z(i)-Zs) / ZZ;
        B(2*(i-1)+1,13) = 1 - k1 * r_pingfang - k2 * r_pingfang^2 - (x(i)-x0) * (2 * k1 * (x(i) - x0) + 4 * k2 * (x(i)-x0)^3 + 4 * k2 * (x(i)-x0) * (y(i)-y0)^2) - 6 * p1 * (x(i)-x0) - 2 * p2 * (y(i)-y0);
        B(2*(i-1)+1,14) = -(x(i)-x0) * (2 * k1 * (y(i) - y0) + 4 * k2 * (y(i)-y0)^3 + 4 * k2 * (y(i)-y0) * (x(i)-x0)^2) - 2 * p1 * (y(i)-y0) - 2 * p2 * (x(i)-x0);
        B(2*(i-1)+1,15) = (x(i) - x0 - deta_x) / fx;  
        B(2*(i-1)+1,16) = 0;
        B(2*(i-1)+1,17) = (x(i)-x0) * (r_pingfang);
        B(2*(i-1)+1,18) = (x(i)-x0) * (r_pingfang^2);
        B(2*(i-1)+1,19) = 3 * (x(i)-x0)^2 + (y(i)-y0)^2;
        B(2*(i-1)+1,20) = 2 * (x(i)-x0) * (y(i)-y0);  
        
%         B(2*(i-1)+1,20+3*(i-1)+1) = -B(2*(i-1)+1,1);
%         B(2*(i-1)+1,20+3*(i-1)+2) = -B(2*(i-1)+1,2);
%         B(2*(i-1)+1,20+3*(i-1)+3) = -B(2*(i-1)+1,3);

        B(2*(i-1)+2,1) = (a2 * fy + a3 * (y(i)-y0 - deta_y)) / ZZ;
        B(2*(i-1)+2,2) = (b2 * fy + b3 * (y(i)-y0 - deta_y)) / ZZ;
        B(2*(i-1)+2,3) = (c2 * fy + c3 * (y(i)-y0 - deta_y)) / ZZ;
        B(2*(i-1)+2,4) = 0;
        B(2*(i-1)+2,5) = -fy * (X(i)-Xs) / ZZ;
        B(2*(i-1)+2,6) = -(y(i)-y0 - deta_y) * (X(i)-Xs) / ZZ;
        B(2*(i-1)+2,7) = 0;
        B(2*(i-1)+2,8) = -fy * (Y(i)-Ys) / ZZ;
        B(2*(i-1)+2,9) = -(y(i)-y0 - deta_y) * (Y(i)-Ys) / ZZ;
        B(2*(i-1)+2,10) = 0;
        B(2*(i-1)+2,11) = -fy * (Z(i)-Zs) / ZZ;
        B(2*(i-1)+2,12) = -(y(i)-y0 - deta_y) * (Z(i)-Zs) / ZZ;
        B(2*(i-1)+2,13) = -(y(i)-y0) * (2 * k1 * (x(i) - x0) + 4 * k2 * (x(i)-x0)^3 + 4 * k2 * (x(i)-x0) * (y(i)-y0)^2) - 2 * p2 * (x(i)-x0) - 2 * p1 * (y(i)-y0);;
        B(2*(i-1)+2,14) = 1 - k1 * r_pingfang - k2 * r_pingfang^2 - (y(i)-y0) * (2 * k1 * (y(i) - y0) + 4 * k2 * (y(i)-y0)^3 + 4 * k2 * (y(i)-y0) * (x(i)-x0)^2) - 6 * p2 * (y(i)-y0) - 2 * p1 * (x(i)-x0);;
        B(2*(i-1)+2,15) = 0; 
        B(2*(i-1)+2,16) = (y(i) - y0 - deta_y) / fy;
        B(2*(i-1)+2,17) = (y(i)-y0) * r_pingfang;
        B(2*(i-1)+2,18) = (y(i)-y0) * r_pingfang^2;
        B(2*(i-1)+2,19) = 2 * (x(i)-x0) * (y(i)-y0);
        B(2*(i-1)+2,20) = 3 * (y(i)-y0)^2 + (x(i)-x0)^2;   
        
%         B(2*(i-1)+2,20+3*(i-1)+1) = -B(2*(i-1)+2,1);
%         B(2*(i-1)+2,20+3*(i-1)+2) = -B(2*(i-1)+2,2);
%         B(2*(i-1)+2,20+3*(i-1)+3) = -B(2*(i-1)+2,3);
      
        L(2*(i-1)+1,1) = x(i) - x0 - deta_x + fx * XX / ZZ; 
        L(2*(i-1)+2,1) = y(i) - y0 - deta_y + fy * YY / ZZ;
    end

   
   C = [0 0 0  2*a1  2*a2  2*a3     0     0     0     0     0     0
         0 0 0     0     0     0  2*b1  2*b2  2*b3     0     0     0
         0 0 0     0     0     0     0     0     0  2*c1  2*c2  2*c3
         0 0 0    a2    a1     0    b2    b1     0    c2    c1     0
         0 0 0    a3     0    a1    b3     0    b1    c3     0    c1
         0 0 0     0    a3    a2     0    b3    b2     0    c3    c2];
    W2 = -[a1^2 + a2^2 + a3^2 - 1
           b1^2 + b2^2 + b3^2 - 1
           c1^2 + c2^2 + c3^2 - 1 
           a1 * a2 + b1 * b2 + c1 * c2 
           a1 * a3 + b1 * b3 + c1 * c3 
           a2 * a3 + b2 * b3 + c2 * c3];

        
