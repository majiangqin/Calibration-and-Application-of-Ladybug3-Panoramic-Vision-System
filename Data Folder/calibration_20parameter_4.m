% 相机检校 calibration
% (ai,bi,ci)
% V = BX - L
% CX + W = 0      附有限制条件的间接平差
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 控制点--真值 
% 多片
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% p198   冯文灏书<近景摄影测量>


clear;

% % 像点坐标: pixel
% 第1张像片
fp=fopen('D:\课题\40mm_zuobiao\2-40-g11-k1-006像点坐标.TXT','r');
[A,count]=fscanf(fp,'%f',[3,187]);
fclose(fp);
x1 = A';
y1 = x1(:,3);
y1 = 2004 - y1;
point_number_xiang =  x1(:,1);
x1(:,3)=[]; x1(:,1)=[];
x1 = x1 - 2672;
[number_xiang1,aaaaa] = size(x1);

% 第2张像片
fp=fopen('D:\课题\40mm_zuobiao\2-40-g11-k1-005像点坐标.TXT','r');
[A,count]=fscanf(fp,'%f',[3,187]);
fclose(fp);
x2 = A';
y2 = x2(:,3);
y2 = 2004 - y2;
point_number_xiang =  x2(:,1);
x2(:,3)=[]; x2(:,1)=[];
x2 = x2 - 2672;
[number_xiang2,aaaaa] = size(x2);

% 第3张像片
fp=fopen('D:\课题\40mm_zuobiao\2-40-g11-k1-003像点坐标.TXT','r');
[A,count]=fscanf(fp,'%f',[3,187]);
fclose(fp);
x3 = A';
y3 = x3(:,3);
y3 = 2004 - y3;
point_number_xiang =  x3(:,1);
x3(:,3)=[]; x3(:,1)=[];
x3 = x3 - 2672;
[number_xiang3,aaaaa] = size(x3);

% 第4张像片
fp=fopen('D:\课题\40mm_zuobiao\2-40-g11-k1-001像点坐标.TXT','r');
[A,count]=fscanf(fp,'%f',[3,187]);
fclose(fp);
x4 = A';
y4 = x4(:,3);
y4 = 2004 - y4;
point_number_xiang =  x4(:,1);
x4(:,3)=[]; x4(:,1)=[];
x4 = x4 - 2672;
[number_xiang4,aaaaa] = size(x4);

% 控制点坐标
fp=fopen('D:\课题\40mm_zuobiao\2-40-g11-k1-001物方坐标.txt','r');
[A,count]=fscanf(fp,'%f',[4,inf]);
fclose(fp);
X = A';
Z = X(:,4);
Y = X(:,3);
point_number_wu =  X(:,1);
X(:,4)=[]; X(:,3)=[];X(:,1)=[]; 
[number,aaaaa] = size(X);
% 控制点坐标2
fp=fopen('D:\课题\40mm_zuobiao\ttt.txt','r');
[At,count]=fscanf(fp,'%f',[4,inf]);
fclose(fp);
AAt = At';
Xt = AAt(:,2);
Zt = AAt(:,4);
Yt = AAt(:,3);



% 初值
x0 = 0;  y0 = 0;
fx = 4000;  fy = 4000;
k1 = 0; k2 = 0; p1 = 0; p2 = 0;

% 第1张像片
% wu_Point_1 = [X(1) Y(1) Z(1)];
% wu_Point_2 = [X(2) Y(2) Z(2)];
% xiang_point_1 = [x1(1) y1(1)];
% xiang_point_2 = [x1(2) y1(2)];
% m = dist(wu_Point_1,wu_Point_2') / dist(xiang_point_1,xiang_point_2');
Xs1 = mean(Xt); Ys1 = mean(Yt); 
Zs1 = mean(Zt); %+ fx * m

a11 = 1;  a12 = 0;  a13 = 0;
b11 = 0;  b12 = 1;  b13 = 0;
c11 = 0;  c12 = 0;  c13 = 1;

% 第2张像片
Xs2 = Xs1; Xs3 = Xs1; Xs4 = Xs1;
Ys2 = Ys1; Ys3 = Ys1; Ys4 = Ys1;
Zs2 = Zs1; Zs3 = Zs1; Zs4 = Zs1; 
 
a21 = 1;  a22 = 0;  a23 = 0;
b21 = 0;  b22 = 1;  b23 = 0;
c21 = 0;  c22 = 0;  c23 = 1;

% 第3张像片

a31 = 1;  a32 = 0;  a33 = 0;
b31 = 0;  b32 = 1;  b33 = 0;
c31 = 0;  c32 = 0;  c33 = 1;

% 第4张像片

a41 = 1;  a42 = 0;  a43 = 0;
b41 = 0;  b42 = 1;  b43 = 0;
c41 = 0;  c42 = 0;  c43 = 1;

%迭代次数 ip
ip = 0;
dX = zeros(56,1);
while max(abs(dX)) > 0.0001 | ip == 0 
    ip = ip + 1;

    % 列误差方程式
    [B1,L1,C1,W1] = calibration_SetErrorEquation2(x1,y1,X,Y,Z,Xs1,Ys1,Zs1,a11,a12,a13,b11,b12,b13,c11,c12,c13,x0,y0,fx,fy,k1,k2,p1,p2);
%     [B2,L2,C2,W2] = calibration_SetErrorEquation2(x2,y2,X,Y,Z,Xs2,Ys2,Zs2,a21,a22,a23,b21,b22,b23,c21,c22,c23,x0,y0,fx,fy,k1,k2,p1,p2);
%     [B3,L3,C3,W3] = calibration_SetErrorEquation2(x3,y3,X,Y,Z,Xs3,Ys3,Zs3,a31,a32,a33,b31,b32,b33,c31,c32,c33,x0,y0,fx,fy,k1,k2,p1,p2);
%     [B4,L4,C4,W4] = calibration_SetErrorEquation2(x4,y4,X,Y,Z,Xs4,Ys4,Zs4,a41,a42,a43,b41,b42,b43,c41,c42,c43,x0,y0,fx,fy,k1,k2,p1,p2);

% 合成大的系数阵
% 56个未知数  Xs1,Ys1,Zs1,a11,a12,a13,b11,b12,b13,c11,c12,c13,
%             Xs2,Ys2,Zs2,a21,a22,a23,b21,b22,b23,c21,c22,c23
%             Xs3,Ys3,Zs3,a31,a32,a33,b31,b32,b33,c31,c32,c33
%             Xs4,Ys4,Zs4,a41,a42,a43,b41,b42,b43,c41,c42,c43
%             x0,y0,fx,fy,k1,k2,p1,p2
    
    
    if ip == 1
        U1 = zeros(2*number,12);
    end
%     B1_1 = B1(:,1:12);      B1_2 = B1(:,13:20);
%     B2_1 = B2(:,1:12);      B2_2 = B2(:,13:20);
%     B3_1 = B3(:,1:12);      B3_2 = B3(:,13:20);
% 
%     B1_new = [B1_1 U1 U1 U1 B1_2];
%     B2_new = [U1 B2_1 U1 U1 B2_2];
%     B3_new = [U1 U1 B3_1 U1 B3_2];
%     B4_new = [U1 U1 U1 B4];
%     B = [B1_new;B2_new;B3_new;B4_new];
%     L = [L1;L2;L3;L4];
%     
    if ip == 1
        U2 = zeros(6,12);  U3 = zeros(6,8);
    end
%     C1_new = [C1 U2 U2 U2 U3];
%     C2_new = [U2 C2 U2 U2 U3];
%     C3_new = [U2 U2 C3 U2 U3];
%     C4_new = [U2 U2 U2 C4 U3];
%     C = [C1_new;C2_new;C3_new;C4_new];
%     WW = [W1;W2;W3;W4];

    B = B1; C = [C1 U3]; WW = W1; L =L1;
    xxxx=B(1:8,:);
    
    N1 = B' * B;
    M1 = [N1;C];
    N2 = C'; N3 = zeros(6,6);  M2 = [N2;N3];
%     N2 = C'; N3 = zeros(24,24);  M2 = [N2;N3];
    M = [M1 M2];
    WL1 = B' * L;
    W = [WL1; WW];
    dX = givens(M,W);
    dX(21:26)=[];
%     dX(57:80)=[];

    V = B * dX - L;
k = 0;
    k = k + 1;
    Xs1 = Xs1 + dX(k,1);   k = k + 1;    Ys1 = Ys1 + dX(k,1);   k = k + 1;    Zs1 = Zs1 + dX(k,1);    k = k + 1;
    a11 = a11 + dX(k,1);   k = k + 1;    a12 = a12 + dX(k,1);   k = k + 1;    a13 = a13 + dX(k,1);    k = k + 1;
    b11 = b11 + dX(k,1);   k = k + 1;    b12 = b12 + dX(k,1);   k = k + 1;    b13 = b13 + dX(k,1);    k = k + 1;
    c11 = c11 + dX(k,1);   k = k + 1;    c12 = c12 + dX(k,1);   k = k + 1;    c13 = c13 + dX(k,1);    k = k + 1;
%     Xs2 = Xs2 + dX(k,1);   k = k + 1;    Ys2 = Ys2 + dX(k,1);   k = k + 1;    Zs2 = Zs2 + dX(k,1);    k = k + 1;
%     a21 = a21 + dX(k,1);   k = k + 1;    a22 = a22 + dX(k,1);   k = k + 1;    a23 = a23 + dX(k,1);    k = k + 1;
%     b21 = b21 + dX(k,1);   k = k + 1;    b22 = b22 + dX(k,1);   k = k + 1;    b23 = b23 + dX(k,1);    k = k + 1;
%     c21 = c21 + dX(k,1);   k = k + 1;    c22 = c22 + dX(k,1);   k = k + 1;    c23 = c23 + dX(k,1);    k = k + 1;
%     Xs3 = Xs3 + dX(k,1);   k = k + 1;    Ys3 = Ys3 + dX(k,1);   k = k + 1;    Zs3 = Zs3 + dX(k,1);    k = k + 1;
%     a31 = a31 + dX(k,1);   k = k + 1;    a32 = a32 + dX(k,1);   k = k + 1;    a33 = a33 + dX(k,1);    k = k + 1;
%     b31 = b31 + dX(k,1);   k = k + 1;    b32 = b32 + dX(k,1);   k = k + 1;    b33 = b33 + dX(k,1);    k = k + 1;
%     c31 = c31 + dX(k,1);   k = k + 1;    c32 = c32 + dX(k,1);   k = k + 1;    c33 = c33 + dX(k,1);    k = k + 1;
%     Xs4 = Xs4 + dX(k,1);   k = k + 1;    Ys4 = Ys4 + dX(k,1);   k = k + 1;    Zs4 = Zs4 + dX(k,1);    k = k + 1;
%     a41 = a41 + dX(k,1);   k = k + 1;    a42 = a42 + dX(k,1);   k = k + 1;    a43 = a43 + dX(k,1);    k = k + 1;
%     b41 = b41 + dX(k,1);   k = k + 1;    b42 = b42 + dX(k,1);   k = k + 1;    b43 = b43 + dX(k,1);    k = k + 1;
%     c41 = c41 + dX(k,1);   k = k + 1;    c42 = c42 + dX(k,1);   k = k + 1;    c43 = c43 + dX(k,1);    k = k + 1;
    x0 = x0 + dX(k,1);     k = k + 1;    y0 = y0 + dX(k,1);     k = k + 1;   
    fx = fx + dX(k,1);     k = k + 1;    fy = fy + dX(k,1);     k = k + 1;
    k1 = k1 + dX(k,1);     k = k + 1;    k2 = k2 + dX(k,1);     k = k + 1;
    p1 = p1 + dX(k,1);     k = k + 1;    p2 = p2 + dX(k,1);     k = k + 1;
end

V = B * dX - L;
pvv = V'*V;
 vmax = max(V);
 vmin = min(V);
% S = [Xs;Ys;Zs];
N = [x0;y0;fx;fy];
% % R =[a1 a2 a3
% %     b1 b2 b3
% %     c1 c2 c3];
% % fai = atan(-a3/c3);
% % omiga = asin(-b3);
% % kaba = atan(b1/b2);
% % jiaoyuansu = [fai; omiga; kaba];   
jibiancanshu = [k1;k2;p1;p2];
% 
% fp=fopen('C:\xf\shanghai\40mm-zuobiao\calib_result_92点_4片.txt','w');
% fprintf(fp,'x0 = %f\ny0 = %f\nfx = %f\nfy = %f\n',N);
% fprintf(fp,'k1 = %f\nk2 = %f\np1 = %f\np2 = %f\n',jibiancanshu);
% % fprintf(fp,'%f\n',V);
% fclose(fp);