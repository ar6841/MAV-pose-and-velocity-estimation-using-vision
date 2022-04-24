function [position, orientation, R_c2w] = estimatePose(data, t)
%% Parameters
K=[311.0520        0        201.8724;0         311.3885    113.6210;0            0           1   ];
%% A matrix calculation
fns=fieldnames(data);
A=double.empty(0,9);
for i=1:length(data(t).id)
    
    p_w=getCorner(data(t).id(1,i));
    
    for j=1:5

        a=[p_w(2*j-1),  p_w(2*j),   1,  0,  0,  0,  -data(t).(fns{j+7})(1,i)*p_w(2*j-1),    -data(t).(fns{j+7})(1,i)*p_w(2*j),  -data(t).(fns{j+7})(1,i);   0,  0,  0,  p_w(2*j-1), p_w(2*j),   1,  -data(t).(fns{j+7})(2,i)*p_w(2*j-1),    -data(t).(fns{j+7})(2,i)*p_w(2*j),  -data(t).(fns{j+7})(2,i)];
        %data(t).(fns{j+7})(1,i)
        A=vertcat(A,a);
    end  
end
%% Calculating homography matrix
[U,S,V]=svd(A);
clear A;
h=transpose(V(:,9));
HL=vertcat(h(1,1:3),h(1,4:6),h(1,7:9));
%H=vertcat(h(1,1:3),h(1,4:6),h(1,7:9));

S=svd(HL);
%% H matrix Sign correction
H=HL/S(2);
x1=vertcat(data(t).p0(:,1),1);
x=getCorner(data(t).id(1));
x2=vertcat(x(1),x(2),1);
sign=transpose(x1)*H*x2;
if(sign<0) % Sign correction
    H=-1*H;
end
   
%% Estimating orientation and posistion cam_world
RT=K\H; % '\' does K(inv) *H
R1=RT(:,1);
R2=RT(:,2);

T_temp=RT(:,3);
T=T_temp/norm(R1);

R_temp=horzcat(R1,R2,cross(R1,R2));
[U,S,V]=svd(R_temp);
Vt=transpose(V);
R_c2w=U*[1, 0,  0;  0,  1,  0;  0,  0,  det(U*Vt)]*Vt;


%% Homogeneous Transformation to world frame
%R_body_cam=rotx(180)*rotz(45);
R_body_cam=[0.7071, -0.7071, 0;  -0.7071, -0.7071,    0; 0,   0,  -1.0000];

R_cam_body=transpose(R_body_cam);
T_cam_body=[-0.04;0;-0.03];
H_cam_body=vertcat(horzcat(R_cam_body,T_cam_body),[0,    0,  0,  1]);

H_cam_world=vertcat(horzcat(R_c2w,T),[0,    0,  0,  1]);

H_world_body=H_cam_world\H_cam_body; %H_cam_world(inv)*H_cam_body
T_world_body=H_world_body(1:3,4);
R_body_world=R_body_cam*R_c2w;
R_world_body=transpose(R_body_world);

%% Outputs

position=T_world_body;

orientation(3)=atan2(R_world_body(2,1),R_world_body(1,1));
orientation(2)=atan2(-R_world_body(3,1),sqrt((R_world_body(1,1))^2+(R_world_body(2,1))^2));
orientation(1)=atan2(R_world_body(3,2),R_world_body(3,3));
end