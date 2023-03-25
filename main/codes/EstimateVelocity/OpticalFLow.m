close all;
clear all;
clc;

addpath('../../data')

datasetNum = 1;

[sampledData, sampledVicon, sampledTime] = init(datasetNum);

%% INITIALIZE CAMERA MATRIX AND OTHER NEEDED INFORMATION

N = length(sampledData);

estimatedV=zeros(6,N-1);

K=[311.0520        0        201.8724;0         311.3885    113.6210;0            0           1   ];
T_cam_body=[-0.04;0;-0.03];

%% Ransac Parameters
ransac_flag=true; %Change this to enable or disable RANSAC

e=8/10; %Ransac hyper parameter(Assume P_success=0.99)

cutoff_metric=120; % Increase this to reduce time but also reduce accuracy

%% Calculate dt (Vectorized)
i=2:N;
del=sampledData(i);  
dt=[del(1:N-1).t]-[sampledData(1:(N-1)).t];
dt=lowpass(dt,0.5,1000);

for n = 2:N
 %% Detecting points in image n-1
    
    points1=detectFASTFeatures(sampledData(n-1).img); 
    points1=points1(points1.Metric>cutoff_metric); % Detect points with strongest features

    point_tracker=vision.PointTracker();
    initialize(point_tracker,points1.Location,sampledData(n-1).img); %Initialize KLT algorithm for image n-1

%% Detecting matching features in image n
    [points2,validity,s]=point_tracker(sampledData(n).img);

%% Normalize image coordinates
    matched_points1=K\(vertcat(transpose(points1.Location),ones(1,length(points1.Location))));
    matched_points2=K\(vertcat(transpose(points2),ones(1,length(points1.Location))));
   
 %% Calculate optical flow
    opt_flow=matched_points2-matched_points1;
    opt_flow=opt_flow(1:2,:)/dt(n-1);


%% Alternative method for feature tracking commented out (ignore)    
%     [features1,valid_points1]=extractFeatures(sampledData(n-1).img,points1);
%     [features2,valid_points2]=extractFeatures(sampledData(n).img,points2);
% 
%     index_pairs=matchFeatures(features1,features2);
%     matched_points1=valid_points1(index_pairs(:,1),:).Location;
%     matched_points2=valid_points2(index_pairs(:,2),:).Location;
%     figure; 
%     showMatchedFeatures(sampledData(n-1).img,sampledData(n).img,matched_points1,matched_points2);
% 
%     matched_points1=K\vertcat(transpose(valid_points1(index_pairs(:,1),:).Location),ones(1,length(index_pairs)));
%     matched_points2=K\vertcat(transpose(valid_points2(index_pairs(:,2),:).Location),ones(1,length(index_pairs)));

%% Calculate vector going from camera frame origin to world frame origin
    [position_body,   orientation_body,    R_c2w]=estimatePose(sampledData,n-1);
    
    position_cam=-R_c2w*(position_body-(R_c2w\T_cam_body)); 
   
    
    if(ransac_flag==false)

        H=zeros(2*length(matched_points1),6);

        A=double.empty(0,3);
        B=double.empty(0,3);

        for i=1:length(matched_points1)
            %% Calculate Zc for each matched point (Depth of points)
            G=[R_c2w(1,1),  R_c2w(1,2), -matched_points1(1,i);  R_c2w(2,1), R_c2w(2,2), -matched_points1(2,i);  R_c2w(3,1), R_c2w(3,2), -1];

            Lambda_solve = G\(-position_cam);
            Z = Lambda_solve(3); 
       
            %% Compute H matrix
            A=(1/Z)*[-1,    0,  matched_points1(1,i);   0,  -1, matched_points1(2,i)];
            B=[matched_points1(1,i)*matched_points1(2,i),   -(1+(matched_points1(1,i)^2)),    matched_points1(2,i);   1+(matched_points1(2,i)^2), -matched_points1(1,i)*matched_points1(2,i), -matched_points1(1,i)];
            H((2*i-1):2*i,:)=horzcat(A,B);
            
        end        
%% Reshape so matrix dimentions match
        %Ht=transpose(H);
        p_dot=reshape(opt_flow,[],1);

%% Compute Velocity and angular velocity of camera expressed in camera frame
        Vel_cam=((H'*H)\H')*p_dot;
    
    else
        %% Computing Vel using RANSAC

        for j=1:length(matched_points1)

            %% Passing Depth of corner

            G = [R_c2w(1,1),  R_c2w(1,2), -matched_points1(1,j);  R_c2w(2,1), R_c2w(2,2), -matched_points1(2,j);  R_c2w(3,1), R_c2w(3,2), -1];
            Lambda_solve = G\(-position_cam);
            Z(j) = Lambda_solve(3);
        
        end

        % Z is table of point depths
        Vel_cam=velocityRANSAC(opt_flow,matched_points1,Z,R_c2w,e);

    end
%% Express Vel in world frame    
    R_w2c = R_c2w';
    Vel=vertcat(horzcat(R_w2c,zeros(3)),horzcat(zeros(3),R_w2c))*Vel_cam;
    
    %% STORE THE COMPUTED VELOCITY IN THE VARIABLE estimatedV AS BELOW

    estimatedV(:,n) = Vel;
end

for i=1:6
    estimatedV(i,:)=lowpass(estimatedV(i,:),10,1000);  
end

plotData(estimatedV, sampledData, sampledVicon, sampledTime, datasetNum)
