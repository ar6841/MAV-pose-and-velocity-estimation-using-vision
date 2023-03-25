function [Vel] = velocityRANSAC(optV,optPos,Z,R_c2w,e)

    %% Input Parameter Description
    % optV = p_dot 
    % optPos = normalized image coordinates of previous frame 
    % Z = Table of depth Z(i) for optPos(i)
    % R_c2w = Rotation defining camera to world frame
    % e = RANSAC hyper parameter    

    k=round(log(1-0.99)/log(1-(e*e*e))); % Calculate this somewhere else
    
    threshold_trail=0.04;
    
    inlier_table=zeros(k,length(optPos)+1);
    %% Inlier table calculation
    for i=1:k
        elements=randperm(length(optPos),3); % Random number generation
       %% Calculate V for 3 random corners 
        trail_opt_flow=optV(:,[elements(1)  elements(2) elements(3)]);
        trail_pos=optPos(:,[elements(1)  elements(2) elements(3)]);
        
        A_trail=vertcat((1/Z(elements(1)))*[-1,    0,  trail_pos(1,1);   0,  -1, trail_pos(2,1)],(1/Z(elements(2)))*[-1,    0,  trail_pos(1,2);   0,  -1, trail_pos(2,2)],(1/Z(elements(3)))*[-1,    0,  trail_pos(1,3);   0,  -1, trail_pos(2,3)]);
        B_trail=vertcat([trail_pos(1,1)*trail_pos(2,1),   -(1+(trail_pos(1,1)*trail_pos(1,1))),    trail_pos(2,1);   1+(trail_pos(2,1)*trail_pos(2,1)), -trail_pos(1,1)*trail_pos(2,1), -trail_pos(1,1)],[trail_pos(1,2)*trail_pos(2,2),   -(1+(trail_pos(1,2)*trail_pos(1,2))),    trail_pos(2,2);   1+(trail_pos(2,2)*trail_pos(2,2)), -trail_pos(1,2)*trail_pos(2,2), -trail_pos(1,2)],[trail_pos(1,3)*trail_pos(2,3),   -(1+(trail_pos(1,3)*trail_pos(1,3))),    trail_pos(2,3);   1+(trail_pos(2,3)*trail_pos(2,3)), -trail_pos(1,3)*trail_pos(2,3), -trail_pos(1,3)]);

        H_trail=horzcat(A_trail,B_trail);
    
        %Ht_trail=transpose(H_trail);

        p_dot_trail=reshape(trail_opt_flow,[],1);

        %warning('off','MATLAB:nearlySingularMatrix');
        %vel_trail=((H_trail'*H_trail)\H_trail')*p_dot_trail;
        
        vel_trail=pinv(H_trail)*p_dot_trail; % Test velocity
        
        l=0; % number of inliers

        for j=1:length(optPos)
            
            A_check=(1/Z(j))*[-1,    0,  optPos(1,j);   0,  -1, optPos(2,j)];
            B_check=[optPos(1,j)*optPos(2,j),   -(1+(optPos(1,j)*optPos(1,j))),    optPos(2,j);   1+(optPos(2,j)*optPos(2,j)), -optPos(1,j)*optPos(2,j), -optPos(1,j)];
            H_check=horzcat(A_check,B_check);

            argmin=abs(H_check*vel_trail-optV(:,j));
            
            if((argmin(1)<=threshold_trail)&&(argmin(2)<=threshold_trail))

                l=l+1;
                inlier_table(i,l)=j; %save the index of the inlier into inlier_table
            end
            if(j==length(optPos))
                inlier_table(i,length(optPos)+1)=l; %save the number of inliers found in the last column
            end
        end
    end

    [~,max_index]=max(inlier_table(:,length(optPos)+1));

    inlier_table(:,length(optPos)+1)=[]; %remove last column, as its not needed
    
    inlier_pos=inlier_table(max_index,:);
    inlier_pos=inlier_pos(inlier_pos~=0); % remove zeros from max inlier row

    H=zeros(2*length(inlier_pos),6);
    p_dot=zeros(2*length(inlier_pos),1);
%% Computing V using only inliers
    for m=1:length(inlier_pos)
        
        A=(1/Z(inlier_pos(m)))*[-1,    0,  optPos(1,inlier_pos(m));   0,  -1, optPos(2,inlier_pos(m))];
        B=[optPos(1,inlier_pos(m))*optPos(2,inlier_pos(m)),   -(1+(optPos(1,inlier_pos(m))*optPos(1,inlier_pos(m)))),    optPos(2,inlier_pos(m));   1+(optPos(2,inlier_pos(m))*optPos(2,inlier_pos(m))), -optPos(1,inlier_pos(m))*optPos(2,inlier_pos(m)), -optPos(1,inlier_pos(m))];
        H((2*m-1):2*m,:)=horzcat(A,B);
        p_dot((2*m-1):(2*m),1)=optV(:,inlier_pos(m));

    end

    %% Output Parameter Description
    % Vel = Linear velocity and angualr velocity vector
    Vel=((H'*H)\H')*p_dot;
    
end