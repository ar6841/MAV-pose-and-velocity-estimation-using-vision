function [Vel] = par_velocityRANSAC(optV,optPos,Z,e)

    %% Input Parameter Description
    % optV = p_dot 
    % optPos = normalized image coordinates of previous frame 
    % Z = Table of depth Z(i) for optPos(i)
    % e = RANSAC hyper parameter    

    k=round(log(1-0.99)/log(1-(e*e*e))); % Calculate this somewhere else
    
    threshold_trail=0.04;
    
    LEN = length(optPos);

    inlier_table=zeros(k,LEN);

    %% Inlier table calculation
    parfor i=1:k
        elements=randperm(LEN,3); % Random number generation

       %% Calculate V for 3 random corners 
        trail_opt_flow = optV(:,[elements(:)]);
        trail_pos = optPos(:,[elements(:)]);
        
        A_trail=vertcat((1/Z(elements(1)))*[-1,    0,  trail_pos(1,1);   0,  -1, trail_pos(2,1)],(1/Z(elements(2)))*[-1,    0,  trail_pos(1,2);   0,  -1, trail_pos(2,2)],(1/Z(elements(3)))*[-1,    0,  trail_pos(1,3);   0,  -1, trail_pos(2,3)]);
        B_trail=vertcat([trail_pos(1,1)*trail_pos(2,1),   -(1+(trail_pos(1,1)*trail_pos(1,1))),    trail_pos(2,1);   1+(trail_pos(2,1)*trail_pos(2,1)), -trail_pos(1,1)*trail_pos(2,1), -trail_pos(1,1)],[trail_pos(1,2)*trail_pos(2,2),   -(1+(trail_pos(1,2)*trail_pos(1,2))),    trail_pos(2,2);   1+(trail_pos(2,2)*trail_pos(2,2)), -trail_pos(1,2)*trail_pos(2,2), -trail_pos(1,2)],[trail_pos(1,3)*trail_pos(2,3),   -(1+(trail_pos(1,3)*trail_pos(1,3))),    trail_pos(2,3);   1+(trail_pos(2,3)*trail_pos(2,3)), -trail_pos(1,3)*trail_pos(2,3), -trail_pos(1,3)]);

        H_trail=horzcat(A_trail,B_trail);

        p_dot_trail=reshape(trail_opt_flow,[],1);
        
        vel_trail=pinv(H_trail)*p_dot_trail; % Test velocity
        

        for j=1:LEN
            
            A_check=(1/Z(j))*[-1,    0,  optPos(1,j);   0,  -1, optPos(2,j)];
            B_check=[optPos(1,j)*optPos(2,j),   -(1+(optPos(1,j)*optPos(1,j))),    optPos(2,j);   1+(optPos(2,j)*optPos(2,j)), -optPos(1,j)*optPos(2,j), -optPos(1,j)];
            H_check=horzcat(A_check,B_check);

            argmin=abs(H_check*vel_trail-optV(:,j));
            
            if((argmin(1)<=threshold_trail)&&(argmin(2)<=threshold_trail))
                inlier_table(i,j)=1; %save the index of the inlier into inlier_table
            end


        end
    end

    [~,max_inliers_index] = max(sum(inlier_table,2));
    
    inlier_pos=find(inlier_table(max_inliers_index,:)); % remove zeros from max inlier row and return indexes

    H=zeros(2*length(inlier_pos),6);
    p_dot=zeros(2*length(inlier_pos),1);

%% Computing V using only inliers

    for m=1:length(inlier_pos)
        
        A = (1/Z(inlier_pos(m)))*[-1,    0,  optPos(1,inlier_pos(m));   0,  -1, optPos(2,inlier_pos(m))];
        B = [optPos(1,inlier_pos(m))*optPos(2,inlier_pos(m)),   -(1+(optPos(1,inlier_pos(m))*optPos(1,inlier_pos(m)))),    optPos(2,inlier_pos(m));   1+(optPos(2,inlier_pos(m))*optPos(2,inlier_pos(m))), -optPos(1,inlier_pos(m))*optPos(2,inlier_pos(m)), -optPos(1,inlier_pos(m))];
        H((2*m-1):2*m,:) = horzcat(A,B);
        p_dot((2*m-1):(2*m),1) = optV(:,inlier_pos(m));

    end


    %% Output Parameter Description
    % Vel = Linear velocity and angualr velocity vector
    
    Vel=((H'*H)\H')*p_dot;
   
    
end