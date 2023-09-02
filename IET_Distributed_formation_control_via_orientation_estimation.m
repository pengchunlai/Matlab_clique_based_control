clear;clc;

iteration=500;% this is iteration number 
% %%----5 agents----%%
%     clique=[1 2 3;2 3 4;2 4 5];%each column represents each clique
%     n=max(clique(:));%number of robots
%     %x(:,:,1)=[0.2578   -2.2269    4.9514    2.2442    2.6658;-8.4748   -3.1847   -1.8468   0.5773   -7.2945];%5 agents
%     x(:,:,1)=[6.5   10.8   8.5   15    9.2;3.2   10.0  -2.5  -13.2  0.0];%randn(2,n);%[-3 -2 3 -1;-1 -1.5 -1 1];
%     %x_star=5*[1.5 1 2 2 1; 2.86 2 2 1 1];%5 agents
%     x_star=4*[1.5 1 2 2.5 0.5; 3 2 2 1 1];%5 agents
%     %----desired orientation for each agents----%
%     %theta_star = [1.18,0.57,-2.51,-1.34,2];%5 agents
%     theta_star =[pi/2,pi-pi/6,pi/6,-pi/4,pi+pi/4];
%     %theta = [1.6 0.8 -0.9 0 -0.75];%5 agents angle between y axis of local frame of agents and global frame
%     theta = [3.0,2.1,4.3,4.0,2.8,0.9];%5*rand(1,n);%[1.6 0.8 -0.9 0 -0.75 0];% 6 angle between y axis of local frame of agents and global frame
%     theta_hat =[1.2,4.4,0.1,2.4,0.8,4.8];% 5 agents_estimated angle between y axis of local frame of agents and global frame
% %%----5 agents----%%
%%----6 agents----%%
	clique=[1 2 3;2 4 5;2 3 5;3 5 6];%each column represents each clique
	n=max(clique(:));%number of robots
    x_star=5*[0 -0.5 0.5 -1 0 1;1.1 0.1 0.1 -1+0.1 -1+.01 -1+.01];%desired formation of 6 agents
    theta_star = [pi/2,pi/3+pi/2,pi/6,pi,-pi/2,0];
    theta = [2.84,2.34,0.059,1.68,0.81,3.97];%5*rand(1,n);%[0 0.8 -0.9 0 -0.75 0];% 6 angle between y axis of local frame of agents and global frame
	x=zeros(2,n,iteration);%initialize the location of agents
    x(:,:,1)=1.7*[-4 -4 -4 4 4 4;-5 -3 -0 0 2 5];%5*randn(2,n);%1.5*[-4 -4 -4 4 4 4;-4 -3 -0 0 2 5];%%[-3 -2 3 -1;-1 -1.5 -1 1];
    theta_hat =[1.2,4.4,0.1,2.4,0.8,4.8];% 6 agents_estimated angle between y axis of local frame of agents and global frame
%%----6 agents----%%
%clique=[1 2 3];%each column represents each clique
%clique=[1 2 3;2 4 5;2 3 5;3 5 6];%each column represents each clique
%clique=[1 2 3;2 4 6;2 6 7;2 3 7;3 7 8;3 5 8;2 3 9;3 7 10];
% clique=[1 3 4;2 3 6;3 6 7;3 4 7;4 7 8;4 5 8;6 9 10;6 7 10;7 10 11;7 8 11;8 11 12;10 11 13];




% x(:,:,1)=1.5*[-4 -3 -1 1 2 4;0 0 0 0 0 0];%5*randn(2,n);%[-3 -2 3 -1;-1 -1.5 -1 1];
%x(:,:,1)=[3 1 2 5 1;3 2 1 2 1];%initial position of 5 agents
%x(:,:,1)=1.5*[-3 -2 2 3;-1 -1 -1 -1 ];%3*randn(2,n);%initial position of 4 agents
%x(:,:,1)=1.5*[0 -2 -2 0 0.5 1.5 2.4 1 3 5;3 -2 1  -2 -1 2 1 0 2 -1];%initial position of 10 agents

v=zeros(2,n,iteration);%velocity of agents

%x_star=5*[0 -0.5 0.5 -1 0 1;1.1 0.1 0.1 -1+0.1 -1+.01 -1+.01];%desired formation of 6 agents
b=[1;0];%orientation of the each agent
%----desired orientation for each agents----%
R_star =zeros(2,2,n);
b_star = zeros(2,n);
for i=1:n
    R_star(:,:,i) = [cos(theta_star(i)) -sin(theta_star(i));sin(theta_star(i)) cos(theta_star(i))];
    b_star(:,i) = [cos(theta_star(i));sin(theta_star(i))];
end
%----initial orientation of each agent-----------%
R=zeros(2,2,n,iteration);%local frame of agents
for i=1:n
    R(:,:,i,1)=[cos(theta(i)) -sin(theta(i));sin(theta(i)) cos(theta(i))];%initial local frame for agents
end
%----initial state of estimated orientation----%
R_hat=zeros(2,2,n,iteration);
for i=1:n
    R_hat(:,:,i,1)=[cos(theta_hat(i)) -sin(theta_hat(i));sin(theta_hat(i)) cos(theta_hat(i))];%initial local frame for agents
end

%----shape of the robot----------------------%
probot=zeros(2,6,n);
probot0=[-1 1 2 1 -1 -1 ;
         1 1 0 -1 -1 1 ];%the shape of the robot
for i=1:n
    probot(:,:,i)=probot0;%the shape of the robot
end

%%---the below is the parameter in the paper
% %----initial orientation of each agent-----------%
% theta = theta_star+[1.6 0.8 -0.9 0 -0.75];%angle between y axis of local frame of agents and global frame
% R=zeros(2,2,n,iteration);%local frame of agents
% for i=1:n
%     R(:,:,i,1)=[cos(theta(i)) -sin(theta(i));sin(theta(i)) cos(theta(i))];%initial local frame for agents
% end
% 
% %----initial state of estimated orientation----%
% theta_hat = theta - [0.51 1.3 0.9 -0.7 2.4];%estimated angle between y axis of local frame of agents and global frame
% R_hat=zeros(2,2,n,iteration);
% for i=1:n
%     R_hat(:,:,i,1)=[cos(theta_hat(i)) -sin(theta_hat(i));sin(theta_hat(i)) cos(theta_hat(i))];%initial local frame for agents
% end
% x(:,:,1)=[3 1 2 5 1;3 2 1 2 1];%3*randn(2,n);%initial position of agents
%
% x_star=[1.5 1 2 2 1; 2.86 2 2 1 1];%desired formation of 5 agents
% %----desired orientation for each agents----%
% theta_star = [1.18,0.57,-2.51,-1.34,2];
% R_star =zeros(2,2,n);
% b_star = zeros(2,n);
% for i=1:n
%     R_star(:,:,i) = [cos(theta_star(i)) -sin(theta_star(i));sin(theta_star(i)) cos(theta_star(i))];
%     b_star(:,i) = [cos(theta_star(i));sin(theta_star(i))];
% end

t=0.1;%time interval for iteration

%% 
for k=1:iteration

    for i=1:n
        [row,col]= find(clique==i);
        s=zeros(2,2);
        phi = zeros(2,2);%phi is the first term in the equation of R_hat_dot
        for j=1:max(row) %cliuqe j
            cl_n=size(clique(j,:));%cl_n(2) is the number of agents in clique j
            matrix_z=zeros(2,2);
            matrix_y=zeros(2,2);
            xk_i=zeros(2,cl_n(2));%relative position of other agents in clique j saw from i
            Rk_i=zeros(2,2,cl_n(2));%relative orientation of other agents in clique j saw from i
            %--define the relative measurement of agent i's neighbors in clique j---
            for z=1:cl_n(2)%cl_n(2) is the number of agents in clique j
                clique_xk=clique(j,z);%the agent k in the clique where agent i locates,i.e., the neighbors of i 
                xk_i(:,z)=transpose(R(:,:,i,k))*(x(:,clique_xk,k)-x(:,i,k));%relative position of k from i
                Rk_i(:,:,z)=transpose(R(:,:,clique_xk,k))*R(:,:,i,k);%relative orientation of i in k's frame
            end
            %--compute x_dot and phi of agent i's neighbors in clique j---                   
            for z=1:cl_n(2)%cl_n(2) is the number of agents in clique j
                clique_xk=clique(j,z);%the neighbor k of agent i in clique j 
                matrix_R = R_hat(:,:,clique_xk,k)*Rk_i(:,:,z)-R_hat(:,:,i,k);
                if clique_xk ~= i
                    matrix_z=matrix_z+matrix_R/(norm(matrix_R,'fro'))^(0.9);
                end
                x_dot_cli(:,z) = x(:,z,k)-x(:,i,k)-transpose(R(:,:,i,k))*R_star(:,:,i)*(x_star(:,z)-x_star(:,i));%compute x_dot in each clique
            end
            x_dot(:,j) = sum(x_dot_cli,2);%compute x_dot for all cliques
            phi= phi+matrix_z;%sum up phi for all cliques

        end
        R_bar=transpose(R_star(:,:,i))*R_hat(:,:,i,k);
        s=0.5*(transpose(R_bar)-R_bar);
        R_hat_dot= phi + R_hat(:,:,i,k)*s;
       
        %----iteration----%
        v(:,i) = sum(x_dot,2); %velocity of x_i
        x(:,i,k+1)=v(:,i)*t+x(:,i,k);%update the position for agent i
        R_hat(:,:,i,k+1)=R_hat(:,:,i,k)+R_hat_dot*t;%update the orientation for agent i
        R(:,:,i,k+1)=R(:,:,i,k)*expm(s*t);%update the orientation for agent i
        
        norm_R(i,k)=norm(R(:,:,i,k+1));
        probot(:,:,i)=R(:,:,i,k)*probot0+kron(x(:,i,k),[1 1 1 1 1 1]);%motion of the shape of robots
        probot_plot(:,:,i,k+1)=probot(:,:,i);%record the motion of shape of robots
    end

end

% verify the final results of relative distance-----------------------------
k=1;
for i=1:n-1
    for j=i+1:n
        desired_dist_agent(k)=norm(x_star(:,i)-x_star(:,j));
        dist_agent(k)=norm(x(:,i,iteration)-x(:,j,iteration));
        k=k+1;
    end
end
desired_dist_agent-dist_agent

% %---- plot the position and orientation error during iteration-------------
% ax1 = nexttile;
% x_pos(:,:) = x(1,:,1:iteration);
% plot(ax1,1:iteration,x_pos(:,:));
% title(ax1,'Convergence of coordinate x');
% 
% ax2 = nexttile;
% y_pos(:,:) = x(2,:,1:iteration);
% plot(ax2,1:iteration,y_pos(:,:));
% title(ax2,'Convergence of coordinate y');
% figure
% for k=1:iteration
%     for j=1:n
%         norm_orientation(j,k) = norm(R(:,:,j,k+1)*[1;0]-b_star(:,j));
%     end
% end
% plot(1:iteration,norm_orientation(:,:));
% title('Convergence of orientation error');
figure
%----plot the initial postion of robot-----------------------------
for i=1:n
    plot(x(1,i,1),x(2,i,1),'x','Color','k','MarkerSize',20,'LineWidth',2);hold on %position of robot n
end
axis square
axis equal

%trejatory-------------------------------------------------------------------
for k=1:iteration
    for i=1:n
        plot(x(1,i,k),x(2,i,k),'.','Color',[0.5 0.5 0.5],'MarkerSize',3);hold on %robot i
    end
end

%%plot the shape of robots---------------------------   
for i=1:n
    plot(probot_plot(1,:,i,iteration),probot_plot(2,:,i,iteration),'LineWidth',3);hold on % shape of robot n
    plot(x(1,i,iteration),x(2,i,iteration),'.','Color','k','MarkerSize',3);hold on %position of robot n
    %axis([-25 5 -25 5]) %the span of x and y axis
    axis equal
    for i=1:size(clique,1)
        for j=1:size(clique(i,:),2)
            if j~=3
             plot([x(1,clique(i,j),iteration) x(1,clique(i,j+1),iteration)],[x(2,clique(i,j),iteration) x(2,clique(i,j+1),iteration)],'Color','k');hold on%j j+1
            else
             plot([x(1,clique(i,j),iteration) x(1,clique(i,1),iteration)],[x(2,clique(i,j),iteration) x(2,clique(i,1),iteration)],'Color','k');hold on%j 1
            end
        end
    end
end



%show the motion of robots------------------------
M = moviein(iteration);%create and store the image for each instant
fig=figure;

for k=2:iteration
    for i=1:n
        plot(probot_plot(1,:,i,k),probot_plot(2,:,i,k),'LineWidth',5);hold on % shape of robot n
        plot(x(1,i,k),x(2,i,k),'.','Color','k','MarkerSize',20);hold on %position of robot n
        %plot(0,0,'+','Color','k','MarkerSize',10);hold on
        axis equal;
        axis([-5 10 -5 10]) %the span of x and y axis
    end
    % the topology of robots------------------------
    for i=1:size(clique,1)
        for j=1:size(clique(i,:),2)
            if j~=3
             plot([x(1,clique(i,j),k) x(1,clique(i,j+1),k)],[x(2,clique(i,j),k) x(2,clique(i,j+1),k)],'Color','k');hold on%j j+1
            else
             plot([x(1,clique(i,j),k) x(1,clique(i,1),k)],[x(2,clique(i,j),k) x(2,clique(i,1),k)],'Color','k');hold on%j 1
            end
        end
    end

    %----------------------------------------------
    drawnow
    M(k) = getframe(fig);%show the motion
    im{k} = frame2im(M(k));
    hold off
end

%% the old version that only consider 3 robots
% %% initial state with bad simulation results
% x1(:,1) = [5;2]; x2(:,1) = [0;5]; x3(:,1) = [1;0]; %initial position
% %%-----initial estimated orientation angle-----%%
% theta1_hat = 0;
% theta2_hat = pi/2;
% theta3_hat = pi/4;
% 
% %%-----initial orientation angle-----%%
% theta1 = 5.7411;
% theta2 = pi;
% theta3 = -pi/2;
% %% the above is the initial state with bad simulation results
% % %%-----initial orientation angle-----%%
% % theta1 = 5.7411;
% % theta2 = pi;
% % theta3 = -pi/2;
% % %%-----initial estimated orientation angle-----%%
% % theta1_hat = pi/rand;
% % theta2_hat = pi/2;
% % theta3_hat = pi/4;
% % x1(:,1) = [5;2]; x2(:,1) = [0;5]; x3(:,1) = [1;0]; %initial position
% t = 0.2;
% iter = 500;
% 
% x1_star = [0;0];
% x2_star = [0;4];
% x3_star = [3;0];
% %%-----initial orientation of robot-----%%
% R1(:,:,1) = [cos(theta1) -sin(theta1); sin(theta1) cos(theta1)];%initial local frame for agents
% R2(:,:,1) = [cos(theta2) -sin(theta2); sin(theta2) cos(theta2)];%initial local frame for agents
% R3(:,:,1) = [cos(theta3) -sin(theta3); sin(theta3) cos(theta3)];%initial local frame for agents
% 
% 
% %%-----initial estimated orientation-----%%
% R1_hat(:,:,1) = [cos(theta1_hat) -sin(theta1_hat); sin(theta1_hat) cos(theta1_hat)];%initial local frame for agents
% R2_hat(:,:,1) = [cos(theta2_hat) -sin(theta2_hat); sin(theta2_hat) cos(theta2_hat)];%initial local frame for agents
% R3_hat(:,:,1) = [cos(theta3_hat) -sin(theta3_hat); sin(theta3_hat) cos(theta3_hat)];%initial local frame for agents
% 
% %%-----desired orientation angle-----%%
% theta1_star = 0;
% theta2_star = pi/4;
% theta3_star = pi/2;
% 
% %%-----desired orientation-----%%
% R1_star = [cos(theta1_star) -sin(theta1_star); sin(theta1_star) cos(theta1_star)];
% R2_star = [cos(theta2_star) -sin(theta2_star); sin(theta2_star) cos(theta2_star)];
% R3_star = [cos(theta3_star) -sin(theta3_star); sin(theta3_star) cos(theta3_star)];
% 
% 
% for k = 1:iter
%     %%-----relative orientation-----%%
%     R12(:,:,k) = transpose(R2(:,:,k))*R1(:,:,k);%[cos(theta12) -sin(theta12); sin(theta12) cos(theta12)];%initial local frame for agents
%     R13(:,:,k) = transpose(R3(:,:,k))*R1(:,:,k);%[cos(theta13) -sin(theta13); sin(theta13) cos(theta13)];%initial local frame for agents
%     R23(:,:,k) = transpose(R3(:,:,k))*R2(:,:,k);%[cos(theta23) -sin(theta23); sin(theta23) cos(theta23)];%initial local frame for agents
%     R21(:,:,k) = transpose(R12(:,:,k));
%     R31(:,:,k) = transpose(R13(:,:,k));
%     R32(:,:,k) = transpose(R23(:,:,k));
%     %%-----iteration of the estimated orientation----%
%     X12 = R2_hat(:,:,k)*R12(:,:,k) - R1_hat(:,:,k); X13 = R3_hat(:,:,k)*R13(:,:,k) - R1_hat(:,:,k); 
%     X21 = R1_hat(:,:,k)*R21(:,:,k) - R2_hat(:,:,k); X23 = R3_hat(:,:,k)*R23(:,:,k) - R2_hat(:,:,k);
%     X31 = R1_hat(:,:,k)*R31(:,:,k) - R3_hat(:,:,k); X32 = R2_hat(:,:,k)*R32(:,:,k) - R3_hat(:,:,k);
%     R1_bar(:,:,k)=transpose(R1_star)*R1_hat(:,:,k);S1(:,:,k)=0.5*(transpose(R1_bar(:,:,k))-R1_bar(:,:,k));
%     R2_bar(:,:,k)=transpose(R2_star)*R2_hat(:,:,k);S2(:,:,k)=0.5*(transpose(R2_bar(:,:,k))-R2_bar(:,:,k));
%     R3_bar(:,:,k)=transpose(R3_star)*R3_hat(:,:,k);S3(:,:,k)=0.5*(transpose(R3_bar(:,:,k))-R3_bar(:,:,k));
% 
%     R1_hat_dot(:,:,k) = X12/sqrt(norm(X12,'fro')) + X13/sqrt(norm(X13,'fro')) + R1_hat(:,:,k)*S1(:,:,k); 
%     R2_hat_dot(:,:,k) = X21/sqrt(norm(X21,'fro')) + X23/sqrt(norm(X23,'fro')) + R2_hat(:,:,k)*S2(:,:,k);
%     R3_hat_dot(:,:,k) = X31/sqrt(norm(X31,'fro')) + X32/sqrt(norm(X32,'fro')) + R3_hat(:,:,k)*S3(:,:,k);
%     
%     R1_hat(:,:,k+1) = R1_hat(:,:,k)+R1_hat_dot(:,:,k)*t;
%     R2_hat(:,:,k+1) = R2_hat(:,:,k)+R2_hat_dot(:,:,k)*t;
%     R3_hat(:,:,k+1) = R3_hat(:,:,k)+R3_hat_dot(:,:,k)*t;
% 
%     %%----orientation control----%%
%     R1(:,:,k+1) = R1(:,:,k)*expm(0.5*S1(:,:,k)*t);
%     R2(:,:,k+1) = R2(:,:,k)*expm(0.5*S2(:,:,k)*t);
%     R3(:,:,k+1) = R3(:,:,k)*expm(0.5*S3(:,:,k)*t);
% %     %compute the angle of R_hat
% %     z1 = R1_hat(1,1) + i*R1_hat(2,1);
% %     z2 = R2_hat(1,1) + i*R2_hat(2,1);
% %     z3 = R3_hat(1,1) + i*R3_hat(2,1);
% %     
% %     theta1(k) = angle(z1)*180/pi;
% %     theta2(k) = angle(z2)*180/pi;
% %     theta3(k) = angle(z3)*180/pi;
% %     theta_(:,k) = [theta1(k);theta2(k);theta3(k)];%check the angle of estimated orientation
%     
%     %%----position control----%%
%     x1_dot(:,k) = x2(:,k)+x3(:,k)-2*x1(:,k)-R1(:,:,k)*transpose(R1_star)*(x3_star-x1_star)-R1(:,:,k)*transpose(R1_star)*(x2_star-x1_star);
%     x2_dot(:,k) = x1(:,k)+x3(:,k)-2*x2(:,k)-R2(:,:,k)*transpose(R2_star)*(x3_star-x2_star)-R2(:,:,k)*transpose(R2_star)*(x1_star-x2_star);
%     x3_dot(:,k) = x1(:,k)+x2(:,k)-2*x3(:,k)-R3(:,:,k)*transpose(R3_star)*(x1_star-x3_star)-R3(:,:,k)*transpose(R3_star)*(x2_star-x3_star);
%     x1(:,k+1) = x1(:,k) + x1_dot(:,k)*t;
%     x2(:,k+1) = x2(:,k) + x2_dot(:,k)*t;
%     x3(:,k+1) = x3(:,k) + x3_dot(:,k)*t;
%      %compute the angle of R
%     z1 = R1(1,1,k) + i*R1(2,1,k);
%     z2 = R2(1,1,k) + i*R2(2,1,k);
%     z3 = R3(1,1,k) + i*R3(2,1,k);
%     
%     R1_theta(k) = angle(z1)*180/pi;
%     R2_theta(k) = angle(z2)*180/pi;
%     R3_theta(k) = angle(z3)*180/pi;
%     theta_(:,k) = [R1_theta(k);R2_theta(k);R3_theta(k)];%check the angle of estimated orientation
% end
% 
% %%----verify the final result
% [R1_theta(iter)-R2_theta(iter),R2_theta(iter)-R3_theta(iter),R3_theta(iter)-R1_theta(iter)]
% [norm(x1(:,iter)-x2(:,iter))-norm(x1_star-x2_star),norm(x2(:,iter)-x3(:,iter))-norm(x2_star-x3_star),norm(x1(:,iter)-x3(:,iter))-norm(x1_star-x3_star)]
% 
% % show the convergence of robots' orientation
% plot(R1_theta);hold on
% plot(R2_theta);hold on
% plot(R3_theta);
% 
% % show the convergence of robots' position
% figure
% plot(x1_dot(1,:));hold on
% plot(x1_dot(2,:));hold on
% plot(x2_dot(1,:));hold on
% plot(x2_dot(2,:));hold on
% plot(x3_dot(1,:));hold on
% plot(x3_dot(2,:));hold on
% 
% % %the shape of robot
% probot0=2*[0.2,0.1,-0.1,-0.1,0.1,0.2;
%            0, -0.1,-0.1, 0.1,0.1,0];
% probot1(:,:)=R1(:,:,iter)*probot0+kron(x1(:,iter),[1 1 1 1 1 1]);
% probot2(:,:)=R2(:,:,iter)*probot0+kron(x2(:,iter),[1 1 1 1 1 1]);
% probot3(:,:)=R3(:,:,iter)*probot0+kron(x3(:,iter),[1 1 1 1 1 1]);
% 
% %plot all robots
% figure
% %%---the trajactory of all robots----
% plot(x1(1,:),x1(2,:));hold on
% plot(x2(1,:),x2(2,:));hold on
% plot(x3(1,:),x3(2,:));hold on
% %%----the topology of all robots----
% plot([x1(1,iter) x2(1,iter)],[x1(2,iter) x2(2,iter)],'Color','k');hold on%1 and 2
% plot([x1(1,iter) x3(1,iter)],[x1(2,iter) x3(2,iter)],'Color','k');hold on%1 and 3
% plot([x2(1,iter) x3(1,iter)],[x2(2,iter) x3(2,iter)],'Color','k');hold on%2 and 3
% %%---the final orientation of all robots----
% plot(probot1(1,:),probot1(2,:));hold on
% plot(probot2(1,:),probot2(2,:));hold on
% plot(probot3(1,:),probot3(2,:));
% axis equal;
% %axis([-2 2 -2 2])