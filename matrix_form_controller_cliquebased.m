clc;clear;

iteration=300;%iteration number

% %%----5 agents----%%
%     clique=[1 2 3;2 3 4;2 4 5];%each column represents each clique
%     n=max(clique(:));%number of robots
%     %x(:,:,1)=[0.2578   -2.2269    4.9514    2.2442    2.6658;-8.4748   -3.1847   -1.8468   0.5773   -7.2945];%5 agents
%     x(:,:,1)=[6.5 10.8 8.5 15 9.2;3.2 10.0 -2.5 -13.2 0.0];%randn(2,n);%[-3 -2 3 -1;-1 -1.5 -1 1];
%     %x_star=5*[1.5 1 2 2 1; 2.86 2 2 1 1];%5 agents
%     x_star=4*[1.5 1 2 2.5 0.5; 3 2 2 1 1];%5 agents
%     %----desired orientation for each agents----%
%     %theta_star = [1.18,0.57,-2.51,-1.34,2];%5 agents
%     theta_star =-pi/2+[pi/2,pi-pi/6,pi/6,-pi/4,pi+pi/4];
%     %theta = [1.6 0.8 -0.9 0 -0.75];%5 agents angle between y axis of local frame of agents and global frame
%     theta = [3.0,2.1,4.3,4.0,2.8,0.9];%5*rand(1,n);%[1.6 0.8 -0.9 0 -0.75 0];% 6 angle between y axis of local frame of agents and global frame
%     %theta_hat =[1.2,4.4,0.1,2.4,0.8,4.8];% 5 agents_estimated angle between y axis of local frame of agents and global frame
% %%----5 agents----%%
%%----6 agents----%%
	clique=[1 2 3;2 4 5;2 3 5;3 5 6];%each column represents each clique
	n=max(clique(:));%number of robots
    x_star=5*[0 -0.5 0.5 -1 0 1;1.1 0.1 0.1 -1+0.1 -1+.01 -1+.01];%desired formation of 6 agents
    theta_star = -pi/2+[pi/2,pi/3+pi/2,pi/6,pi,-pi/2,0];
    theta = [2.84,2.34,0.059,1.68,0.81,3.97];%5*rand(1,n);%[0 0.8 -0.9 0 -0.75 0];% 6 angle between y axis of local frame of agents and global frame
	x=zeros(2,n,iteration);%initialize the location of agents
    x(:,:,1)=1.7*[-4 -4 -4 4 4 4;-5 -3 -0 0 2 5];%5*randn(2,n);%1.5*[-4 -4 -4 4 4 4;-4 -3 -0 0 2 5];%%[-3 -2 3 -1;-1 -1.5 -1 1];
%%----6 agents----%%

%clique=[1 2 3];%each column represents each clique
%clique=[1 2 3;2 3 4];%each column represents each clique
%clique=[1 2 3;2 4 6;2 6 7;2 3 7;3 7 8;3 5 8;2 3 9;3 7 10];%(10 robot)
% clique=[1 3 4;2 3 6;3 6 7;3 4 7;4 7 8;4 5 8;6 9 10;6 7 10;7 10 11;7 8 11;8 11 12;10 11 13];
n=max(clique(:));%number of robots
%----initial position of all agents-----


%x(:,:,1)=1.5*[-3 -2 2 3;-1 -1 -1 -1 ];%3*randn(2,n);%initial position of 4 agents
%x(:,:,1)=1.5*[0 -2 -2 0 0.5 1.5 2.4 1 3 5;3 -2 1  -2 -1 2 1 0 2 -1];%initial position of 10 agents

v=zeros(2,n,iteration);%initialize the velocity of agents

%x_star=5*[1.5 1 2 2 1; 2.86 2 2 1 1];%desired formation of 5 agents
%x_star=1.6*[0 -1 1 -3 3 -2 0 2 -2 2;2*sqrt(2) 1*sqrt(2) 1*sqrt(2) 1*sqrt(2) 1*sqrt(2) 0 0 0 -1*sqrt(2) -1*sqrt(2)];%desired formation of 6 agents

b=[1;0];%orientation of the each agent
%b_s=[0 0.5 -0.5 -0.5 0.5 0.71 0 -0.71 -0.5 0.5;1 0.87 0.87 0.87 0.87 0.71 -1 0.71 -0.87 -0.87];

%----initial orientation of all agents-----
R=zeros(2,2,n,iteration);%local frame of agents
for i=1:n
    R(:,:,i,1)=[cos(theta(i)) -sin(theta(i)); sin(theta(i)) cos(theta(i))];%initial local frame for agents
end
%----desired orientation for each agents----%
R_star =zeros(2,2,n);
b_star = zeros(2,n);
for i=1:n
    R_star(:,:,i) = [cos(theta_star(i)) -sin(theta_star(i));sin(theta_star(i)) cos(theta_star(i))];
    %b_star(:,i) = [cos(theta_star(i));sin(theta_star(i))];
end

%shape of the robot------------------------------
probot=zeros(2,6,n);
probot0=[-1 1 2 1 -1 -1 ;
         1 1 0 -1 -1 1 ];%the shape of the robot
for i=1:n
    probot(:,:,i)=probot0;%the shape of the robot
end

t=0.05;%time interval for iteration

%% 
for k=1:iteration
    for i=1:n
        [row,col]= find(clique==i);
        s=zeros(2,2);
        for j=1:max(row) %cliuqe j
            cl_n=size(clique(j,:));%cl_n(2) is the number of agents in clique j
            matrix_z=zeros(2,2);
            matrix_y=zeros(2,2);
            xk_i=zeros(2,cl_n(2));%relative position of other agents in clique j saw from i
            Rk_i=zeros(2,2,cl_n(2));%relative orientation of other agents in clique j saw from i
            %--compute the average of matrix x and x_star of clique j---
            for z=1:cl_n(2)%cl_n(2) is the number of agents in clique j
                clique_xk=clique(j,z);%the agent k in the clique where agent i locates
                xk_i(:,z)=transpose(R(:,:,i,k))*(x(:,clique_xk,k)-x(:,i,k));%relative position of k from i
                Rk_i(:,:,z)=transpose(R(:,:,i,k))*R(:,:,clique_xk,k);%relative orientation of k from i
            end
            ave_x=mean(xk_i(:,:),2);
            ave_x_star=mean(x_star(:,clique(j,:)),2);
            %------------------------------------------------------------
            for z=1:cl_n(2)%cl_n(2) is the number of agents in clique j
                clique_xk=clique(j,z);%the agent k in the clique where agent i locates
                matrix_z=matrix_z+(x_star(:,clique_xk)-ave_x_star)*transpose(xk_i(:,z)-ave_x);
                matrix_y=matrix_y+R_star(:,:,clique_xk)*transpose(Rk_i(:,:,z));
            end
            [U,S,V] = svd(matrix_z+matrix_y);%target at agent1 and 2
            Rc=V*blkdiag(eye(1),det(U*V))*transpose(U);%construction of Rc
            Phi(:,j)=Rc*(x_star(:,i)-ave_x_star)+ave_x;
            ss=transpose(([1 0;0 1]-Rc*R_star(:,:,i)))-([1 0;0 1]-Rc*R_star(:,:,i));
            s=s+ss;%control input for orientation
        end
        phi=sum(Phi,2);%control input for position
        norm_phi(i,k)=norm(phi);
        %--------------------------------------
        v=R(:,:,i,k)*phi;%velocity of robot n
        x(:,i,k+1)=v*t+x(:,i,k);%update the position for agent i
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

%---- plot the position and orientation error during iteration-------------
ax1 = nexttile;
x_pos(:,:) = x(1,:,1:iteration);
plot(ax1,1:iteration,x_pos(:,:));
title(ax1,'Convergence of coordinate x');

ax2 = nexttile;
y_pos(:,:) = x(2,:,1:iteration);
plot(ax2,1:iteration,y_pos(:,:));
title(ax2,'Convergence of coordinate y');
figure
for k=1:iteration
    for j=1:n
        norm_orientation(j,k) = norm(R(:,:,j,k+1)-R_star(:,:,j));
    end
end
plot(1:iteration,norm_orientation(:,:));
title('Convergence of orientation error');
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
        plot(x(1,i,k),x(2,i,k),'.','Color',[0.5 0.5 0.5],'MarkerSize',7);hold on %robot i
    end
end

%%plot the shape of robots---------------------------   
for i=1:n
    plot(probot_plot(1,:,i,iteration),probot_plot(2,:,i,iteration),'LineWidth',8);hold on % shape of robot n
    plot(x(1,i,iteration),x(2,i,iteration),'.','Color','k','MarkerSize',12);hold on %position of robot n
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
        plot(probot_plot(1,:,i,k),probot_plot(2,:,i,k),'LineWidth',7);hold on % shape of robot n
        plot(x(1,i,k),x(2,i,k),'.','Color','k','MarkerSize',20);hold on %position of robot n
        %plot(0,0,'+','Color','k','MarkerSize',10);hold on
        axis equal;
        axis([-12 10 -12 10]) %the span of x and y axis
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





