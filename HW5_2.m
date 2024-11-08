
% simulations for Lecture 1
% inverted pendulum 
% Santosh Devasia
clc
clear all       % clear all variables  
%close all      % close all figures 
nfig=0;         % figure number

GoverL=1;   % g/l
Kfb =.2;    % velocity feedback
Kposfb = 0; % position feedback
Lx=5;Ly=5;Npoints=51; % width of plot and number of points

% with position feedback 
%Kposfb = 2*GoverL;Lx=3.5*pi;Ly=5*pi;Npoints=21;  % position feedback 


axisrange =[-Lx,Lx,-Ly,Ly];


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% %%%%%%%%%%%%%%%%%% %%%%%%%%%%%%%%%%%%  
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% %%%%%%%%%%%%%%%%%% %%%%%%%%%%%%%%%%%%  
% Step 1 Phase portrait 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% %%%%%%%%%%%%%%%%%% %%%%%%%%%%%%%%%%%%  
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% %%%%%%%%%%%%%%%%%% %%%%%%%%%%%%%%%%%%  

xdom = linspace(-Lx,Lx,Npoints);
ydom = linspace(-Ly,Ly,Npoints);
[X1,X2] = meshgrid(xdom,ydom); % generate mesh of domain 
U = -X2-X1.*(1-X1.^2-X2.^2); % dx/dt F1 
V = X1-X2.*(1-X1.^2-X2.^2);   % F2 dy/dt
nfig=nfig+1; figure(nfig); clf
quiver(X1,X2,U,V,LineWidth=1)
xlabel('$X1$','Interpreter','latex')
ylabel('$X2$','Interpreter','latex')
set(gca,'FontSize',20)
axis(axisrange)
%pause(0.01)
%saveas(gcf,'L1_fig_1_pendulum','epsc')
hold on 


%return

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% plot the level sets
% 
V_fun=0.5*X1.^2+0.5*X2.^2;
% 
% s=surf(X1,X2,V_fun,'FaceAlpha',0.5)
% s.EdgeColor = 'none';
% hold on
% % [X1,X2,V_fun] = peaks;
% [M,c] = contour3(X1,X2,V_fun,[1 2 4 8 13 17 25],'r','ShowText','on')
% c.LineWidth = 3;
% 
% t=linspace(0,2*pi,201);
% x_cood=cos(t);
% y_cood=sin(t);
% plot(x_cood,y_cood,'c-',LineWidth=2);

% % plot V_dot
% 
V_dot=-(1-X1.^2-X2.^2).*(X1.^2+X2.^2);
s=surf(X1,X2,V_dot,'FaceAlpha',0.5)
s.EdgeColor = 'none';
hold on
% [X1,X2,V_fun] = peaks;
[M,c] = contour3(X1,X2,V_dot,[0 0],'r','ShowText','on')
c.LineWidth = 3

[M,c] = contour3(X1,X2,V_fun,[1 2 4 8 13 17 25],'b','ShowText','on')
c.LineWidth = 3;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% %%%%%%%%%%%%%%%%%% %%%%%%%%%%%%%%%%%%  
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% %%%%%%%%%%%%%%%%%% %%%%%%%%%%%%%%%%%%  
% Step 2 Equilibrium points 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% %%%%%%%%%%%%%%%%%% %%%%%%%%%%%%%%%%%%  


% Stable
x=[0]; 
y=[0];
plot(x,y,'b.','MarkerSize',50,'LineWidth',0.8); grid; 
hold on 
axis(axisrange)






%return

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% %%%%%%%%%%%%%%%%%% %%%%%%%%%%%%%%%%%%  
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% %%%%%%%%%%%%%%%%%% %%%%%%%%%%%%%%%%%%  
% Step 3 Overlay solutions to specified initial condition
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% %%%%%%%%%%%%%%%%%% %%%%%%%%%%%%%%%%%%  
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% %%%%%%%%%%%%%%%%%% %%%%%%%%%%%%%%%%%%  

param(1) = GoverL;param(2) = Kfb;param(3) = Kposfb; 
options=odeset('RelTol',1e-3);
Tf = 50;
axisrange =[-Lx,Lx,-Ly,Ly];

%%%%%%%%%%%%%%%%%%%%% 
% pre-specified initial conditions
% x1_ini=linspace(-5,5,11)';
% x2_ini=linspace(-5,5,11)';
% for i=1:length(x1_ini)
%     for j=1:length(x2_ini)
%         Xini(j+length(x1_ini)*(i-1),:)=[x1_ini(i) x2_ini(j)];
%     end
% 
% end
% 
% [nx,ny]=size(Xini);
% for jj=1:nx
%    X0= [Xini(jj,1), Xini(jj,2)]; % initial conditions
%    plot_soln(X0,Tf,axisrange,nfig,param,options) 
%    pause(0.1)
% end

%return
%%%%%%%%%%%%%%%%%%%%% 
% initial conditions specified using the plot
% press return to stop


for jj=1:100
    X0= ginput(1); % initial conditions
    if length(X0) ~= 0 
        plot_soln(X0,Tf,axisrange,nfig,param,options) 
    else
        break
    end
end


%   save file if needed
%   saveas(gcf,'L1_fig_1_pendulum','epsc')


return


% plot a solution over time 
function [] = plot_soln(X0,Tf,axisrange,nfig,param,options)
[t,x]=ode45(@(t,y) Sys_sim(t,y,flag,param),[0 Tf],X0,options);
x1 = x(:,1); x2 = x(:,2); 
%nfig=nfig+1; figure(nfig); clf
plot(x1(1),x2(1),'ro','MarkerSize',2,'LineWidth',0.8); grid; 
plot(x1,x2,'r','LineWidth',0.8); grid; 
xlabel('x1')
ylabel('x2')
set(gca,'FontSize',20)
axis(axisrange)
end


% this is the nonlinear simulation
function xdiff = Sys_sim(t,x,flag,param,U)
%t  % print out simulation time to check progress of simulation
% extract system parameters
gl = param(1); km = param(2); kp = param(3);
% initialize d/dt x
xdiff = zeros(2,1);

% U = -X2-X1.*(1-X1.^2-X2.^2); % dx/dt F1 
% V = X1-X2.*(1-X1.^2-X2.^2);   % F2 dy/dt

xdiff(1)= -x(2)-x(1).*(1-x(1).^2-x(2).^2);
 
%U = -k1*
xdiff(2)= x(1)-x(2).*(1-x(1).^2-x(2).^2);
end

