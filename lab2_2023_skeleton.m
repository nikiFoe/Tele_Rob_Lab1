clear all
close all

% 
% Telerobotics lab2, 2020
% Script programmed by Sven Rönnbäck 
% Revised September 23, 2020
% 
% Script to learn Kalman Filer
%
% It is setup to to sensor fusion of laser range data and rate gyro data.
% It estimates range and bearing angle to a wall, and the bias point of a rate gyro
%
% Frequency of rate gyro readings is 100Hz
% Frequency of laser range readings is 25Hz
%
%

log_ad_gyro=load('log_ad_gyro_5.txt');  % load gyro readings 
log_tim55=load('log_tim55_5.txt');		% load laser range readings

time_ad_gyro = log_ad_gyro(:,2);		% get time stamps for gyro readings
time_tim55= log_tim55(:,2);				% get time stamps for laser range readings
time_begin_ad=min(time_ad_gyro);        % Get first time stamp
time_begin_tim55=min(time_tim55);
time_begin = min([time_begin_ad time_begin_tim55]);
rate_ad=log_ad_gyro(:,3)*pi/180;		% Get vector with rate gyro readings

std_ad_gyro = std(rate_ad(1:400));      % Compute roughly std 
mean_ad_gyro= mean(rate_ad(1:400));     % Compute mean roughly

heading_ad=log_ad_gyro(:,4);

% Compute time vector
t=time_ad_gyro-time_begin;

% Plot rate gyro readings
figure
subplot(2,1,1);
plot(t,rate_ad*180/pi,'.');
grid on
title('Rate Gyro');
xlabel('Time (s)');
ylabel('Angular Rate (deg/s)');

subplot(2,1,2);
plot(t,heading_ad,'.');
title('Heading (\psi)');
xlabel('Time (s)');
ylabel('\psi(deg/s)');
grid on

% Code to find the first peak, in first laser scan to get initial estimate
vAngles=[-135:1:135]*pi/180;
vRanges=log_tim55(1,3:273)*0.00106;
[C Th_axis R_axis]=RWhough(vAngles,vRanges,100,0.05);
figure;
surf(Th_axis*180/pi,R_axis,C);     % Show detected Hough Peeks
xlabel('\theta (degrees)');
ylabel('Range (m)');
title('Hough Transform');

% Plot Range and Angle together
figure;
scan = lidarScan(vRanges,vAngles);
plot(scan)

hold on



% Find max peak in Hough Space
max_val=max(C(:))    ;         			% Find value of max peak
[row, col]=find(C == max_val);			 % Find coordinates of top
C(:,max(col-15,1):min(col+15,length(Th_axis)))=0; % Clear current peak   

%PlotHoughLine(R_axis(row), Th_axis(col),'g-');  

%
% Initiate Kalman Filter vectors and matrixes
%
%Start with this estimate, first detected peak and bias
Xe=[R_axis(row); 			% range estimate
	Th_axis(col);			% range estimate
	0];   % 				% bias estimate


T=0.01;   % Run Kalman filter in 100 Hz

% Initiate state transition matrix
F= [1 0 0;% 
	0 1 T;
	0 0 1];   
	
% Observation matrix
H=[1 0 0;% 
	0 1 0];                   % 

% Input vector
G=[0;T;0];    

% Noise of input vector, rate gyro noise
GQ=[std_ad_gyro^2];
 
% Initiate covariances for current start estimate
% Do not touch values in covariance before you get the Kalman Filter running
Pe=[0.4^2 0 0; 
	0 (3*pi/180)^2 0;
	0 0 0.01^2];     %

% Observation noise
R=[0.2^2 0;
	0 (3*pi/180)^2];  % Observation noise



% Process noise	
% Q=[0.05^2 0 0; %   change prosess noise on Q
%    0 (0.01*pi/180)^2 0;
%    0 0 0.001^2];

Q=[0.01 0 0; %   change prosess noise on Q
   0 0.01 0;
   0 0 0.01];

nLen=1500;                         % Loop through this number of gyro measurements
mXlog=zeros(length(Xe),nLen);
mPlog=mXlog;
mKlog=zeros(3,nLen);
mDsqrLog = zeros(2,nLen);
mInnovationLog = zeros(1+2*length(H*Xe),nLen);


bAcceptObservation=0;
last_ind=0;
bPlotHough=1<0;


% Counter variable for the number of scans used
nUsedScans=0;
for n=1:nLen 
    U=-rate_ad(n);   % Read next rate gyro readning
    
    % ADD MISSING CODE BELOW
	% Compute prediction
    Xp=F*Xe + G*U;    

	% ADD MISSING CODE BELOW   
	% Compute prediction
    Pp=F*Pe*F.'+G.'*Q*G;     
 
    ind_scan=find(abs(time_tim55-time_ad_gyro(n))<=0.02);
    nAccepts=0;
    if(length(ind_scan)>0)
        if(last_ind<ind_scan(1))
            
            nUsedScans=nUsedScans+1;
            
            figure(3);
            clf
            hold on
            
            vRanges=log_tim55(ind_scan(1),3:273)*0.00106;
            
            % Plot current scan
            x=cos(vAngles).*vRanges;
            y=sin(vAngles).*vRanges;
            plot(x,y,'.');
            axis([-8 8 -6 6]);
            title(sprintf('Time: %.2f sec,   Scan:%d,  Step:%d', t(n), ind_scan(1),n));
            xlabel('x_s (m)');
            ylabel('y_s (m)');
            grid on
            
            % predict observation
			 % ADD MISSING CODE BELOW   
            Zp= H*Xp;	
            
            % Plot predicted line					
            PlotHoughLine(Zp(1), Zp(2),'g-');
            
            [C Th_axis R_axis]=RWhough(vAngles,vRanges,100,0.05);
           
            if(bPlotHough)
                figure(2);
                surf(Th_axis*180/pi,R_axis,C);
            end
            
            % search for three significant Hough peaks
            for k=1:3          			
				% ADD MISSING CODE BELOW   
                
                max_val= max(C(:));  % find max value of peak
                %max_ind=   			% find index of max peak
                [row col]= find(C == max_val); % find row and column of max peak
 
               % Plot observations with read dashed line
                figure(3);
                hold on
                PlotHoughLine(R_axis(row), Th_axis(col),'--r');
                drawnow
                  
                % Remove found max peak
                C(:,max(col-15,1):min(col+15,length(Th_axis)))=0;
                
				%% Max peak is our observation
				% ADD MISSING CODE BELOW  

                Z=[R_axis(row); Th_axis(col)];
                
                % Compute innovation vector
				% ADD MISSING CODE BELOW   
                vInno = Z -H*Xp;
                
                % Compute covariance for innovation vector
				% ADD MISSING CODE BELOW   
                S = H*Pp*H.'+R;
                
                % Compute Mahalanobis distance
				% ADD MISSING CODE BELOW   
                invS = inv(S);
                d_sqr = vInno.'*invS*vInno;
                % Chi squre test for observation
				% ADD MISSING CODE BELOW   
                bAcceptObservation = d_sqr<5.99;          % add expression to accept observaton 95% Chi^2-test. Two degrees of freedom      
                
                % Log and store innovation vector, Mahalanobis distance and innovation covariance
                vInnoLog=[vInno; sqrt(diag(S));t(n)];
                mInnovationLog(:,n)=vInnoLog;
                mDsqrLog(:,n)=[t(n);d_sqr];
                if(bAcceptObservation)
                  break;
                end
            end
        end
        last_ind = ind_scan(1);
    end
    
    if(bAcceptObservation>0)
      % add equations for accept
      % Compute new estimate when accepted observation
    	% ADD MISSING CODE BELOW   
        K= Pp*H.'*(H*Pp*H.'+ R);                    
        Xe = Xp + K*(Z-H*Xp);
        I=eye(size(Xe,1));
        Pe=(I - K*H)*Pp;
        
        % Log Kalman gain
        mKlog(:,n)=[t(n);diag(K)];
		
		nAccepts = nAccepts + 1;		 %  Valid observation found in scan
        disp(sprintf('Accepted observations %d\n',nAccepts));
     else
		% Compute new estimate for rejected observation
    	% ADD MISSING CODE BELOW  
		Pe=Pp;
		Xe=Xp;
		
		% Log Kalman Gain
        mKlog(:,n)=[t(n);0;0];
        % disp(sprintf('No observation at %d\n',ind_scan));
      end
  
    % Log, store, state vector estimate and covariance for state estimate
    mXlog(:,n)=Xe;
    mPlog(:,n)=sqrt(diag(Pe));
    
end

figure
%subplot(2,1,1);
hold on
plot(t(1:nLen),2*mPlog(1,:)+mXlog(1,:),'k-.');
plot(t(1:nLen),-2*mPlog(1,:)+mXlog(1,:),'k-.');
plot(t(1:nLen),mXlog(1,:),'b-');
xlabel('Time (s)');
ylabel('Range (m)');
title('Range estimate');
grid on

figure
%subplot(2,1,2);
hold on
plot(t(1:nLen),(2*mPlog(2,:)+mXlog(2,:))*180/pi,'k-.');
plot(t(1:nLen),(-2*mPlog(2,:)+mXlog(2,:))*180/pi,'k-.');
plot(t(1:nLen),mXlog(2,:)*180/pi,'b-');
xlabel('Time (s)');
ylabel('Angle (deg)');
title('Angle estimate');
grid on

figure
subplot(1,1,1);
hold on
plot(t(1:nLen),mXlog(3,:),'b-');
plot(t(1:nLen),2*mPlog(3,:)+mXlog(3,:),'k-.');
plot(t(1:nLen),-2*mPlog(3,:)+mXlog(3,:),'k-.');
title('Bias estimate');
xlabel('Time (s)');
grid on

figure
%subplot(2,1,1);
hold on
t1=mInnovationLog(size(mInnovationLog,1),:);
t_ind=find(t1>0);
plot(t1(t_ind),mInnovationLog(1,t_ind),'b-');
plot(t1(t_ind),2*mInnovationLog(3,t_ind)+mInnovationLog(1,t_ind)*0,'k-.');
plot(t1(t_ind),-2*mInnovationLog(3,t_ind)+mInnovationLog(1,t_ind)*0,'k-.');
title('Innovation Range');
ylabel('(m)');
xlabel('Time (s)');
grid on

figure
%subplot(2,1,2);
hold on
plot(t1(t_ind),mInnovationLog(2,t_ind)*180/pi,'b-');
plot(t1(t_ind),(2*mInnovationLog(4,t_ind)+mInnovationLog(2,t_ind))*180/pi,'k-.');
plot(t1(t_ind),(-2*mInnovationLog(4,t_ind)+mInnovationLog(2,t_ind))*180/pi,'k-.');
title('Innovation Angle');
ylabel('(deg)');
xlabel('Time (s)');
grid on

figure
subplot(2,1,1);
plot(mKlog(1,:), mKlog(2,:),'b-');
ylabel('gain');
xlabel('Time (s)');
title('Kalman Gain');
grid on

subplot(2,1,2);
plot(mKlog(1,:), mKlog(3,:),'b-');
ylabel('gain');
xlabel('Time (s)');
title('Kalman Gain');
grid on


figure
plot(mDsqrLog(1,:), mDsqrLog(2,:),'b.');
title('Mahalanobis Distance');
ylabel('\chi^2');
xlabel('Time (s)');
grid on


% Print all figures as png and coloured eps files 
nFigs=get(gcf,'Number');   % Get number of last figure
for n=1:nFigs
	figure(n);
	print('-dpng',sprintf('lab2_figure_%d.png',n));
	print('-depsc',sprintf('lab2_figure_%d.eps',n));
end
%}
