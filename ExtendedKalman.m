close all;
clear all;
clc;

% %% Load dataset
% load("GNSSINS.mat");
% GNSSData = in_data.GNSS.pos_ned;
% GNSSTime = in_data.GNSS.t;
% 
% ACCData = in_data.IMU.acc;
% ACCTime = in_d11311ata.IMU.t;
% 
% GYROData = in_data.IMU.gyro;
% GYROTime = in_data.IMU.t;
% 
% SPEEDData = in_data.SPEEDOMETER.speed;
% SPEEDTime = in_data.SPEEDOMETER.t;


imu = rossubscriber('/drone/sensors/imu_grav', @imuhandler);
gnss = rossubscriber('/drone/sensors/gps', @gnssHandler);


%% load other dataset
%load('gazebo_data.mat');
%GNSSData= [msg.Pose.Pose.Position.X, msg.Pose.Pose.Position.Y, msg.Pose.Pose.Position.Z]';% data_gps';
%GYROData= [msg.AngularVelocity.X msg.AngularVelocity.Y msg.AngularVelocity.Z]';
%ACCData = [msg.LinearAcceleration.X msg.LinearAcceleration.Y -msg.LinearAcceleration.Z]';%data_imu(:,1:3)';
 %data_imu(:,4:6)';
%% Set static data
tf = 300;
global dt;
dt = 0.01;	% Gyro and Acc frequency

%% matrices
global C;
C = [	1 0 0 0 0 0 0 0 0 0;	% 6x10 matrix that defines the parts of the state vector that we measure
		0 1 0 0 0 0 0 0 0 0;
		0 0 1 0 0 0 0 0 0 0;
		0 0 0 0 0 0 1 0 0 0;
		0 0 0 0 0 0 0 1 0 0;
		0 0 0 0 0 0 0 0 1 0;
		0 0 0 0 0 0 0 0 0 1	];

global R;
R = eye(7)*0.1;		% 6x6 matrix that defines the sensor variance
global Q;
Q = eye(10)*0,1;	% 10x10 matrix that defines the model variance  - the uncertainty in the model
global g;
g = [0 0 9.80]';

%% Kalman Filter
%[p n] = size(GNSSData)
global q1;
global q2;
global q3;
global q4;
q1 = 0;
q2 = 0;
q3 = 0;
q4 = 0;

%% Initial quess
global xHat;
xHat = [0 0 0 0 0 0 0 0 0 0]'
global Pplus;
Pplus = eye(10)*10;	% 10x10 matrix of covariances - initial guess

%% for plotting
xHatArray = xHat;
xHatArray2 = xHat;
xArray = [0 0 0 0 0 0 0 0 0 0]';
bArray = [0 0 0 0 0 0 0 0 0 0]';
kArray = [];
%% Loop variables
temp = 2;
global theta;
theta = 0;
global phi;
phi = 0;
global psy;
psy = 0;
global Pmin;
Pmin = 0;
global H;
H = eye(10);
global old_time;
old_time = 0;
%% The Kalman filter
%for i=1:n-1
	%for k=1:10
	%% Math before filter
        function imuhandler(~,msg)
        time = msg.Header.Stamp.Sec;
        global old_time
        global dt;
 		%dt = time-old_time;
        GYRODataX = msg.AngularVelocity.X;
        GYRODataY = msg.AngularVelocity.Y;
        GYRODataZ = msg.AngularVelocity.Z;
		% Extract angles from data
        global theta;
        global phi;
        global psy;
		theta = theta + GYRODataX*dt;	%y - pitch
		phi = phi + GYRODataY*dt;	%x - roll
		psy = psy + GYRODataZ*dt;	%z - yaw
        
        global Q0;
        global Q1;
        global Q2;
        global Q3;
		Q0 = cos(phi*0.5)*cos(theta*0.5)*cos(psy*0.5)+sin(phi*0.5)*sin(theta*0.5)*sin(psy*0.5);
		Q1 = sin(phi*0.5)*cos(theta*0.5)*cos(psy*0.5)-cos(phi*0.5)*sin(theta*0.5)*sin(psy*0.5);
		Q2 = cos(phi*0.5)*sin(theta*0.5)*cos(psy*0.5)+sin(phi*0.5)*cos(theta*0.5)*sin(psy*0.5);
		Q3 = cos(phi*0.5)*cos(theta*0.5)*sin(psy*0.5)-sin(phi*0.5)*sin(theta*0.5)*cos(psy*0.5);
        global q0;
        global q1;
        global q2;
        global q3;
        global xHat;
        global g;
		q0 = xHat(7);
		q1 = xHat(8);
		q2 = xHat(9);
		q3 = xHat(10);

		% Quaternion rotation matrix
		Rq = [	(q0^2+q1^2-q2^2-q3^2) 2*(q1*q2-q0*q3) 2*(q0*q2+q1*q3);
				2*(q1*q2+q0*q3) (q0^2-q1^2+q2^2-q3^2) 2*(q1*q3-q0*q1);
				2*(q1*q3-q0*q2) 2*(q0*q1+q2*q3) (q0^2-q1^2-q2^2+q3^2)	];
		%Rq = quat2rotm([q0,[q1,q2,q3]])
	
		% Omega matrix
		Wx = GYRODataX;
		Wy = GYRODataY;
		Wz = GYRODataZ;
		Omega = [	0 Wz -Wy Wx;
					-Wz 0 Wx Wy;
					Wy -Wx 0 Wz;
					-Wx -Wy -Wz 0	];
		nW = sqrt(Wx^2+Wy^2+Wz^2);
			
		J = [	1 0 0 dt 0 0 0 0 0 0;
				0 1 0 0 dt 0 0 0 0 0;
				0 0 1 0 0 dt 0 0 0 0;
				0 0 0 1 0 0 0 0 0 0;
				0 0 0 0 1 0 0 0 0 0;
				0 0 0 0 0 1 0 0 0 0;
				0 0 0 0 0 0 cos((dt*nW)/2) cos((dt*nW)/2)-(Wz*sin((dt*nW)/2))/nW cos((dt*nW)/2)-(Wy*sin((dt*nW)/2))/nW cos((dt*nW)/2)-(Wx*sin((dt*nW)/2))/nW;
				0 0 0 0 0 0 cos((dt*nW)/2)+(Wz*sin((dt*nW)/2))/nW cos((dt*nW)/2) cos((dt*nW)/2)-(Wx*sin((dt*nW)/2))/nW cos((dt*nW)/2)-(Wy*sin((dt*nW)/2))/nW;
				0 0 0 0 0 0 cos((dt*nW)/2)-(Wy*sin((dt*nW)/2))/nW cos((dt*nW)/2)+(Wx*sin((dt*nW)/2))/nW cos((dt*nW)/2) cos((dt*nW)/2)-(Wz*sin((dt*nW)/2))/nW;
				0 0 0 0 0 0 cos((dt*nW)/2)+(Wx*sin((dt*nW)/2))/nW cos((dt*nW)/2)+(Wy*sin((dt*nW)/2))/nW cos((dt*nW)/2)+(Wz*sin((dt*nW)/2))/nW cos((dt*nW)/2)	];
		% Update u
		u = [(msg.LinearAcceleration.X-g(1)) (msg.LinearAcceleration.Y-g(2)) (msg.LinearAcceleration.Z-g(3)) msg.LinearAcceleration.X msg.LinearAcceleration.Y (msg.LinearAcceleration.Z-g(3)) 0 0 0 0]';
		% Update B
		B = [(dt*dt)/2 (dt*dt)/2 (dt*dt)/2 dt dt dt 0 0 0 0]';
		
		% F
		F = [	1 0 0 dt 0 0 0 0 0 0;
				0 1 0 0 dt 0 0 0 0 0;
				0 0 1 0 0 dt 0 0 0 0;
				0 0 0 1 0 0 0 0 0 0;
				0 0 0 0 1 0 0 0 0 0;
				0 0 0 0 0 1 0 0 0 0;
				0 0 0 0 0 0 cos((dt*nW)/2) (Wz*sin((dt*nW)/2))/nW -(Wy*sin((dt*nW)/2))/nW (Wx*sin((dt*nW)/2))/nW;
				0 0 0 0 0 0 -(Wz*sin((dt*nW)/2))/nW cos((dt*nW)/2) (Wx*sin((dt*nW)/2))/nW (Wy*sin((dt*nW)/2))/nW;
				0 0 0 0 0 0 (Wy*sin((dt*nW)/2))/nW -(Wx*sin((dt*nW)/2))/nW cos((dt*nW)/2) (Wz*sin((dt*nW)/2))/nW;
				0 0 0 0 0 0 -(Wx*sin((dt*nW)/2))/nW -(Wy*sin((dt*nW)/2))/nW -(Wz*sin((dt*nW)/2))/nW cos((dt*nW)/2)	];
        global Pplus;
        global Q;
        global Pmin;
		%% Predict step
		xHat = F*xHat + B.*u;
		Pmin = J*Pplus*J'+Q;
        global old_time;
        old_time = time;
        end 
function gnssHandler(~, msg)
	%% Update step
	% Create x - Values in x-vector is measured.
	x = [msg.Pose.Pose.Position.X msg.Pose.Pose.Position.Y -msg.Pose.Pose.Position.Z 0 0 0 msg.Pose.Pose.Orientation.X msg.Pose.Pose.Orientation.Y msg.Pose.Pose.Orientation.Z msg.Pose.Pose.Orientation.W];
	global C;
    global Pplus;
    global R
    global Pmin;
    global xHat;
	% Create y from x
	y = C*x';
	% Kalman gain
	K = Pmin * C' * inv(C*Pmin*C'+R);
	Pplus = (eye(10)-K*C) * Pmin;
	xHat = xHat + K*(y - C*xHat);
    xHat

end


