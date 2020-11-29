close all;
clear all;
clc;



%% load other dataset
load('data_v3.mat');
GNSSData = gps';
ACCData = imu_acc';
GYROData = imu_angvel';
Q_1 = imu_orientation';

%% Set static data
tf = 300;
dt = 0.01;	% Gyro and Acc frequency

%% matrices
C = [	1 0 0 0 0 0 0 0 0 0;	% 6x10 matrix that defines the parts of the state vector that we measure
		0 1 0 0 0 0 0 0 0 0;
		0 0 1 0 0 0 0 0 0 0;
		0 0 0 0 0 0 1 0 0 0;
		0 0 0 0 0 0 0 1 0 0;
		0 0 0 0 0 0 0 0 1 0;
		0 0 0 0 0 0 0 0 0 1	];


R = eye(7)*0.1;		% 6x6 matrix that defines the sensor variance
Q = eye(10)*0,1;	% 10x10 matrix that defines the model variance  - the uncertainty in the model

g = [0 0 9.82]';

%% Kalman Filter
[p n] = size(GNSSData)

q1 = 0;
q2 = 0;
q3 = 0;
q4 = 0;

%% Initial quess
xHat = [GNSSData(1,1) GNSSData(2,1) GNSSData(3,1) 0 0 0 0 0 0 0]';
Pplus = eye(10)*10;	% 10x10 matrix of covariances - initial guess

%% for plotting
xHatArray = xHat;
xHatArray2 = xHat;
xArray = [0 0 0 0 0 0 0 0 0 0]';
bArray = [0 0 0 0 0 0 0 0 0 0]';
kArray = [];
%% Loop variables
temp = 2;
theta = 0;
phi = 0;
psy = 0;
myVal = [1 1 1]';
H = eye(10);

%% The Kalman filter
for i=1:n-1
	for k=1:10
	%% Math before filter
% 		dt = ACCTime(temp)- ACCTime(temp-1);
		% Extract angles from data
		theta = theta + GYROData(1,temp)*dt;	%y - pitch
		phi = phi + GYROData(2,temp)*dt;	%x - roll
		psy = psy + GYROData(3,temp)*dt;	%z - yaw

		Q0 = cos(phi*0.5)*cos(theta*0.5)*cos(psy*0.5)+sin(phi*0.5)*sin(theta*0.5)*sin(psy*0.5);
		Q1 = sin(phi*0.5)*cos(theta*0.5)*cos(psy*0.5)-cos(phi*0.5)*sin(theta*0.5)*sin(psy*0.5);
		Q2 = cos(phi*0.5)*sin(theta*0.5)*cos(psy*0.5)+sin(phi*0.5)*cos(theta*0.5)*sin(psy*0.5);
		Q3 = cos(phi*0.5)*cos(theta*0.5)*sin(psy*0.5)-sin(phi*0.5)*sin(theta*0.5)*cos(psy*0.5);

		q0 = xHat(7);
		q1 = xHat(8);
		q2 = xHat(9);
		q3 = xHat(10);

		% Quaternion rotation matrix
 		Rq1 = [	(q0^2+q1^2-q2^2-q3^2) 2*(q1*q2-q0*q3) 2*(q0*q2+q1*q3);
 				2*(q1*q2+q0*q3) (q0^2-q1^2+q2^2-q3^2) 2*(q1*q3-q0*q1);
 				2*(q1*q3-q0*q2) 2*(q0*q1+q2*q3) (q0^2-q1^2-q2^2+q3^2)	];
		%Rq = quat2rotm([q0,[q1,q2,q3]])
		Rq = [	1 0 0;
				0 -1 0;
				0 0 -1	];
	
		% Omega matrix
		Wx = GYROData(1,temp)*dt;
		Wy = GYROData(2,temp)*dt;
		Wz = GYROData(3,temp)*dt;
		Omega = [	0 Wz -Wy Wx;
					-Wz 0 Wx Wy;
					Wy -Wx 0 Wz;
					-Wx -Wy -Wz 0	];
		nW = sqrt(Wx^2+Wy^2+Wz^2);

		% Jacobian
% 		J = [	1 0 0 dt 0 0 0 0 0 0;
% 				0 1 0 0 dt 0 0 0 0 0;
% 				0 0 1 0 0 dt 0 0 0 0;
% 				0 0 0 1 0 0 0 0 0 0;
% 				0 0 0 0 1 0 0 0 0 0;
% 				0 0 0 0 0 1 0 0 0 0;
% 				0 0 0 0 0 0	cos((dt*nW)/2) cos((dt*nW)/2)-(Wz*sin((dt*nW)/2))/nW cos((dt*nW)/2)-(Wy*sin((dt*nW)/2))/nW cos((dt*nW)/2)-(Wx*sin((dt*nW)/2))/nW;
% 				0 0 0 0 0 0 cos((dt*nW)/2)+(Wz*sin((dt*nW)/2))/nW cos((dt*nW)/2) cos((dt*nW)/2)-(Wx*sin((dt*nW)/2))/nW cos((dt*nW)/2)-(Wy*sin((dt*nW)/2))/nW;
% 				0 0 0 0 0 0 cos((dt*nW)/2)-(Wy*sin((dt*nW)/2))/nW cos((dt*nW)/2)+(Wx*sin((dt*nW)/2))/nW cos((dt*nW)/2) cos((dt*nW)/2)-(Wz*sin((dt*nW)/2))/nW;
% 				0 0 0 0 0 0 cos((dt*nW)/2)+(Wx*sin((dt*nW)/2))/nW cos((dt*nW)/2)+(Wy*sin((dt*nW)/2))/nW cos((dt*nW)/2)+(Wz*sin((dt*nW)/2))/nW cos((dt*nW)/2)	];

			
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

		% Construct Q from insane formula
		QTemp = (cos(0.5*dt*nW)*eye(4)+(1/nW)*sin(0.5*dt*nW)*Omega);

		% Update u
		u = [(dt^2/2) (dt^2/2) (dt^2/2) dt dt dt 0 0 0 0]';
		% Update B
 		B = [Rq1*(ACCData(:,temp)-g); Rq1*(ACCData(:,temp)-g); diag((cos(0.5*dt*nW)*eye(4)+(1/nW)*sin(0.5*dt*nW)*Omega))];
		%B = [(ACCData(:,temp)-g); (ACCData(:,temp)-g); diag((cos(0.5*dt*nW)*eye(4)+(1/nW)*sin(0.5*dt*nW)*Omega))];
		
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

		%% Predict step
		xHat = F*xHat + B.*u;
		Pmin = J*Pplus*J'+Q;
		temp = temp + 1;
		
	end 
	%% Update step
	% Create x - Values in x-vector is measured.
	GPS = Rq*[GNSSData(1,i) GNSSData(2,i) GNSSData(3,i)]';
    Rq2 = [1 1 1 0;
           1 1 1 0;
           1 1 1 0;
           0 0 0 0];
    Quat = Rq2*[Q_1(1,temp) Q_1(2,temp) Q_1(3,temp) Q_1(4,temp)]';
	x = [GPS(1) GPS(2) GPS(3) 0 0 0 Quat(1) Quat(2) Quat(3) Q(4)];
	
	% Create y from x
	y = C*x';
	% Kalman gain
	K = Pmin * C' * inv(C*Pmin*C'+R);
	Pplus = (eye(10)-K*C) * Pmin;
	xHat = xHat + K*(y - C*xHat);
	
	%% Plot data update
	xHatArray = [xHatArray xHat];
	xArray = [xArray x'];
end


temp;
dt = 1;
%% Graphs

figure(1)
tArray = (1:length(xHatArray))./100;
tGPSArray = (1:length(gps))./10;
figure(1)
subplot(3,1,1)
plot(xHatArray(1,:),':','LineWidth',3);
hold on;
plot(gps(:,1));
hold off;
ylabel('x $[m/s]$', 'Interpreter', 'latex');
grid on
legend("Estimate", "Ground Truth", 'Location', 'ne', 'Interpreter', 'latex')
subplot(3,1,2)
plot(xHatArray(2,:),':','LineWidth',3);
hold on;
plot( -gps(:,2));
hold off;
ylabel('y $[m/s]$', 'Interpreter', 'latex');
grid on
subplot(3,1,3)
plot(xHatArray(3,:),':','LineWidth',3);
hold on;
plot(-gps(:,3));
hold off;
xlabel('Time $[sec]$', 'Interpreter', 'latex');
ylabel('z $[m/s]$', 'Interpreter', 'latex');
grid on

figure(2)
% Qarternions
subplot(4,1,1)
plot(xHatArray(7,:),':','LineWidth',3);
hold on;
plot(xArray(7,:));
hold off;
ylabel('Quaternion 1', 'Interpreter', 'latex');
legend("Estimated", "Ground Truth", 'Interpreter', 'latex')
grid on
ylim([0 1.1])

subplot(4,1,2)
plot(xHatArray(8,:),':','LineWidth',3);
hold on;
plot(xArray(8,:));
hold off;
ylabel('Quaternion 2', 'Interpreter', 'latex');
grid on

subplot(4,1,3)
plot(xHatArray(9,:),':','LineWidth',3);
hold on;
plot(xArray(9,:));
hold off;
ylabel('Quaternion 3', 'Interpreter', 'latex');
grid on

subplot(4,1,4)
plot(xHatArray(10,:),':','LineWidth',3);
hold on;
plot(xArray(10,:));
hold off;
xlabel('Time $[sec]$', 'Interpreter', 'latex');
ylabel('Quaternion 4', 'Interpreter', 'latex');
grid on
