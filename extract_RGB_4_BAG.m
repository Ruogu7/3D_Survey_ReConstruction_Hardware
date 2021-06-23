% Parse the ROS BAG file to extract the RGB\Depth Image\IMU data
%
% Prototype: pos2gpx(fname, pos)
% Inputs: fname - file name, with default extension '.gpx'
%         pos - geographic position [lat,lon,hgt] array
% Notes: the steps of loading *.gpx file to Google Earth are
%   1) start Google Earth
%   2) in Google Earth menu: File->Open
%   3) in Open dialog: Select this generated *.gpx file->Open
%
% See also  gpsplot, avpfile.

% Copyright(c) 2009-2014, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 05/06/2021, 14/03/2014

close all; clear all; clc;
tic

%% define
rgb_width = 1280;
rgb_height = 720;
rgb_colors = 3;

depthIM_width = 848;
depthIM_height = 480;

%% 读取BAG文件
bagfile = rosbag('20210623_072340.bag');
rosbag info '20210623_072340.bag'
% 20210610_163434.bag  在cet测试的
% 20210613_082743.bag  在lmars测试的，8:27开始,持续12.4分钟

% 保存 D435i数据包的AvailableTopics
AvailableTopics_D435i = bagfile.AvailableTopics;
AvailableTopics_D435i{1,1}   % data in table
class(AvailableTopics_D435i)   % "table"
toc

rgb_info = select(bagfile,'Topic','device_0/info')

%% 提取RGB图像
% source: https://blog.csdn.net/weixin_42459037/article/details/82772017
rgb_im = select(bagfile,'Topic','device_0/sensor_1/Color_0/image/data');
rgb_im_Structs = readMessages(rgb_im,'DataFormat','struct');
N_rgb = rgb_im.NumMessages;     % RGB影像 帧数
% x = readMessages(rgb_im,1);   % 单帧影像
% class(x)
save_path = strcat('.\RGB_im\');

for i = 1:N_rgb
    RGB_im_s = readMessages(rgb_im,i);   % 单帧影像
    tmp2 = RGB_im_s{1,1}.Data;           % 影像数据
    imageArrayB = tmp2(1:3:end);
    imageArrayG = tmp2(2:3:end);
    imageArrayR = tmp2(3:3:end);
    
    imageMatrixB = rot90(flip(reshape(imageArrayB,[rgb_width, rgb_height]), 1),3);
    imageMatrixG = rot90(flip(reshape(imageArrayG,[rgb_width, rgb_height]), 1),3);
    imageMatrixR = rot90(flip(reshape(imageArrayR,[rgb_width, rgb_height]), 1),3);
    
    % D435i中RGB影像的编码方式是rgb8
    % 之前，我的错误在于，图像的三位矩阵中，以R、G、B的顺序。正确的顺序是B、G、R
    imageMatrix = uint8(zeros(rgb_height, rgb_width, rgb_colors));
    imageMatrix(:,:,1) = imageMatrixB;
    imageMatrix(:,:,2) = imageMatrixG;
    imageMatrix(:,:,3) = imageMatrixR;
    
    name = num2str(i);
    imwrite(imageMatrix,strcat(save_path,name,'.jpg'));
    if mod(i,100)==0
        disp(['RGB影像数据解析，任务处理进度%：', num2str(i/N_rgb*100)]);
    end
end
toc
% 打印空格
disp("\n");

%% 提取深度信息（跟红外影像，同分辨率和频帧）
depthIm_info = select(bagfile,'Topic','device_0/sensor_0/info')
% device_0/sensor_2/info
% source: https://blog.csdn.net/weixin_42459037/article/details/82772017
Depth_data = select(bagfile,'Topic','device_0/sensor_0/Depth_0/image/data');
N_Depth = Depth_data.NumMessages;     % Depth深度影像 帧数
savepath4Depth = strcat('.\DepthData\');
tic
for k = 1:N_Depth
    depth_data_single = readMessages(Depth_data,k);
    depthArray = depth_data_single{1,1}.Data;
    % size(depthArray)
    
    depthArrayLo = depthArray(1:2:end);
    % size(depthArrayLo)
    depthArrayHi = depthArray(2:2:end);
    
    depthMatrixLo = rot90(flip(reshape(depthArrayLo,[depthIM_width, depthIM_height]), 1),3);
    depthMatrixHi = rot90(flip(reshape(depthArrayHi,[depthIM_width, depthIM_height]), 1),3);
    
    depthMatrixLo_U16 = uint16(depthMatrixLo(:,51:640));
    depthMatrixHi_U16 = uint16(depthMatrixHi(:,51:640));
    
    depthMatrix_U16 = depthMatrixLo_U16 + (2^8)*depthMatrixHi_U16;
    
    % figure
    % caxis([0 255]);
    % image(depthMatrixHi_U16,'CDataMapping','scaled');
    % colorbar
    % 存储至image文件中
    name = num2str(k);
    % help:  https://blog.csdn.net/aaa8493077/article/details/102334855
    % 示范： imwrite(data11,'image.png','png','bitdepth',16);
    imwrite(depthMatrix_U16,strcat(savepath4Depth,name,'.png'),'png','bitdepth',16);
    
    % class(depth_data_s )
    %     depth_data_single = depth_data_s{1,1}.Data;
    %     ss = size(depth_data_single,1);
    %
    %     temp_image_vector_1 = depth_data_single(1:size(depth_data_single,1)/2,1)';
    %     temp_image_vector_2 = depth_data_single(size(depth_data_single,1)/2+1:end,1)';
    %
    %     tmp2_depth_left = reshape(temp_image_vector_1,[848,480])';
    %     tmp2_depth_right = reshape(temp_image_vector_2,[848,480])';
    %     name = num2str(k);
    %     imwrite(tmp2_depth_left,strcat(savepath4Depth,name,'_left.jpg'));
    %     imwrite(tmp2_depth_right,strcat(savepath4Depth,name,'_right.jpg'));
    
    if mod(k,500)==0
        disp(['深度数据解析，任务处理进度%：', num2str(k/N_Depth*100)]);
    end
end
toc

% 打印空格
disp("\n");


%% 提取3D点云图像
% source: https://blog.csdn.net/weixin_42459037/article/details/82772017










%% 提取IMU数据
% source: https://blog.csdn.net/weixin_42459037/article/details/82772017

imu_info = select(bagfile,'Topic','device_0/sensor_2/Accel_0/imu/metadata')


Accel_imu_im = select(bagfile,'Topic','device_0/sensor_2/Accel_0/imu/data');
N_Accel_imu = Accel_imu_im.NumMessages;     % IMU数据 帧数
x = readMessages(Accel_imu_im,1);     % 单帧  IMU
x;   %



for j = 1: N_Accel_imu
    Accel_imu_im_s = readMessages(Accel_imu_im,j);     % 单帧 IMU数据
    % Orientation
    % tmp2 = imu_im_s{i,1}     ;           % Cell结构
    Accel_Orientation_matrix(j,1) = Accel_imu_im_s{1,1}.Orientation.X; % Orientation: 4 * 1 (x,y,z,w)
    Accel_Orientation_matrix(j,2) = Accel_imu_im_s{1,1}.Orientation.Y; % Orientation: 4 * 1 (x,y,z,w)
    Accel_Orientation_matrix(j,3) = Accel_imu_im_s{1,1}.Orientation.Z; % Orientation: 4 * 1 (x,y,z,w)
    Accel_Orientation_matrix(j,4) = Accel_imu_im_s{1,1}.Orientation.W; % Orientation: 4 * 1 (x,y,z,w)
    
    Accel_AngularVelocity_matrix(j,1) = Accel_imu_im_s{1,1}.AngularVelocity.X; % 3 * 1 (x,y,z)
    Accel_AngularVelocity_matrix(j,2) = Accel_imu_im_s{1,1}.AngularVelocity.Y; % 3 * 1 (x,y,z)
    Accel_AngularVelocity_matrix(j,3) = Accel_imu_im_s{1,1}.AngularVelocity.Z; % 3 * 1 (x,y,z)
    
    Accel_LinearAcceleration_matrix(j,1) = Accel_imu_im_s{1,1}.LinearAcceleration.X; % 3 * 1 (x,y,z)
    Accel_LinearAcceleration_matrix(j,2) = Accel_imu_im_s{1,1}.LinearAcceleration.Y; % 3 * 1 (x,y,z)
    Accel_LinearAcceleration_matrix(j,3) = Accel_imu_im_s{1,1}.LinearAcceleration.Z; % 3 * 1 (x,y,z)
    
    Accel_OrientationCovariance_matrix(j,:) = [Accel_imu_im_s{1,1}.OrientationCovariance]'; % 9 * 1
    Accel_AngularVelocityCovariance_matrix(j,:) = [Accel_imu_im_s{1,1}.AngularVelocityCovariance]';  % 9 * 1
    Accel_LinearAccelerationCovariance_matrix(j,:) = [Accel_imu_im_s{1,1}.LinearAccelerationCovariance]';  % 9 * 1
    if mod(j,500)==0
        disp(['IMU数据解析，任务处理进度%：', num2str(j/N_Accel_imu*100)]);
    end
end
% save to file in folder
saveMatrxi2txt('.\IMU_file\Accel_Orientation_matrix.txt', Accel_Orientation_matrix)
saveMatrxi2txt('.\IMU_file\Accel_AngularVelocity_matrix.txt', Accel_AngularVelocity_matrix)
saveMatrxi2txt('.\IMU_file\Accel_LinearAcceleration_matrix.txt', Accel_LinearAcceleration_matrix)
saveMatrxi2txt('.\IMU_file\Accel_OrientationCovariance_matrix.txt', Accel_OrientationCovariance_matrix)
saveMatrxi2txt('.\IMU_file\Accel_AngularVelocityCovariance_matrix.txt', Accel_AngularVelocityCovariance_matrix)
saveMatrxi2txt('.\IMU_file\Accel_LinearAccelerationCovariance_matrix.txt', Accel_LinearAccelerationCovariance_matrix)

% Orientation_matrix_file = fopen('.\IMU_file\Orientation_matrix.txt','a');
% fprintf(Orientation_matrix_file,'%d,',Orientation_matrix);   % 注意：%d后有逗号。
% fclose(Orientation_matrix_file);
% AngularVelocity_matrix_file = fopen('.\IMU_file\AngularVelocity_matrix.txt','a');
% fprintf(AngularVelocity_matrix_file,'%d,',AngularVelocity_matrix);   % 注意：%d后有逗号。
% fclose(AngularVelocity_matrix_file);
% LinearAcceleration_matrix_file = fopen('.\IMU_file\LinearAcceleration_matrix.txt','a');
% fprintf(LinearAcceleration_matrix_file,'%d,',LinearAcceleration_matrix);   % 注意：%d后有逗号。
% fclose(LinearAcceleration_matrix_file);
% OrientationCovariance_matrix_file = fopen('.\IMU_file\OrientationCovariance_matrix.txt','a');
% fprintf(OrientationCovariance_matrix_file,'%d,',OrientationCovariance_matrix);   % 注意：%d后有逗号。
% fclose(OrientationCovariance_matrix_file);
% AngularVelocityCovariance_matrix_file = fopen('.\IMU_file\AngularVelocityCovariance_matrix.txt','a');
% fprintf(AngularVelocityCovariance_matrix_file,'%d,',AngularVelocityCovariance_matrix);   % 注意：%d后有逗号。
% fclose(AngularVelocityCovariance_matrix_file);
% LinearAccelerationCovariance_matrix_file = fopen('.\IMU_file\LinearAccelerationCovariance_matrix.txt','a');
% fprintf(LinearAccelerationCovariance_matrix_file,'%d,',LinearAccelerationCovariance_matrix);
% fclose(LinearAccelerationCovariance_matrix_file);

%% 提取Gyro数据
% source: https://blog.csdn.net/weixin_42459037/article/details/82772017

imu_info = select(bagfile,'Topic','device_0/sensor_2/Accel_0/imu/metadata')


Gyro_imu = select(bagfile,'Topic','device_0/sensor_2/Gyro_0/imu/data');
N_Gyro_imu = Gyro_imu.NumMessages;     % IMU数据 帧数
x = readMessages(Gyro_imu,1);     % 单帧  IMU
x;   %



for m = 1: N_Gyro_imu
    Gyro_imu_s = readMessages(Gyro_imu,m);     % 单帧 IMU数据
    % Orientation
    % tmp2 = imu_im_s{i,1}     ;           % Cell结构
    Gyro_imu_Orientation_matrix(m,1) = Gyro_imu_s{1,1}.Orientation.X; % Orientation: 4 * 1 (x,y,z,w)
    Gyro_imu_Orientation_matrix(m,2) = Gyro_imu_s{1,1}.Orientation.Y; % Orientation: 4 * 1 (x,y,z,w)
    Gyro_imu_Orientation_matrix(m,3) = Gyro_imu_s{1,1}.Orientation.Z; % Orientation: 4 * 1 (x,y,z,w)
    Gyro_imu_Orientation_matrix(m,4) = Gyro_imu_s{1,1}.Orientation.W; % Orientation: 4 * 1 (x,y,z,w)
    
    Gyro_imu_AngularVelocity_matrix(m,1) = Gyro_imu_s{1,1}.AngularVelocity.X; % 3 * 1 (x,y,z)
    Gyro_imu_AngularVelocity_matrix(m,2) = Gyro_imu_s{1,1}.AngularVelocity.Y; % 3 * 1 (x,y,z)
    Gyro_imu_AngularVelocity_matrix(m,3) = Gyro_imu_s{1,1}.AngularVelocity.Z; % 3 * 1 (x,y,z)
    
    Gyro_imu_LinearAcceleration_matrix(m,1) = Gyro_imu_s{1,1}.LinearAcceleration.X; % 3 * 1 (x,y,z)
    Gyro_imu_LinearAcceleration_matrix(m,2) = Gyro_imu_s{1,1}.LinearAcceleration.Y; % 3 * 1 (x,y,z)
    Gyro_imu_LinearAcceleration_matrix(m,3) = Gyro_imu_s{1,1}.LinearAcceleration.Z; % 3 * 1 (x,y,z)
    
    Gyro_imu_OrientationCovariance_matrix(m,:) = [Gyro_imu_s{1,1}.OrientationCovariance]'; % 9 * 1
    Gyro_imu_AngularVelocityCovariance_matrix(m,:) = [Gyro_imu_s{1,1}.AngularVelocityCovariance]';  % 9 * 1
    Gyro_imu_LinearAccelerationCovariance_matrix(m,:) = [Gyro_imu_s{1,1}.LinearAccelerationCovariance]';  % 9 * 1
    if mod(m,500)==0
        disp(['IMU数据解析，任务处理进度%：', num2str(m/N_Gyro_imu*100)]);
    end
end
% save to file in folder
saveMatrxi2txt('.\IMU_file\Gyro_imu_Orientation_matrix.txt', Gyro_imu_Orientation_matrix);
saveMatrxi2txt('.\IMU_file\Gyro_imu_AngularVelocity_matrix.txt', Gyro_imu_AngularVelocity_matrix);
saveMatrxi2txt('.\IMU_file\Gyro_imu_LinearAcceleration_matrix.txt', Gyro_imu_LinearAcceleration_matrix);
saveMatrxi2txt('.\IMU_file\Gyro_imu_OrientationCovariance_matrix.txt', Gyro_imu_OrientationCovariance_matrix);
saveMatrxi2txt('.\IMU_file\Gyro_imu_AngularVelocityCovariance_matrix.txt', Gyro_imu_AngularVelocityCovariance_matrix);
saveMatrxi2txt('.\IMU_file\Gyro_imu_LinearAccelerationCovariance_matrix.txt', Gyro_imu_LinearAccelerationCovariance_matrix);

%% 基于加速度和角速度，推导运动轨迹
% 利用工具，推导轨迹
% source: https://ww2.mathworks.cn/help/nav/ref/kinematictrajectory-system-object.html
% 初始化Trajectory
trajectory = kinematicTrajectory('SampleRate',30,'Position',[15,15,15]);
same_size = min(size(Accel_LinearAcceleration_matrix,1), size(Gyro_imu_AngularVelocity_matrix,1));

Gravity_disappear_matrix = [zeros(1,same_size);9.8*ones(1,same_size);zeros(1,same_size)]';
bodyAcceleration = Accel_LinearAcceleration_matrix(1:same_size,:);
bodyAcceleration = bodyAcceleration + Gravity_disappear_matrix;
bodyAngularVelocity = Gyro_imu_AngularVelocity_matrix(1:same_size,:);
[position,orientation,velocity,acceleration,angularVelocity] = trajectory(bodyAcceleration,bodyAngularVelocity);
% 对轨迹，进行可视化
plot3(position(:,1),position(:,2),position(:,3));
position(1:3:end, :)
acceleration(1:3:end, :)
velocity(1:3:end, :)
% 2.1323   13.7645   -8.8212
% 2.0651   13.7837   -8.8504
% 速度13.8m/s，明显高于步行速度，不对。

% 数据记录
position_by_imu = zeros(same_size,3);
velocity_by_imu = zeros(same_size,3);
dt = 1/30;                % 假设时间间隔均匀，为1/30; 
Gravity = [0 9.8 0];      % 重力加速度
body_gesture = [0 0 0];   % 初始姿态
tmp_P =  [0 0 0 ];             % 初始位置    
tmp_V =  [0 0 0 ];             % 初始位置    
linear_acceleration_init = [0.2 -0.2 9.8];
linear_acceleration_init = [0 0 0];
for i_imu = 1:same_size
    % 获取当前观测值
    linear_acceleration_echo = bodyAcceleration(i_imu,:);
    Angular_Velocity_echo = bodyAngularVelocity(i_imu,:);
    
    % t时刻 瞬时加速度
    % Body在世界坐标系统下的加速度 = Body姿态 * Body加速度 - 重力加速度；
    % un_acc_0 = tmp_Q * (acc_0 - tmp_Ba) - estimator.g;
    % 不考虑偏差 tmp_Ba
    un_acc_0 = body_gesture * linear_acceleration_init - Gravity; 
    
    % un_gyr = 0.5 * (gyr_0 + angular_velocity) - tmp_Bg;
    un_gyr = 0.5 * ( gyr_0 + Angular_Velocity_echo ) - tmp_Bg;
    
    % tmp_Q = tmp_Q * Utility::deltaQ(un_gyr * dt);
    % 更新
    body_gesture_echo = body_gesture * Utility::deltaQ(un_gyr * dt);
    
    % t+1时刻 瞬时加速度
    % un_acc_1 = tmp_Q * (linear_acceleration - tmp_Ba) - estimator.g;
    % 不考虑偏差 tmp_Ba
    un_acc_1 = body_gesture_echo * linear_acceleration_echo - [0 0 9.8];
    
    % 求平均加速度
    % un_acc = 0.5 * (un_acc_0 + un_acc_1);
    un_acc = 0.5 * (un_acc_0 + un_acc_1);
    
    % 当前瞬时位置
    % tmp_P = tmp_P + dt * tmp_V + 0.5 * dt * dt * un_acc;
    tmp_P = tmp_P + dt * tmp_V + 0.5 * dt * dt * un_acc;
    
    % 当前瞬时速度
    % tmp_V = tmp_V + dt * un_acc;
    tmp_V = tmp_V + dt * un_acc;
    
    % 将当前状态，作为下一阶段的前状态；
    % acc_0 = linear_acceleration;
    % gyr_0 = angular_velocity;
    linear_acceleration_init = linear_acceleration_echo;
    linear_acceleration_init = Angular_Velocity_echo;
end




% 测试数据
% 假如一个物体，处于匀速直线运动状态，加速度各项为零，角速度
% bodyAcceleration = zeros(10,3);
% bodyAngularVelocity = ones(10,3);
% trajectory = kinematicTrajectory('SampleRate',30,'Position',[15,15,15]);
% [position,orientation,velocity,acceleration,angularVelocity] = trajectory(bodyAcceleration,bodyAngularVelocity);










