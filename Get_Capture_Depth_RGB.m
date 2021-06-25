function Get_Capture_Depth_RGB( folder_project, ID_Station )
% Link: https://github.com/IntelRealSense/librealsense/blob/master/wrappers/matlab/depth_example.m
% Link:
% ��D435i����У���ȡһ֡Frame�е���Ⱥ�RGBӰ��
% Input: number(��Ϊ���)
% Output: ���Ӱ�񣬴���ļ���
% example��
%   {��Ч�����豸}   Get_Capture_Depth_RGB( '.\project1\', 2 )
% 
% Author: ruogu7, 380545156@qq.com
% See also  gpsplot, avpfile.
% Copyright(c) 2019-2021, by Shenghua Chen, All rights reserved.
% China Nanhu Academy of CETC, Jiaxing, P.R.China
% 30/06/2021, 14/03/2019

%% Make Pipeline object to manage streaming
pipe = realsense.pipeline();
% Make Colorizer object to prettify depth output
colorizer = realsense.colorizer();

% Start streaming on an arbitrary camera with default settings
profile = pipe.start();

% Get streaming device's name
dev = profile.get_device();
name = dev.get_info(realsense.camera_info.name);

% Get frames. We discard the first couple to allow
% the camera time to settle
tic
for i = 1:100
    fs = pipe.wait_for_frames();
end
toc

% Stop streaming
pipe.stop();

%% info of save path


%% �ɼ������Ϣ
% Select depth frame
depth = fs.get_depth_frame();

% Colorize depth frame
color = colorizer.colorize(depth);

% Get actual data and convert into a format imshow can use
% (Color data arrives as [R, G, B, R, G, B, ...] vector)
data = color.get_data();
depth_img = permute(reshape(data',[3,color.get_width(),color.get_height()]),[3 2 1]);

% save depth image
imwrite(depth_img, strcat(folder_project, num2str(ID_Station),'_depth.png'),'png','bitdepth',16);

% Display image
subplot(1,2,1)
imshow(depth_img);
title(sprintf("Colorized depth frame from %s", name));

%% �ɼ�RGBӰ����Ϣ
% Select depth frame
rgb_image = fs.get_color_frame();

% Get actual data and convert into a format imshow can use
% (Color data arrives as [R, G, B, R, G, B, ...] vector)
rgb_image_data = rgb_image.get_data();

imageArrayB = rgb_image_data(1:3:end);
imageArrayG = rgb_image_data(2:3:end);
imageArrayR = rgb_image_data(3:3:end);

imageMatrixB = rot90(flip(reshape(imageArrayB,[rgb_image.get_width(), rgb_image.get_height()]), 1),3);
imageMatrixG = rot90(flip(reshape(imageArrayG,[rgb_image.get_width(), rgb_image.get_height()]), 1),3);
imageMatrixR = rot90(flip(reshape(imageArrayR,[rgb_image.get_width(), rgb_image.get_height()]), 1),3);

% D435i��RGBӰ��ı��뷽ʽ��rgb8
% ֮ǰ���ҵĴ������ڣ�ͼ�����λ�����У���R��G��B��˳����ȷ��˳����B��G��R
imageMatrix = uint8(zeros(rgb_image.get_height(), rgb_image.get_width(), 3));
imageMatrix(:,:,1) = imageMatrixB;
imageMatrix(:,:,2) = imageMatrixG;
imageMatrix(:,:,3) = imageMatrixR;

% save rgb image
imwrite(imageMatrix,strcat(folder_project, num2str(ID_Station),'_rgb.jpg'));

% Display image
subplot(1,2,2)
imshow(imageMatrix);
title(sprintf("RGB Image frame from %s", name));

toc

%% �ɼ�IMU����


end

