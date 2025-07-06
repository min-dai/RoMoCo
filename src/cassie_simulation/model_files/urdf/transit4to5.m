%% check kinematics

xyaxes = [0 0 -1 1 0 0];
x = xyaxes(1:3)';
y = xyaxes(4:end)';
z = cross(x,y);
R = [x y z]
    
eul = rotm2eul(R,'zyx')

flip(eul)

Rcontact =

   -0.7660         0   -0.6428
    0.6428         0   -0.7660
         0   -1.0000         0
         
         

close all
cassiev4 = importrobot('cassieRigid.urdf');
cassiev4.DataFormat = 'column';
cassiev5 = importrobot('cassieRigid_new.urdf');
cassiev5.DataFormat = 'column';

q = homeConfiguration(cassiev4);


getTransform(cassiev4,q,'LeftToePitchLink')
getTransform(cassiev5,q,'LeftToePitchLink','LeftTarsusPitchLink')


figure
show(cassiev4,q)

figure
show(cassiev5,zeros(12,1))

%% dynamics
mujoco_order = [
0.00017388 0.00016793 0.00033261 0.00011814 -1.36e-06 4e-07

];

formatSpec = '      <inertia ixx="%e" iyy="%e " izz="%e" ixy="%e" ixz="%e" iyz="%e"/> \n';

clc
fprintf(formatSpec,mujoco_order')


    