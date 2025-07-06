classdef H1Log < handle
    
    properties
        nConfigSpace = 18+39; % 18 for legs, 39 for arms
        nContactF = 6;
        path = [getenv('HOME') '/ROBOTLOG/H1/']
        
        inair_output_list = {'LeftFootx','LeftFooty','LeftFootz','LeftFootPitch','LeftFootYaw','RightFootx','RightFooty','RightFootz','RightFootPitch','RightFootYaw'};
        
        stand_output_list = {'COMx','COMy','COMz','PelvisRoll','PelvisPitch','PelvisYawDiff'}
        
        walk_output_list = {'xCOM','yCOM','zCOM',...
            'stanceHipYaw','deltaPitch','deltaRoll',...
            'swingStepx','swingStepy','swingStepz',...
            'swingHipYaw','deltaSwingFoot'};


        q_list = {'BasePosX','BasePosY','BasePosZ','BaseRotZ','BaseRotY','BaseRotX',...
            'LeftHipYaw','LeftHipPitch','LeftHipRoll','LeftKneePitch','LeftAnklePitch','LeftAnkleRoll',...
            'RightHipYaw','RightHipPitch','RightHipRoll','RightKneePitch','RightAnklePitch','RightAnkleRoll'}
        
        motor_list = {'LeftHipPitch','LeftHipRoll','LeftHipYaw','LeftKneePitch','LeftAnklePitch','LeftAnkleRoll',...
            'RightHipPitch','RightHipRoll','RightHipYaw','RightKneePitch','RightAnklePitch','RightAnkleRoll'};
        
        joint_max = [pi*ones(1,6) ,...
            0.3927   0.3927  1.3963  -0.6458    2.9671   -0.5236 ,...
            0.3927   0.3927  1.3963  -0.6458    2.9671   -0.5236   ]
        joint_min = [-pi*ones(1,6) ,...
            -0.3927 -0.3927 -0.8727  -2.8623    0.8727 -2.4435 ,...
            -0.3927 -0.3927 -0.8727  -2.8623    0.8727 -2.4435 ]
        torque_bound = [112.5000  112.5000  195.2000  195.2000   45.0000  112.5000  112.5000  195.2000  195.2000   45.0000];
    end
    
    methods
        function  obj =   H1Log()
        end
        
        function plotInAir(obj)
            
            fileID = fopen( [obj.path, 'logInAir.bin']);
            raw = fread(fileID,'float');
            
            nY = 12;
            LengthVec = [1,obj.nConfigSpace, obj.nConfigSpace,12,17,nY,nY,nY,nY,nY];
            
            N = floor(length(raw) / sum(LengthVec));  % Number of samples
            
            [t, q,dq,u_leg,u_arm, ya,dya,yd,dyd,d2yd] = obj.readRaw(raw, N, LengthVec);
            

            output_list = {'LeftFootx','LeftFooty','LeftFootz','LeftFootYaw','LeftFootPitch','LeftFootRoll',...
                        'RightFootx','RightFooty','RightFootz','RightFootYaw','RightFootPitch','RightFootRoll'};


            %plot ya yd
            figure
            tiledlayout(2,6);
            for i=1:nY
                nexttile; plot(t, ya(i,:));  hold on; plot(t, yd(i,:),'k-'); title(output_list{i}); grid on;
            end

            %plot u
            figure
            tiledlayout(2,6);
            for i = 1:12
                nexttile; plot(t, u_leg(i,:)); title(obj.motor_list{i}); xlabel('Time (s)'); ylabel('Torque (Nm)'); grid on;
            end

            plotJointPos(obj, t, q)
            
            
        end
        

        function plotStand(obj)

            fileID = fopen( [obj.path, 'logStand.bin']);
            raw = fread(fileID,'float');
            
            nY = 6;
            LengthVec = [1,obj.nConfigSpace, obj.nConfigSpace,12,39,nY,nY,nY,nY,nY];
            
            N = floor(length(raw) / sum(LengthVec));  % Number of samples
            
            [t, q,dq,u_leg,u_arm, ya,dya,yd,dyd,d2yd] = obj.readRaw(raw, N, LengthVec);
            

            output_list = {'COMx','COMy','COMz','PelvisYawDiff','PelvisPitch','PelvisRoll'};


            %plot ya yd
            figure
            tiledlayout(1,6);
            for i=1:nY
                nexttile; plot(t, ya(i,:));  hold on; plot(t, yd(i,:),'k-'); title(output_list{i}); grid on;
            end

            %plot u
            figure
            tiledlayout(2,6);
            for i = 1:12
                nexttile; plot(t, u_leg(i,:)); title(obj.motor_list{i}); xlabel('Time (s)'); ylabel('Torque (Nm)'); grid on;
            end

            plotJointPos(obj, t, q)
        end

        function plotWalk(obj)
            fileID = fopen( [obj.path, 'logWalk.bin']);
            raw = fread(fileID,'float');
            
            nY = 10;
            LengthVec = [1,obj.nConfigSpace, obj.nConfigSpace,12,39,nY,nY,nY,nY,nY,3,3,3,3,1,1];
            
            N = floor(length(raw) / sum(LengthVec));  % Number of samples
            
            [t, q,dq,u_leg,u_arm, ya,dya,yd,dyd,d2yd, pCOM, vCOM, Lcom, Lpivot,vdx,vdy] = obj.readRaw(raw, N, LengthVec);
            

            output_list = {'zCOM',...
            'stanceHipYaw','basePitch','baseRoll',...
            'swingStepx','swingStepy','swingStepz',...
            'swingHipYaw','SwingDeltaPitch', 'SwingDeltaRoll'};


            %plot ya yd
            figure
            tiledlayout(2,5);
            for i=1:nY
                nexttile; plot(t, ya(i,:));  hold on; plot(t, yd(i,:),'k-'); title(output_list{i}); grid on;
            end

            %plot u
            figure
            tiledlayout(2,6);
            for i = 1:12
                nexttile; plot(t, u_leg(i,:)); title(obj.motor_list{i}); xlabel('Time (s)'); ylabel('Torque (Nm)'); grid on;
            end

            plotJointPos(obj, t, q)


            %plot COM
            figure
            tiledlayout(3,3);
            nexttile; plot(t, pCOM(1,:)); title('COM X'); xlabel('Time (s)'); ylabel('X (m)'); grid on;
            nexttile; plot(t, pCOM(2,:)); title('COM Y'); xlabel('Time (s)'); ylabel('Y (m)'); grid on;
            nexttile; plot(t, pCOM(3,:)); title('COM Z'); xlabel('Time (s)'); ylabel('Z (m)'); grid on;
            nexttile; plot(t, vCOM(1,:)); hold on; plot(t, vdx); title('COM Vel X'); xlabel('Time (s)'); legend('actual','desired'); grid on;
            nexttile; plot(t, vCOM(2,:)); hold on; plot(t, vdy); title('COM Vel Y'); xlabel('Time (s)'); legend('actual','desired'); grid on;
            nexttile; plot(t, vCOM(3,:)); title('COM Vel Z'); xlabel('Time (s)'); ylabel('Z (m/s)'); grid on;
            nexttile; plot(t, Lpivot(1,:)); title('Lpivot X'); xlabel('Time (s)'); ylabel('X (m)'); grid on;
            nexttile; plot(t, Lpivot(2,:)); title('Lpivot Y'); xlabel('Time (s)'); ylabel('Y (m)'); grid on;
            nexttile; plot(t, Lpivot(3,:)); title('Lpivot Z'); xlabel('Time (s)'); ylabel('Z (m)'); grid on;

        end


        function plotJointPos(obj, t, q)
            base_pos = q(1:3,:);
            base_rot = q(4:6,:);

            figure;
            tiledlayout(3,2);
            nexttile; plot(t, base_pos(1,:)); title('Base Position X'); xlabel('Time (s)'); ylabel('X (m)'); grid on;
            nexttile; plot(t, base_rot(1,:)); title('Base Rotation Z'); xlabel('Time (s)'); ylabel('Yaw (rad)'); grid on;
            nexttile; plot(t, base_pos(2,:)); title('Base Position Y'); xlabel('Time (s)'); ylabel('Y (m)'); grid on;
            nexttile; plot(t, base_rot(2,:)); title('Base Rotation Y'); xlabel('Time (s)'); ylabel('Pitch (rad)'); grid on;
            nexttile; plot(t, base_pos(3,:)); title('Base Position Z'); xlabel('Time (s)'); ylabel('Z (m)'); grid on;
            nexttile; plot(t, base_rot(3,:)); title('Base Rotation X'); xlabel('Time (s)'); ylabel('Roll (rad)'); grid on;

            figure
            tiledlayout(2,6);
            nexttile; plot(t, q(7,:)); title('Left Hip Pitch'); xlabel('Time (s)'); ylabel('Angle (rad)'); grid on;
            nexttile; plot(t, q(8,:)); title('Left Hip Roll'); xlabel('Time (s)'); ylabel('Angle (rad)'); grid on;
            nexttile; plot(t, q(9,:)); title('Left Hip Yaw'); xlabel('Time (s)'); ylabel('Angle (rad)'); grid on;
            nexttile; plot(t, q(10,:)); title('Left Knee Pitch'); xlabel('Time (s)'); ylabel('Angle (rad)'); grid on;
            nexttile; plot(t, q(11,:)); title('Left Ankle Pitch'); xlabel('Time (s)'); ylabel('Angle (rad)'); grid on;
            nexttile; plot(t, q(12,:)); title('Left Ankle Roll'); xlabel('Time (s)'); ylabel('Angle (rad)'); grid on;
            nexttile; plot(t, q(13,:)); title('Right Hip Pitch'); xlabel('Time (s)'); ylabel('Angle (rad)'); grid on;
            nexttile; plot(t, q(14,:)); title('Right Hip Roll'); xlabel('Time (s)'); ylabel('Angle (rad)'); grid on;
            nexttile; plot(t, q(15,:)); title('Right Hip Yaw'); xlabel('Time (s)'); ylabel('Angle (rad)'); grid on;
            nexttile; plot(t, q(16,:)); title('Right Knee Pitch'); xlabel('Time (s)'); ylabel('Angle (rad)'); grid on;
            nexttile; plot(t, q(17,:)); title('Right Ankle Pitch'); xlabel('Time (s)'); ylabel('Angle (rad)'); grid on;
            nexttile; plot(t, q(18,:)); title('Right Ankle Roll'); xlabel('Time (s)'); ylabel('Angle (rad)'); grid on;

                        % 
            % figure
            % tiledlayout(1,3);
            % nexttile; plot(t, q(19,:)); title('Waist Yaw'); xlabel('Time (s)'); ylabel('Angle (rad)'); grid on;
            % nexttile; plot(t, q(20,:)); title('Waist Roll'); xlabel('Time (s)'); ylabel('Angle (rad)'); grid on;
            % nexttile; plot(t, q(21,:)); title('Waist Pitch'); xlabel('Time (s)'); ylabel('Angle (rad)'); grid on;
            % 
            % figure
            % tiledlayout(2,7);
            % nexttile; plot(t, q(22,:)); title('Left Shoulder Pitch'); xlabel('Time (s)'); ylabel('Angle (rad)'); grid on;
            % nexttile; plot(t, q(23,:)); title('Left Shoulder Roll'); xlabel('Time (s)'); ylabel('Angle (rad)'); grid on;
            % nexttile; plot(t, q(24,:)); title('Left Shoulder Yaw'); xlabel('Time (s)'); ylabel('Angle (rad)'); grid on;
            % nexttile; plot(t, q(25,:)); title('Left Elbow'); xlabel('Time (s)'); ylabel('Angle (rad)'); grid on;
            % nexttile; plot(t, q(26,:)); title('Left Wrist Roll'); xlabel('Time (s)'); ylabel('Angle (rad)'); grid on;
            % nexttile; plot(t, q(27,:)); title('Left Wrist Pitch'); xlabel('Time (s)'); ylabel('Angle (rad)'); grid on;
            % nexttile; plot(t, q(28,:)); title('Left Wrist Yaw'); xlabel('Time (s)'); ylabel('Angle (rad)'); grid on;
            % nexttile; plot(t, q(29,:)); title('Right Shoulder Pitch'); xlabel('Time (s)'); ylabel('Angle (rad)'); grid on;
            % nexttile; plot(t, q(30,:)); title('Right Shoulder Roll'); xlabel('Time (s)'); ylabel('Angle (rad)'); grid on;
            % nexttile; plot(t, q(31,:)); title('Right Shoulder Yaw'); xlabel('Time (s)'); ylabel('Angle (rad)'); grid on;
            % nexttile; plot(t, q(32,:)); title('Right Elbow'); xlabel('Time (s)'); ylabel('Angle (rad)'); grid on;
            % nexttile; plot(t, q(33,:)); title('Right Wrist Roll'); xlabel('Time (s)'); ylabel('Angle (rad)'); grid on;
            % nexttile; plot(t, q(34,:)); title('Right Wrist Pitch'); xlabel('Time (s)'); ylabel('Angle (rad)'); grid on;
            % nexttile; plot(t, q(35,:)); title('Right Wrist Yaw'); xlabel('Time (s)'); ylabel('Angle (rad)'); grid on;
            
%             %test q gradient
%             for i=1:35
%                 obj.testGradient(t,q(i,:),dq(i,:))
%             end
        end

        function plotJointVel(obj, t, dq)
            base_pos = dq(1:3,:);
            base_rot = dq(4:6,:);

            figure;
            tiledlayout(3,2);
            nexttile; plot(t, base_pos(1,:)); title('Base X'); xlabel('Time (s)'); ylabel('X (m/s)'); grid on;
            nexttile; plot(t, base_rot(1,:)); title('Base Z'); xlabel('Time (s)'); ylabel('Yaw (rad/s)'); grid on;
            nexttile; plot(t, base_pos(2,:)); title('Base Y'); xlabel('Time (s)'); ylabel('Y (m/s)'); grid on;
            nexttile; plot(t, base_rot(2,:)); title('Base Y'); xlabel('Time (s)'); ylabel('Pitch (rad/s)'); grid on;
            nexttile; plot(t, base_pos(3,:)); title('Base Z'); xlabel('Time (s)'); ylabel('Z (m/s)'); grid on;
            nexttile; plot(t, base_rot(3,:)); title('Base X'); xlabel('Time (s)'); ylabel('Roll (rad/s)'); grid on;

            figure
            tiledlayout(2,6);
            nexttile; plot(t, dq(7,:)); title('Left Hip Pitch'); xlabel('Time (s)'); ylabel('Angle (rad/s)'); grid on;
            nexttile; plot(t, dq(8,:)); title('Left Hip Roll'); xlabel('Time (s)'); ylabel('Angle (rad/s)'); grid on;
            nexttile; plot(t, dq(9,:)); title('Left Hip Yaw'); xlabel('Time (s)'); ylabel('Angle (rad/s)'); grid on;
            nexttile; plot(t, dq(10,:)); title('Left Knee Pitch'); xlabel('Time (s)'); ylabel('Angle (rad/s)'); grid on;
            nexttile; plot(t, dq(11,:)); title('Left Ankle Pitch'); xlabel('Time (s)'); ylabel('Angle (rad/s)'); grid on;
            nexttile; plot(t, dq(12,:)); title('Left Ankle Roll'); xlabel('Time (s)'); ylabel('Angle (rad/s)'); grid on;
            nexttile; plot(t, dq(13,:)); title('Right Hip Pitch'); xlabel('Time (s)'); ylabel('Angle (rad/s)'); grid on;
            nexttile; plot(t, dq(14,:)); title('Right Hip Roll'); xlabel('Time (s)'); ylabel('Angle (rad/s)'); grid on;
            nexttile; plot(t, dq(15,:)); title('Right Hip Yaw'); xlabel('Time (s)'); ylabel('Angle (rad/s)'); grid on;
            nexttile; plot(t, dq(16,:)); title('Right Knee Pitch'); xlabel('Time (s)'); ylabel('Angle (rad/s)'); grid on;
            nexttile; plot(t, dq(17,:)); title('Right Ankle Pitch'); xlabel('Time (s)'); ylabel('Angle (rad/s)'); grid on;
            nexttile; plot(t, dq(18,:)); title('Right Ankle Roll'); xlabel('Time (s)'); ylabel('Angle (rad/s)'); grid on;
        end
        
        function testGradient(obj,t,q,dq)
            %assume q,dq is a 1xn vector
            figure
            plot(t, dq); hold on; plot(t, gradient(q,t)); xlabel('Time (s)'); ylabel('Velocity (rad/s)'); legend('dq','dq from q');grid on;
        end
        
        
        function plotWalking(obj,logfilename)
            %             close all;
            ifMultiDomain = 1;
            %0: QP
            %1: IK
            %2: unconstrianed motor
            method = 0
            
            
            if ifMultiDomain
                outputSize = 12;
                nContactF = obj.nContactF;
                walk_output_list = {'xCOM','yCOM','zCOM',...
                    'stanceHipYaw','deltaStanceFoot','deltaPitch','deltaRoll',...
                    'swingStepx','swingStepy','swingStepz',...
                    'swingHipYaw','deltaSwingFoot','zCOM_sw','deltaYaw'};
            else
                
                outputSize = 11;
                nContactF = obj.nContactF;
                walk_output_list = {'xCOM','yCOM','zCOM',...
                    'stanceHipYaw','deltaPitch','deltaRoll',...
                    'swingStepx','swingStepy','swingStepz',...
                    'swingHipYaw','deltaSwingFoot'};
            end
            fileID = fopen( [obj.path, 'walk_log.bin']);
            raw = fread(fileID,'float');
            
            
            
            
            if (method==0 && ~ifMultiDomain)
                LengthVec = [1,1,1,1, ...
                    outputSize, outputSize,outputSize,outputSize,outputSize, ...
                    3, 3, 2,2,2,1, ...
                    obj.nConfigSpace, obj.nConfigSpace,1,1,3,1,1,...
                    1, 10, 2, nContactF, nContactF, ...
                    1];
                sum(LengthVec)
                N = floor(length(raw) / sum(LengthVec));  % Number of samples
                
                [t, stanceLeg, tau,dtau,...
                    ya,dya, yd, dyd, d2yd, ...
                    pCOM, vCOM, Lpivot, Lcom, E,zdes, ...
                    q,dq,tmpx,tmpy, pCOM_abs, vx_des,vy_des,...
                    ifSolved, u, Frod, Fleft, Fright, ...
                    dt_qp] = obj.readRaw(raw, N, LengthVec);
            end
            
            if (method==0 && ifMultiDomain)
                LengthVec = [1,1,1,1, ...
                    outputSize, outputSize,outputSize,outputSize,outputSize, ...
                    3, 3, 2,2,2,1, ...
                    obj.nConfigSpace, obj.nConfigSpace,1,1,3,1,1,...
                    1,1, 3,3,3, 2,2,2, 2, 1,1,1,1, ...
                    1, 10, 2, nContactF, nContactF, ...
                    1];
                sum(LengthVec)
                N = floor(length(raw) / sum(LengthVec));  % Number of samples
                
                [t, stanceLeg, tau,dtau,...
                    ya,dya, yd, dyd, d2yd, ...
                    pCOM, vCOM, Lpivot, Lcom, E,zdes, ...
                    q,dq,tmpx,tmpy,pCOM_abs, vx_des,vy_des,...
                    curDomain, curMode, pCOM_toe, pCOM_mid, pCOM_heel, Lpivot_toe, Lpivot_mid, Lpivot_heel, stepsize, Ex_toe, Ex_mid,Ex_heel,Ey_mid,...
                    ifSolved, u, Frod, Fleft, Fright, ...
                    dt_qp] = obj.readRaw(raw, N, LengthVec);
                
            end
            
            if (method==1)
                if ~ifMultiDomain
                    LengthVec = [1,1,1,1, outputSize, outputSize,outputSize,outputSize,outputSize, ...
                        3, 3, 2,2,2,1, obj.nConfigSpace, obj.nConfigSpace,1,1,3,1,1,...
                        10, 10, 10, 10, outputSize,10, 1];
                    N = floor(length(raw) / sum(LengthVec));  % Number of samples
                    
                    [t, stanceLeg, tau,dtau, ya,dya, yd, dyd, d2yd, ...
                        pCOM, vCOM, Lpivot, Lcom, E,zdes, ...
                        q,dq,tmpx,tmpy, pCOM_abs, vx_des,vy_des,...
                        u, u_fb, u_ff, qmd, yd_ik,u_qpinback, dt_qp] = obj.readRaw(raw, N, LengthVec);
                    
                    
                else
                    LengthVec = [1,1,1,1, outputSize, outputSize,outputSize,outputSize,outputSize, ...
                        3, 3, 2,2,2,1, obj.nConfigSpace, obj.nConfigSpace,1,1,3,1,1,...
                        1,1, 3,3,3, 2,2,2, 2, 1,1,1,1, ...
                        10, 10,10,10, outputSize,1];
                    N = floor(length(raw) / sum(LengthVec));  % Number of samples
                    
                    [t, stanceLeg, tau,dtau, ya,dya, yd, dyd, d2yd, ...
                        pCOM, vCOM, Lpivot, Lcom, E,zdes, ...
                        q,dq,tmpx,tmpy, pCOM_abs, vx_des,vy_des,...
                        curDomain, curMode, pCOM_toe, pCOM_mid, pCOM_heel, Lpivot_toe, Lpivot_mid, Lpivot_heel, stepsize, Ex_toe, Ex_mid,Ex_heel,Ey_mid,...
                        u, u_fb, u_ff, qmd, yd_ik,dt_qp] = obj.readRaw(raw, N, LengthVec);
                    
                    
                end
                
                
                figure
                qorder = [7,8,9,10,12,13,14,15,16,18];
                for i = 1:10
                    subplot(2,5,i)
                    plot(t, qmd(i,:),'.')
                    hold on
                    plot(t,q(qorder(i),:),'.')
                    legend('qd','qa')
                    title([obj.q_list{qorder(i)}])
                end
            end
            
            if (method==2)
                if ~ifMultiDomain
                    LengthVec = [1,1,1,1, outputSize, outputSize,outputSize,outputSize,outputSize, ...
                        3, 3, 2,2,2,1, obj.nConfigSpace, obj.nConfigSpace,1,1,3,1,1,...
                        10,10,10,10,1];
                    N = floor(length(raw) / sum(LengthVec));  % Number of samples
                    
                    [t, stanceLeg, tau,dtau, ya,dya, yd, dyd, d2yd, ...
                        pCOM, vCOM, Lpivot, Lcom, E,zdes, ...
                        q,dq,tmpx,tmpy,pCOM_abs, vx_des,vy_des,...
                        u,u_fb,u_ff,u_qpinback ,dt_qp] = obj.readRaw(raw, N, LengthVec);
                else
                    LengthVec = [1,1,1,1, ...
                        outputSize, outputSize,outputSize,outputSize,outputSize, ...
                        3, 3, 2,2,2,1, ...
                        obj.nConfigSpace, obj.nConfigSpace,1,1,3,1,1,...
                        1,1, 3,3,3, 2,2,2, 2, 1,1,1,1, ...
                        10,10,10, ...
                        1];
                    sum(LengthVec)
                    N = floor(length(raw) / sum(LengthVec));  % Number of samples
                    
                    [t, stanceLeg, tau,dtau,...
                        ya,dya, yd, dyd, d2yd, ...
                        pCOM, vCOM, Lpivot, Lcom, E,zdes, ...
                        q,dq,tmpx,tmpy, pCOM_abs, vx_des,vy_des,...
                        curDomain, curMode, pCOM_toe, pCOM_mid, pCOM_heel, Lpivot_toe, Lpivot_mid, Lpivot_heel, stepsize, Ex_toe, Ex_mid,Ex_heel,Ey_mid, ...
                        u,u_fb,u_ff, ...
                        dt_qp] = obj.readRaw(raw, N, LengthVec);
                end
                
            end
            
            
            %         idx_start = find(t>=2,1);
            %         idx_end = find(t>=5,1);
            %         distance_traveled_in_xCOM = q(1,idx_end)-q(1,idx_start)
            % %         torque_sqrt = 0;
            % %         for i = idx_start:idx_end
            % %             torque_sqrt = torque_sqrt + u(:,i)'*u(:,i);
            % %         end
            %
            %         trapzu = zeros(10,1);
            %         for i = 1:10
            %             trapzu(i) = trapz(t(idx_start:idx_end)-t(idx_start),abs(u(i,idx_start:idx_end)));
            %         end
            %         trapzu
            %         sum(trapzu)/distance_traveled_in_xCOM
            %         speed = distance_traveled_in_xCOM/5
            
            
            
            
            q(:,1)
            %         qidx = 1;
            %             Jy = mexed.outputs.JyaLeftStanceDSSS(q(:,qidx));
            %             J_hol = [mexed.constraints.J_achilles_constraint(q(:,qidx));
            %                     mexed.constraints.J_leftSole_constraint(q(:,qidx))];
            %                 J_hol'*J_hol
            %             pinv(eye(18)- J_hol'*J_hol)*Jy
            %
            
            %             stanceLeg(1)
            
            %             figure
            %             for i =1:2
            %                 subplot(2,1,i)
            %                 plot(t,Lpivot(i,:))
            %                 title('Lpivot')
            %             end
            
            % figure
            % plot(t,d2yd(1,:))
            % hold on
            % plot(t,gradient(dyd(1,:),t))
            %
            % figure
            % subplot(2,1,1)
            % plot(t,'.')
            % subplot(2,1,2)
            % plot(1./gradient(t),'.')
            
            
            
            % idx = 8; %swy
            % figure,
            % i=idx;
            % plot(t, gradient(yd(i,:),t), 'r', t, dyd(i,:), 'b');
            % title('comp')
            
            % figure
            % subplot(2,1,1)
            % plot(t,tau)
            % title('tau')
            % subplot(2,1,2)
            % plot(t,dtau)
            % title('dtau')
            
            %%
            subplotrow = 4;
            figure,
            for i = 1:outputSize
                subplot(subplotrow,3,i), plot(t, ya(i,:), 'r.', t, yd(i,:), 'b.');
                hold on
                if (method == 1)
                    plot(t,yd_ik(i,:),'k.')
                    legend('actual','desired','desired_ik');
                end
                title([walk_output_list{i} '-pos' ]);
            end
            legend('actual','desired');
            
            %             figure,
            %             for i = 1:outputSize
            %                 subplot(4,3,i), plot(t, ya(i,:)- yd(i,:), '.');
            %
            %                 title([walk_output_list{i} '-pos error' ]);
            %             end
            %             legend('error');
            %
            
            if (ifMultiDomain)
                subplot(subplotrow,3,1)
                hold on
                plot(t,curDomain)
                legend('actual','desired', 'curDomain');
                
                subplot(subplotrow,3,8)
                hold on
                plot(t,curMode)
                hold on
                plot(t,stepsize(1,:),'.')
                legend('actual','desired', 'curMode', 'stepX');
                
                subplot(subplotrow,3,9)
                hold on
                plot(t,stepsize(2,:),'.')
                legend('actual','desired', 'stepY');
            end
            
            
            
            %             subplot(4,3,12)
            %             plot(t, tau, t, stanceLeg); title('tau');
            
            
            %              subplot(4,3,2)
            %             hold on
            %             plot(t,tmpy)
            
            
            %             figure,
            %             for i = 1:outputSize
            %                 subplot(subplotrow,3,i), plot(t, dya(i,:), 'r', t, dyd(i,:), 'b'); title([walk_output_list{i} '-vel' ]);
            %                 hold on
            %                 plot(t,gradient(yd(i,:),t))
            %             end
            %
            %             if ~ifMultiDomain
            %             subplot(subplotrow,3,12)
            %             plot(t, dtau, t, stanceLeg); title('tau');
            %             legend('actual','desired');
            %             end
            %
            %             figure,
            %             for i = 1:outputSize
            %                 subplot(subplotrow,3,i), plot(t, d2yd(i,:), 'b'); title([walk_output_list{i} '-acc' ]);
            %                 hold on
            %                 plot(t,gradient(dyd(i,:),t))
            %             end
            %             legend('desired');
            
            
            figure,
            for i = 1:3
                subplot(3,3,i),
                plot(t, pCOM(i,:));
                title('COM Pos')
            end
            
            for i = 1:3
                subplot(3,3,i+3);
                plot(t, vCOM(i,:));
                if i==1
                    hold on
                    plot(t,vx_des)
                elseif i==2
                    hold on
                    plot(t,vy_des)
                end
                
                title('COM vel')
            end
            
            for i = 1:2
                subplot(3,3,i+6);
                %                 plot(t, movmean(gradient(vCOM(i,:)),200));
                plot(t,Lpivot(i,:))
                hold on
                plot(t,Lcom(i,:))
                title('L')
                legend('Lpivot','Lcom')
            end
            %
            %             figure
            %             plot(pCOM(1,:),Lpivot(2,:))
            %             hold on
            %             plot(pCOM(2,:),Lpivot(1,:))
            %
            %             if ifMultiDomain
            %             figure
            %             subplot(4,1,1)
            %             plot(t,Ex_toe(1,:))
            %             legend('Ex_toe')
            %             subplot(4,1,2)
            %             plot(t,Ex_mid(1,:))
            %             legend('Ex_mid')
            %             subplot(4,1,3)
            %             plot(t,Ex_heel(1,:))
            %             legend('Ex_heel')
            %             subplot(4,1,4)
            %             plot(t,Ey_mid(1,:))
            %             legend('Ey_mid')
            %             hold on
            %                 plot(t,curDomain)
            %             end
            
            
            
            
            %             figure
            %             for i = 1:obj.nConfigSpace
            %                 subplot(3,6,i)
            %                 plot(t,dq(i,:))
            %                 title([obj.q_list{i} ' vel'])
            %             end
            
            figure
            for i = 1:obj.nConfigSpace
                subplot(3,6,i)
                plot(t,q(i,:))
                hold on
                yline(obj.joint_min(i))
                hold on
                yline(obj.joint_max(i))
                title([obj.q_list{i} ' pos'])
            end
            
            figure
            for i = 1:10
                subplot(2,5, i)
                plot(t, u(i,:));
                hold on
                if (method ==1 || method ==2)
                    %                                 plot(t, u_ff(i,:));
                    %                                 hold on
                    %                                 plot(t, u_fb(i,:));
                    %                                 hold on
                    plot(t, u_qpinback(i,:))
                    %                                 legend('u','uff','ufb','uqpback')
                    legend('u','uqpback')
                end
                title([obj.motor_list{i} ' Torque']);
                hold on
                yline(-obj.torque_bound(i))
                hold on
                yline(obj.torque_bound(i))
                if ifMultiDomain
                    hold on
                    plot(t,curDomain)
                end
            end
            
            
            
            
            
            %             T = [t(find(diff(curDomain)==-1)); t(find(diff(curDomain)==1))];
            %             stepTFA = diff(reshape([t(1) reshape(T,1,[]) 0 ],2,[]));
            %             stepTFA = stepTFA(1:end-1);
            %             stepTOA = diff(T);
            %             figure
            %             subplot(2,1,1)
            %             plot(stepTFA)
            %             subplot(2,1,2)
            %             plot(stepTOA)
            
            %            T = t(find(tau==0)) ;
            %            figure
            %            plot(diff(T))
            
            %             figure
            %             plot(pCOM(1,:),vCOM(1,:))
            %             title('phase')
            
            %             figure
            %             plot(t,Udes)
            %             title('Udes sagittal')
            %
            %             figure
            %             plot(t,UleftDes,t,UrightDes)
            %             legend('UleftDes','UrightDes')
            %             title('Udes lateral')
            
            
            
            %             log = {};
            %
            %
            %
            %
            %             log.t = t;
            %             log.stanceLeg =stanceLeg;
            %             log.tau =tau;
            %             log.dtau =dtau;
            %             log.ya =ya;
            %             log.dya =dya;
            %             log.yd =yd ;
            %             log.dyd =dyd ;
            %             log.d2yd = d2yd;
            %             log.pCOM = pCOM;
            %             log.vCOM = vCOM;
            %             log.Lpivot = Lpivot;
            %             log.Lcom = Lcom;
            %             log.E = E;
            %             log.zdes =zdes;
            %             log.q =q;
            %             log.dq =dq;
            %             log.vx_des =vx_des;
            %             log.vy_dex =vy_dex;
            %
            %
            %
            %             if (ifMultiDomain)
            %                log.curDomain = curDomain;
            %                log.curMode = curMode;
            %                log.pCOM_toe = pCOM_toe;
            %                log.pCOM_mid = pCOM_mid;
            %                log.pCOM_heel = pCOM_heel;
            %                log.Lpivot_toe = Lpivot_toe;
            %                log.Lpivot_mid = Lpivot_mid;
            %                log.Lpivot_heel = Lpivot_heel;
            %                log.stepsize = stepsize;
            %                log.Ex_toe = Ex_toe;
            %                log.Ex_mid = Ex_mid;
            %                log.Ex_heel = Ex_heel;
            %                log.Ey_mid = Ey_mid;
            %             end
            %
            %
            %                log.ifSolved = ifSolved;
            %                log.u = u;
            %                log.Frod = Frod;
            %                log.Fleft = Fleft;
            %                log.Fright = Fright;
            %                log.dt_qp = dt_qp;
            
            
            
            %             figure,
            %             plot(t, ifSolved); title('QP solved or not');
            %
            %                         figure
            %                         for i = 1:10
            %                             subplot(2,5, i)
            %                             plot(t, u(i,:));
            %                             hold on
            %                             if (method ==2)
            %                                 plot(t, u_ff(i,:));
            %                                 hold on
            %                                 plot(t, u_fb(i,:));
            %                                 legend('u','uff','ufb')
            %                             end
            %                             title([obj.motor_list{i} ' Torque']);
            %                             hold on
            %                             yline(-obj.torque_bound(i))
            %                             hold on
            %                             yline(obj.torque_bound(i))
            %                         end
            
            
            
            
            
            %
            %                         figure,
            %                         for i = 1:obj.nContactF
            %                             subplot(1,obj.nContactF,i), plot(t, Fleft(i,:),'r', t, Fright(i,:),'b'); title('contact Force')
            %                             legend('Left', 'Right')
            %                         end
            % %
            %             figure,
            %             for i = 1:obj.nContactF
            %                 subplot(1,obj.nContactF,i), plot(t, Fstance(i,:)); title('contact Force')
            %                 legend('Stance')
            %             end
            %
            %             figure
            %             subplot(2,1,1)
            %             plot(t,dt_qp)
            %             legend('dt')
            %             subplot(2,1,2)
            %             plot(t,1./dt_qp)
            %             legend('freq')
            %
            %
            %
            
            %               q(12,:) = q(12,:)+.02;
            %               q(18,:) = q(18,:)+.01;
            idx_all = 1:50:length(t);
            leftfootpitch = zeros(size(1:50:length(t)));
            rightfootpitch = zeros(size(1:50:length(t)));
            leftfootheight = zeros(size(1:50:length(t)));
            rightfootheight = zeros(size(1:50:length(t)));;
            for i = 1:length(idx_all)
                
                tmp = [mexed.constraints.p_leftSole_constraint(q(:,idx_all(i))) mexed.constraints.p_rightSole_constraint(q(:,idx_all(i)))];
                leftfootpitch(i) = tmp(4,1);
                rightfootpitch(i) = tmp(4,2);
                leftfootheight(i) = tmp(3,1);
                rightfootheight(i) = tmp(3,2);
            end
            
            %             figure
            %             plot(t(idx_all),leftfootpitch)
            %             hold on
            %             plot(t(idx_all),rightfootpitch)
            %             legend('left','right')
            %             title('foot pitch')
            %
            %             figure
            %             plot(t(idx_all),leftfootheight)
            %             hold on
            %             plot(t(idx_all),rightfootheight)
            %             legend('left','right')
            %             title('foot height')
            
            
        end
        
        
        
        
    end
    
    
    methods
        
        function  varargout  = readRaw(obj, raw, N, LengthVec)
            nout = max(nargout, 1) ;
            
            for i = 1:length(LengthVec)
                varargout{i} = zeros(LengthVec(i), N);
            end
            
            idx = 1;
            for i = 1:N
                for k = 1:nout
                    var_length = LengthVec(k);
                    varargout{k}(:,i) = raw(idx:idx + var_length -1);
                    idx = idx + var_length ;
                end
            end
            
        end
        
    end
end


