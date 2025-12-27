classdef G1Log < handle
    
    properties
        nConfigSpace = 18+17;
        nLoco = 18;
        nContactF = 6;
        path = [getenv('HOME') '/ROBOTLOG/G1/']
        folder_name = 'logs_2025-09-03_20-03-28'
        
        inair_output_list = {'LeftFootx','LeftFooty','LeftFootz','LeftFootPitch','LeftFootYaw','RightFootx','RightFooty','RightFootz','RightFootPitch','RightFootYaw'};
        
        stand_output_list = {'COMx','COMy','COMz','PelvisRoll','PelvisPitch','PelvisYawDiff'}
        
        walk_output_list = {'xCOM','yCOM','zCOM',...
            'stanceHipYaw','deltaPitch','deltaRoll',...
            'swingStepx','swingStepy','swingStepz',...
            'swingHipYaw','deltaSwingFoot'};


        q_list = {'BasePosX','BasePosY','BasePosZ','BaseRotZ','BaseRotY','BaseRotX',...
            'LeftHipPitch','LeftHipRoll','LeftHipYaw','LeftKneePitch','LeftAnklePitch','LeftAnkleRoll',...
            'RightHipPitch','RightHipRoll','RightHipYaw','RightKneePitch','RightAnklePitch','RightAnkleRoll'}
        
        motor_list = {'LeftHipPitch','LeftHipRoll','LeftHipYaw','LeftKneePitch','LeftAnklePitch','LeftAnkleRoll',...
            'RightHipPitch','RightHipRoll','RightHipYaw','RightKneePitch','RightAnklePitch','RightAnkleRoll'};
        

    end
    
    methods
        function newestFolderName = getNewestG1LogFolderNameOnly(obj)

            d = dir(obj.path);
            d = d([d.isdir] & ~ismember({d.name}, {'.', '..'}));
            if isempty(d)
                newestFolderName = '';
                return;
            end
            [~, idx] = max([d.datenum]);
            newestFolderName = d(idx).name;
        end
        function  obj =   G1Log()
        end

        function plotInterface(obj)
            newestFolderName = obj.getNewestG1LogFolderNameOnly();
            full_path = [obj.path, newestFolderName, '/'];
            fileID = fopen( [full_path, 'logInterface.bin']);
            raw = fread(fileID,'float');
            
   
            LengthVec = [1,3,35,35];
            
            N = floor(length(raw) / sum(LengthVec));  % Number of samples
            
            [t, est_v, q, dq] = obj.readRaw(raw, N, LengthVec);
            

            output_list = {'x','y','z'};


            %plot ya yd
            figure
            tiledlayout(1,3);
            for i=1:3
                nexttile; plot(t, est_v(i,:));  ; title(output_list{i});  grid on;
            end

            % obj.plotJointPos(t,q)

            obj.plotJointPosUpper(t,q)

            dt = [t t(end)]- [t(1) t];
            figure
            plot(t,dt(1:end-1))


        end

        function plotSimInterface(obj)
            newestFolderName = obj.getNewestG1LogFolderNameOnly();
            full_path = [obj.path, newestFolderName, '/'];
            fileID = fopen( [full_path, 'logSimInterface.bin']);
            raw = fread(fileID,'float');
            
   
            LengthVec = [1,3,3];
            
            N = floor(length(raw) / sum(LengthVec));  % Number of samples
            
            [t, est_v, true_v] = obj.readRaw(raw, N, LengthVec);
            

            output_list = {'x','y','z'};


            %plot ya yd
            figure
            tiledlayout(1,3);
            for i=1:3
                nexttile; plot(t, est_v(i,:));  hold on; plot(t, true_v(i,:),'k-'); title(output_list{i}); legend('est','true'); grid on;
            end


        end
        
        function plotInAir(obj)
            newestFolderName = obj.getNewestG1LogFolderNameOnly();
            full_path = [obj.path, newestFolderName, '/'];
            fileID = fopen( [full_path, 'logInAir.bin']);

            raw = fread(fileID,'float');
            
            nY = 12;
            LengthVec = [1,obj.nLoco, obj.nLoco,12,nY,nY,nY,nY,nY];
            
            N = floor(length(raw) / sum(LengthVec));  % Number of samples
            
            [t, q,dq,u_leg, ya,dya,yd,dyd,d2yd] = obj.readRaw(raw, N, LengthVec);
            

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
            newestFolderName = obj.getNewestG1LogFolderNameOnly();
            full_path = [obj.path, newestFolderName, '/'];
            fileID = fopen( [full_path, 'logStand.bin']);
            raw = fread(fileID,'float');
            
            nY = 6;
            LengthVec = [1,obj.nLoco, obj.nLoco,12,nY,nY,nY,nY,nY];
            
            N = floor(length(raw) / sum(LengthVec));  % Number of samples
            
            [t, q,dq,u_leg,ya,dya,yd,dyd,d2yd] = obj.readRaw(raw, N, LengthVec);
            

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

        function plotWalk(obj, exportName, folderName)
            if nargin > 2 && ~isempty(folderName)
                newestFolderName = folderName;
            else
                newestFolderName = obj.getNewestG1LogFolderNameOnly();
            end


            full_path = [obj.path, newestFolderName, '/'];
            fileID = fopen( [full_path, 'logWalk.bin']);

            raw = fread(fileID,'float');
            
            nY = 10;
            LengthVec = [1,obj.nLoco, obj.nLoco,12,nY,nY,nY,nY,nY, 3,3,3,3,1,1];

            
            N = floor(length(raw) / sum(LengthVec));  % Number of samples
            
            [t, q,dq,u_leg, ya,dya,yd,dyd,d2yd, pCOM, vCOM, Lcom, Lpivot,vdx,vdy] = obj.readRaw(raw, N, LengthVec);

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


            for i = 4:6
            testGradient(obj,t,q(i,:),dq(i,:))
            end


            %plot u
            figure
            tiledlayout(2,6);
            for i = 1:10
                nexttile; plot(t, u_leg(i,:)); title(obj.motor_list{i}); xlabel('Time (s)'); ylabel('Torque (Nm)'); grid on;
            end

            plotJointPos(obj, t, q)
            plotJointVel(obj, t, dq)

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


            % Export log data if exportName is provided
            if nargin > 1 && ~isempty(exportName)
                % Ensure exportName has .mat extension
                if ~endsWith(exportName, '.mat')
                    exportName = [exportName, '.mat'];
                end

                % Save to current directory
                save(exportName, 't', 'q', 'dq', 'u_leg', ...
                    'ya', 'dya', 'yd', 'dyd', 'd2yd', 'pCOM', 'vCOM', 'Lcom', 'Lpivot', 'vdx', 'vdy');

                fprintf('Log data exported to: %s\n', exportName);
            end

            
        end

        function plotWalkMomentum(obj)
            fileID = fopen( [obj.path, 'logWalk.bin']);
            raw = fread(fileID,'float');
            
            nY = 10;
            LengthVec = [1,obj.nConfigSpace, obj.nConfigSpace,12+11,6,nY,nY,nY,nY,nY, 3,3,3,3,1,1];
            
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


            % for i = 5:7
            % testGradient(obj,t,dyd(i,:),d2yd(i,:))
            % end


            %plot u
            figure
            tiledlayout(2,6);
            for i = 1:10
                nexttile; plot(t, u_leg(i,:)); title(obj.motor_list{i}); xlabel('Time (s)'); ylabel('Torque (Nm)'); grid on;
            end

            % plotJointPos(obj, t, q)
            % plotJointVel(obj, t, dq)

            %plot COM
            figure
            tiledlayout(4,3);
            nexttile; plot(t, pCOM(1,:)); title('COM X'); xlabel('Time (s)'); ylabel('X (m)'); grid on;
            nexttile; plot(t, pCOM(2,:)); title('COM Y'); xlabel('Time (s)'); ylabel('Y (m)'); grid on;
            nexttile; plot(t, pCOM(3,:)); title('COM Z'); xlabel('Time (s)'); ylabel('Z (m)'); grid on;
            nexttile; plot(t, vCOM(1,:)); hold on; plot(t, vdx); title('COM Vel X'); xlabel('Time (s)'); legend('actual','desired'); grid on;
            nexttile; plot(t, vCOM(2,:)); hold on; plot(t, vdy); title('COM Vel Y'); xlabel('Time (s)'); legend('actual','desired'); grid on;
            nexttile; plot(t, vCOM(3,:)); title('COM Vel Z'); xlabel('Time (s)'); ylabel('Z (m/s)'); grid on;
            nexttile; plot(t, Lpivot(1,:)); title('Lpivot X'); xlabel('Time (s)'); ylabel('X (m)'); grid on;
            nexttile; plot(t, Lpivot(2,:)); title('Lpivot Y'); xlabel('Time (s)'); ylabel('Y (m)'); grid on;
            nexttile; plot(t, Lpivot(3,:)); title('Lpivot Z'); xlabel('Time (s)'); ylabel('Z (m)'); grid on;
            nexttile; plot(t, Lcom(1,:)); title('Lcom X'); xlabel('Time (s)'); ylabel('X (m)'); grid on;
            nexttile; plot(t, Lcom(2,:)); title('Lcom Y'); xlabel('Time (s)'); ylabel('Y (m)'); grid on;
            nexttile; plot(t, Lcom(3,:)); title('Lcom Z'); xlabel('Time (s)'); ylabel('Z (m)'); grid on;

            
        end

        function plotWalkToe(obj)
            fileID = fopen( [obj.path, 'logWalk.bin']);
            raw = fread(fileID,'float');
            
            nY = 11;
            LengthVec = [1,obj.nConfigSpace, obj.nConfigSpace,12,17,nY,nY,nY,nY,nY];
            
            N = floor(length(raw) / sum(LengthVec));  % Number of samples
            
            [t, q,dq,u_leg,u_arm, ya,dya,yd,dyd,d2yd] = obj.readRaw(raw, N, LengthVec);
            

            output_list = {'zCOM',...
            'stanceHipYaw','basePitch','baseRoll','StanceSwingPitch',...
            'swingStepx','swingStepy','swingStepz',...
            'swingHipYaw','SwingDeltaPitch', 'SwingDeltaRoll'};


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
        
        function plotJointPosUpper(obj,t,q)
            figure
            tiledlayout(1,3);
            nexttile; plot(t, q(19,:)); title('Waist Yaw'); xlabel('Time (s)'); ylabel('Angle (rad)'); grid on;
            nexttile; plot(t, q(20,:)); title('Waist Roll'); xlabel('Time (s)'); ylabel('Angle (rad)'); grid on;
            nexttile; plot(t, q(21,:)); title('Waist Pitch'); xlabel('Time (s)'); ylabel('Angle (rad)'); grid on;

            figure
            tiledlayout(2,7);
            nexttile; plot(t, q(22,:)); title('Left Shoulder Pitch'); xlabel('Time (s)'); ylabel('Angle (rad)'); grid on;
            nexttile; plot(t, q(23,:)); title('Left Shoulder Roll'); xlabel('Time (s)'); ylabel('Angle (rad)'); grid on;
            nexttile; plot(t, q(24,:)); title('Left Shoulder Yaw'); xlabel('Time (s)'); ylabel('Angle (rad)'); grid on;
            nexttile; plot(t, q(25,:)); title('Left Elbow'); xlabel('Time (s)'); ylabel('Angle (rad)'); grid on;
            nexttile; plot(t, q(26,:)); title('Left Wrist Roll'); xlabel('Time (s)'); ylabel('Angle (rad)'); grid on;
            nexttile; plot(t, q(27,:)); title('Left Wrist Pitch'); xlabel('Time (s)'); ylabel('Angle (rad)'); grid on;
            nexttile; plot(t, q(28,:)); title('Left Wrist Yaw'); xlabel('Time (s)'); ylabel('Angle (rad)'); grid on;
            nexttile; plot(t, q(29,:)); title('Right Shoulder Pitch'); xlabel('Time (s)'); ylabel('Angle (rad)'); grid on;
            nexttile; plot(t, q(30,:)); title('Right Shoulder Roll'); xlabel('Time (s)'); ylabel('Angle (rad)'); grid on;
            nexttile; plot(t, q(31,:)); title('Right Shoulder Yaw'); xlabel('Time (s)'); ylabel('Angle (rad)'); grid on;
            nexttile; plot(t, q(32,:)); title('Right Elbow'); xlabel('Time (s)'); ylabel('Angle (rad)'); grid on;
            nexttile; plot(t, q(33,:)); title('Right Wrist Roll'); xlabel('Time (s)'); ylabel('Angle (rad)'); grid on;
            nexttile; plot(t, q(34,:)); title('Right Wrist Pitch'); xlabel('Time (s)'); ylabel('Angle (rad)'); grid on;
            nexttile; plot(t, q(35,:)); title('Right Wrist Yaw'); xlabel('Time (s)'); ylabel('Angle (rad)'); grid on;
            
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
            
%             %test q gradient
%             for i=1:35
%                 obj.testGradient(t,q(i,:),dq(i,:))
%             end
        end
        
        function testGradient(obj,t,q,dq)
            %assume q,dq is a 1xn vector
            figure
            plot(t, dq); hold on; plot(t, gradient(q,t)); xlabel('Time (s)'); ylabel('Velocity (rad/s)'); legend('dq','dq from q');grid on;
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


