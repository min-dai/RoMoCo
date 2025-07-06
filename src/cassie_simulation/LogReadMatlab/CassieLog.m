classdef CassieLog < handle
    
    properties
        nConfigSpace = 18;
        nContactF = 5;
        path = [getenv('HOME') '/ROBOTLOG/Cassie/']
        
        inair_output_list = {'LeftFootx','LeftFooty','LeftFootz','LeftFootPitch','LeftFootYaw','RightFootx','RightFooty','RightFootz','RightFootPitch','RightFootYaw'};
        
        stand_output_list = {'COMx','COMy','COMz','PelvisRoll','PelvisPitch','PelvisYawDiff'}
        
        walkHLIP_output_list = {'zCOM',...
            'stanceHipYaw','deltaPitch','deltaRoll',...
            'swingStepx','swingStepy','swingStepz',...
            'swingHipYaw','deltaSwingFoot'};

        walkMLIP_output_list = {'zCOM',...
            'stanceHipYaw','deltaPitch','deltaRoll','deltaStanceFoot',...
            'swingStepx','swingStepy','swingStepz',...
            'swingHipYaw','deltaSwingFoot'};

        q_list = {'BasePosX','BasePosY','BasePosZ','BaseRotZ','BaseRotY','BaseRotX',...
            'LeftHipRoll','LeftHipYaw','LeftHipPitch','LeftKneePitch','LeftTarsusPitch','LeftFootPitch',...
            'RightHipRoll','RightHipYaw','RightHipPitch','RightKneePitch','RightTarsusPitch','RightFootPitch'}
        
        motor_list = {'LeftHipRoll','LeftHipYaw','LeftHipPitch','LeftKneePitch','LeftFootPitch', ...
            'RightHipRoll','RightHipYaw','RightHipPitch','RightKneePitch','RightFootPitch'};
        
        joint_max = [pi*ones(1,6) ,...
            0.3927   0.3927  1.3963  -0.6458    2.9671   -0.5236 ,...
            0.3927   0.3927  1.3963  -0.6458    2.9671   -0.5236   ]
        joint_min = [-pi*ones(1,6) ,...
            -0.3927 -0.3927 -0.8727  -2.8623    0.8727 -2.4435 ,...
            -0.3927 -0.3927 -0.8727  -2.8623    0.8727 -2.4435 ]
        torque_bound = [112.5000  112.5000  195.2000  195.2000   45.0000  112.5000  112.5000  195.2000  195.2000   45.0000];
    end
    
    methods
        function  obj =   CassieLog()

        end
        
        function plotInAir(obj)
            
            fileID = fopen( [obj.path, 'logInAir.bin']);
            raw = fread(fileID,'float');
            
            nY = length(obj.inair_output_list);
            nU = length(obj.motor_list);
            LengthVec = [1,obj.nConfigSpace, obj.nConfigSpace,nU,nY,nY,nY,nY,nY];
            
            N = floor(length(raw) / sum(LengthVec));  % Number of samples
            
            [t, q,dq,u, ya,dya,yd,dyd,d2yd] = obj.readRaw(raw, N, LengthVec);
            

            %plot ya yd
            figure
            tiledlayout(2,ceil(nY/2));
            for i=1:nY
                nexttile; plot(t, ya(i,:));  hold on; plot(t, yd(i,:),'k-'); title(obj.inair_output_list{i}); grid on;
            end

            %plot u
            figure
            tiledlayout(2,ceil(nY/2));
            for i = 1:nU
                nexttile; plot(t, u(i,:)); title(obj.motor_list{i}); xlabel('Time (s)'); ylabel('Torque (Nm)'); grid on;
            end

            plotJointPos(obj, t, q)
            
            
        end
        

        function plotStand(obj)

            fileID = fopen( [obj.path, 'logStand.bin']);
            raw = fread(fileID,'float');
            
            nY = length(obj.stand_output_list);
            nU = length(obj.motor_list);
            LengthVec = [1,obj.nConfigSpace, obj.nConfigSpace,nU,nY,nY,nY,nY,nY];
            
            N = floor(length(raw) / sum(LengthVec));  % Number of samples
            
            [t, q,dq,u, ya,dya,yd,dyd,d2yd] = obj.readRaw(raw, N, LengthVec);
            

            %plot ya yd
            figure
            tiledlayout(1,6);
            for i=1:nY
                nexttile; plot(t, ya(i,:));  hold on; plot(t, yd(i,:),'k-'); title(obj.stand_output_list{i}); grid on;
            end

            % %plot u
            % figure
            % tiledlayout(2,6);
            % for i = 1:nU
            %     nexttile; plot(t, u(i,:)); title(obj.motor_list{i}); xlabel('Time (s)'); ylabel('Torque (Nm)'); grid on;
            % end
            % 
            % plotJointPos(obj, t, q)
        end

        function plotWalk(obj)
            fileID = fopen( [obj.path, 'logWalk.bin']);
            raw = fread(fileID,'float');
            
            nY = length(obj.walkHLIP_output_list);
            nU = length(obj.motor_list);
            LengthVec = [1,obj.nConfigSpace, obj.nConfigSpace,nU,nY,nY,nY,nY,nY, 3,3,3,3,1,1];
            
            
            N = floor(length(raw) / sum(LengthVec));  % Number of samples
            
            [t, q,dq,u_leg, ya,dya,yd,dyd,d2yd, pCOM, vCOM, Lcom, Lpivot,vdx,vdy] = obj.readRaw(raw, N, LengthVec);
            
            


            %plot ya yd
            figure
            tiledlayout(2,5);
            for i=1:nY
                nexttile; plot(t, ya(i,:));  hold on; plot(t, yd(i,:),'k-'); title(obj.walkHLIP_output_list{i}); grid on;
            end

            %plot u
            figure
            tiledlayout(2,6);
            for i = 1:nU
                nexttile; plot(t, u_leg(i,:)); title(obj.motor_list{i}); xlabel('Time (s)'); ylabel('Torque (Nm)'); grid on;
            end

            plotJointPos(obj, t, q)
            plotJointVel(obj, t, dq)
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
            tiledlayout(2,ceil((length(obj.q_list)-6)/2));
            for i = 7:length(obj.q_list)
                nexttile; plot(t, q(i,:)); title(obj.q_list{i}); xlabel('Time (s)'); ylabel('Angle (rad)'); grid on;
            end

                        % 

        end

        function plotJointVel(obj, t, dq)
            base_pos = dq(1:3,:);
            base_rot = dq(4:6,:);

            figure;
            tiledlayout(3,2);
            nexttile; plot(t, base_pos(1,:)); title('Base Position X'); xlabel('Time (s)'); ylabel('X (m/s)'); grid on;
            nexttile; plot(t, base_rot(1,:)); title('Base Rotation Z'); xlabel('Time (s)'); ylabel('Yaw (rad/s)'); grid on;
            nexttile; plot(t, base_pos(2,:)); title('Base Position Y'); xlabel('Time (s)'); ylabel('Y (m/s)'); grid on;
            nexttile; plot(t, base_rot(2,:)); title('Base Rotation Y'); xlabel('Time (s)'); ylabel('Pitch (rad/s)'); grid on;
            nexttile; plot(t, base_pos(3,:)); title('Base Position Z'); xlabel('Time (s)'); ylabel('Z (m/s)'); grid on;
            nexttile; plot(t, base_rot(3,:)); title('Base Rotation X'); xlabel('Time (s)'); ylabel('Roll (rad/s)'); grid on;
            


            figure
            tiledlayout(2,ceil((length(obj.q_list)-6)/2));
            for i = 7:length(obj.q_list)
                nexttile; plot(t, dq(i,:)); title(obj.q_list{i}); xlabel('Time (s)'); ylabel('Angular Rate (rad/s)'); grid on;
            end

                        % 

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


