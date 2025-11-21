classdef MPC< matlab.System
    % Model Predictive Control Block
   
    % Define the properties of the block, that can be changed 
    properties(Nontunable)

       % define OCP parameters
       %...
       tau_s = 0.2; % [s]
       Q = [];
       R = [];
       P = [];
       N = 10;
    end
    
    
    methods(Access = protected)
        function u_opt = stepImpl(obj, xk, pose_ref, theta_ref, time_sim, robot_params)
            % Main body of the block. Put the instructions that must be executed at each time step here.
            % Find Nearest index

            disp("........")
            disp(time_sim)
            [~, idx] = min(abs(pose_ref(:,1) - time_sim));

            % We find the final index
            idx_end = min(idx + obj.N, size(pose_ref,1));

            % We asign the Pose Reference
            pose = zeros(obj.N, 2);
            theta = zeros(obj.N, 1);

            % We find the length
            len = idx_end - idx + 1;

            % We asign the values
            pose(1:len, :) = pose_ref(idx:idx_end, 2:3);
            theta(1:len, 1) = theta_ref(idx:idx_end, 2);

            if len < obj.N
                pose(len+1:end,:) = repmat(pose(len,:), obj.N+1 - len, 1);
                theta(len+1:end,1) = theta(len,1);
            end
            
            % Solve the FHOCP
            u_opt = FHOCP(xk, obj.Q, obj.R, obj.P, obj.N, 2, 2, obj.tau_s, pose', theta', robot_params);
          
        end

        function out_size = getOutputSizeImpl(~)
            % Return the size of the output signal
            
            out_size = [2 1]; % Scalar
        end

        function out_type = getOutputDataTypeImpl(~)
            % Return data type of the output port. 
            % Choose between MATLAB Data types (e.g. "double", "int32", "uint64", etc.)
            
            out_type = "double";     
        end

        function out_complex = isOutputComplexImpl(~)
            % Return true if the output signal is complex, false otherwise
            
            out_complex = false;
        end

        function out_fixed = isOutputFixedSizeImpl(~)
            % Return true if the output signal has a fixed size, false otherwise
             out_fixed = true;
        end


        function sts = getSampleTimeImpl(obj)
            % Define the sampling time of the block.
            
            % This is a discrete-time block with sampling time specified by the user via the tau_s parameter
            sts = obj.createSampleTime("Type", "Discrete", "SampleTime", obj.tau_s);
        end
        
        

    end
end
