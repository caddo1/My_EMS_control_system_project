
classdef EMSController < matlab.System
    % EMS MPC controller using YALMIP

    methods(Access = protected)

        %--- Main step -----------------------------------------------
        function [u_bat, e, reward] = stepImpl(~, SoC, loadP, R, T, PV_forecast)
            % Called every Simulink timestep (interpreted MATLAB)
            [u_bat, e, reward] = pv_battery_ems(PV_forecast, SoC, loadP, R, T);
        end

       

        % Number of outputs
        function n = getNumOutputsImpl(~)
            n = 3;   % u_bat, e, and reward
        end

        % Output sizes
        function [sz1, sz2, sz3] = getOutputSizeImpl(~)
            sz1 = [1 1];   % u_bat is scalar
            sz2 = [4 1];   % e is 4x1 (because you have 4 loads)
            sz3 = [1 1];   % reward is scalar
        end

        % Output data types
        function [dt1, dt2, dt3] = getOutputDataTypeImpl(~)
            dt1 = 'double';
            dt2 = 'double';   % you can keep e as double 0/1
            dt3 = 'double';   % reward is scalar
        end

        % Outputs are real (not complex)
        function [c1, c2, c3] = isOutputComplexImpl(~)
            c1 = false;
            c2 = false;
            c3 = false;
        end

        % Outputs have fixed size
        function [fs1, fs2, fs3] = isOutputFixedSizeImpl(~)
            fs1 = true;
            fs2 = true;
            fs3 = true;
        end

    end
end
