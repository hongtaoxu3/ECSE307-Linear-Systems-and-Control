classdef VirtualMotor < handle
    
    properties(Access = public)
        angle
        velocity
        current
        voltage
        time
        angle_final
        velocity_final
        current_final
    end
    
    properties(GetAccess = private, SetAccess = private)
        status
        t
        TimeData = [];
        VoltageData = [];
        CurrentData = [];
        SpeedData = [];
        AngleData = [];
        
        % Recovery parameters
        RecoveryFileName = 'dataVirtualMotor.mat';
        
    end
    
    methods(Access = public)
        
        function self = on(self)
            self.angle_final = 0;
            self.velocity_final = 0;
            self.current_final = 0;
            self.voltage = [];
            self.t = [];
            self.angle = [];
            self.velocity = [];
            self.current = [];
            self.time = [];
            self.voltage = [];
            self.status = true;
        end
        
        function self = off(self)
            self.status = false;
        end

        function self = reset(self)
            self.off;
            self.on;
        end
        
        function self = wait(self,tin)
            self.t = tin;
        end
        
        function self = input(self,volt)

            warning('off')
            
            idx = find(volt>7);
            if ~isempty(idx)
                warning('Input voltage breaches maximum voltage limit: Value saturated to +7V.')
                volt(idx) = 7;
            end
            
            idx = find(volt<-7);
            if ~isempty(idx)
                warning('Input voltage breaches minimum voltage limit: Value saturated to -7V.')
                volt(idx) = -7;
            end

            if ~isempty(self.t)
                if isempty(self.time) && self.t == 0
                    tt = 0:0.01:0.01;
                elseif isempty(self.time) && self.t ~= 0
                    tt = 0:0.01:self.t;
                else
                    tt = self.time(end):0.01:self.t;
                end
            end
            
            if length(self.t)>1  && length(volt)==1
                vsim = volt*ones(1,length(self.t));
                
            elseif isempty(self.t) && length(volt)>1
                vsim = volt;
                tsim(:,1) = 0;
                for i = 2:length(vsim)
                    tsim(:,i) = tsim(:,i-1) + 0.01;
                end
                
            elseif length(volt)==1 && length(self.t)==1 && length(tt) == 2

                if isempty(self.time) && self.t == 0
                    tsim = 0:0.01:0.01;
                elseif isempty(self.time) && self.t ~= 0
                    tsim = 0:0.01:self.t;
                else
                    tsim = self.time(end):0.01:self.t;
                end
                
                vsim = volt*ones(1,length(tsim));
                   
                self.time = [self.time, self.t];
                self.voltage = [self.voltage, volt];
                
            elseif length(volt)==1 && length(self.t)==1 && length(tt) ~= 2
                
                tsim = 0:0.01:self.t;
                vsim = volt*ones(1,length(tsim));
                
                if ~isempty(self.time)
                    self.time = [self.time, tsim+self.time(:,end)+0.01];
                else
                    self.time = tsim;
                end
                
                self.voltage = [self.voltage, vsim];
            end

            J = 2e-05;                % moment of inertia of the rotor kg.m^2/s^2
            b = 1e-06;                % motor viscous friction constant N.m.s
            K = 0.034;                % motor torque constant N.m/Amp
            R = 8.4;                  % Ohms
            L = 0.018;                % Henrys
            
            A = [0      1       0;
                 0      -b/J    K/J;
                 0      -K/L    -R/L];
             
            B = [0; 
                 0; 
                 1/L];
             
            C = eye(3);

            D = zeros(3,1);
            
            sys = ss(A,B,C,D);

            x0 = [self.angle_final;self.velocity_final;self.current_final];
            y = lsim(sys,vsim,tsim,x0);

            if length(volt)==1 && length(self.t)==1 && length(tt) == 2
                x1 = y(end,1);
                x2 = y(end,2);
                x3 = y(end,3);
            elseif length(volt)==1 && length(self.t)==1 && length(tt) ~= 2
                x1 = y(:,1);
                x2 = y(:,2);
                x3 = y(:,3);
            else
                x1 = y(:,1);
                x2 = y(:,2);
                x3 = y(:,3);
            end

            if ~isempty(self.angle) && length(tt) ~= 2
                self.angle(:,end) = [];
                self.velocity(:,end) = [];
                self.current(:,end) = [];
                self.voltage(:,end) = [];
                self.time(:,end) = [];
            end
            
            if isempty(self.t)
                self.voltage = volt';
                self.time = tsim;
            end
            
            self.angle = [self.angle, (x1+rand(size(x1)))'];
            self.velocity = [self.velocity, (x2+rand(size(x2)))'];
            self.current = [self.current, x3'];
            
            self.angle_final = self.angle(:,end);
            self.velocity_final = self.velocity(:,end);
            self.current_final = self.current(:,end);
            
        end
       
        function save(self, students, filename, tag)
            %SAVE [INTERFACE] Save data with the name tag in filename
            
            if nargin < 4
                tag = filename;
                filename = self.RecoveryFileName;
            end
            
            % remove extension and path
            [~, filename, ~] = fileparts(filename);
            
            tag = regexprep(tag, '[\s\.-]{1}', '_', 'all');
            
            data.TimeData = self.time;
            data.VoltageData = self.voltage;
            data.CurrentData = self.current;
            data.SpeedData = self.velocity;
            data.AngleData = self.angle;
            for i = 1:numel(students)
                data.Students(i).ID = dec2hex(str2double(students(i).ID)*7907);
            end
            
            eval([tag, '= data;']);
            
            if exist([filename, '.mat'], 'file') == 0
                eval(['save(filename, ''', tag, ''');']);
            else
                eval(['save(filename, ''', tag, ''', ''-append'');']);
            end
        end
      
        function ok = load(self, filename, tag)
            %LOAD [INTERFACE] Load data with the name tag from filename
            
            if nargin < 3
                tag = filename;
                filename = self.RecoveryFileName;
            end
            
            ok = 0;
            
            % remove extension and path
            [~, filename, ~] = fileparts(filename);
            
            if exist([filename, '.mat'], 'file') == 0
                warning('VirtualMotor::load::Warning: Recovery file %s not found.', filename)
                return
            end
            
            tag = regexprep(tag, '[\s\.-]{1}', '_', 'all');

            eval(['load(filename, ''', tag, ''');']);
            try
                eval(['data = ', tag, ';']);
            catch
                warning('VirtualMotor::load::Warning: Could not load data %s from Recovery file %s.', tag, filename)
                return
            end
            
            ids = nan(1, numel(data.Students));
            for i = 1:numel(data.Students)
                ids(i) = hex2dec(data.Students(i).ID) / 7907;
            end
            fprintf(['VirtualMotor::load::info: Recovering data collected by students: ', repmat('[%d] ', size(ids)), '.\n'], ids);

            self.time = data.TimeData;
            self.voltage = data.VoltageData;
            self.current = data.CurrentData;
            self.velocity = data.SpeedData;
            self.angle = data.AngleData;
            
            ok = 1;
        end
 
    end
    
end