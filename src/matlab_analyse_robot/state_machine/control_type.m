classdef control_type
    %CONTROL_TYPE Summary of this class goes here
    %   Detailed explanation goes here

    enumeration
        jp_reg, jp_track1, jp_track2, op_track_lin, op_track1, op_track2
    end

    properties
        reg_time = 5;
        track_lin_time = 5;
        track1_time = 20;
        track2_time = 20;
    end

end
