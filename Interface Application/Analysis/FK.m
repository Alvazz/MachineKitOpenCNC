function [ FK_points ] = FK(axis_points, tool_translation, work_translation)
%FK Summary of this function goes here
%   Detailed explanation goes here
    X = axis_points(:,1);
    Y = axis_points(:,2);
    Z = axis_points(:,3);
    A = axis_points(:,4);
    B = axis_points(:,5);
    S = -90*ones(length(X),1);

    FK_X = X.*sind(B)-Y.*sind(A).*cosd(B)+Z.*cosd(A).*cosd(B)+tool_translation(1).*sind(A).*sind(S).*cosd(B)...
        -tool_translation(1).*sind(B).*cosd(S)+tool_translation(2).*sind(A).*cosd(B).*cosd(S)+tool_translation(2)...
        .*sind(B).*sind(S)-tool_translation(3).*cosd(A).*cosd(B)-.000135.*sind(A).*cosd(B)-.000291.*sind(B)...
        +4.069679.*cosd(A).*cosd(B)-1.002354+work_translation(1);

    FK_Y = X.*cosd(B)+Y.*sind(A).*sind(B)-Z.*sind(B).*cosd(A)-tool_translation(1).*sind(A).*sind(B).*sind(S)...
        -tool_translation(1).*cosd(B).*cosd(S)-tool_translation(2).*sind(A).*sind(B).*cosd(S)+tool_translation(2)...
        .*sind(S).*cosd(B)+tool_translation(3).*sind(B).*cosd(A)+.000135.*sind(A).*sind(B)-4.069679.*sind(B)...
        .*cosd(A)-.000291.*cosd(B)+1e-6+work_translation(2);

    FK_Z = Y.*cosd(A)+Z.*sind(A)-tool_translation(1).*sind(S).*cosd(A)-tool_translation(2).*cosd(A).*cosd(S)...
        -tool_translation(3).*sind(A)+4.069679.*sind(A)+.000135*cosd(A)+.372+work_translation(3);

    FK_A = -B;
    FK_B = 90-A;

    FK_points = [FK_X FK_Y FK_Z FK_A FK_B];
end

