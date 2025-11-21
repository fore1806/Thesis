function M = transformationMatrix(theta, d, l, r)
    % M Matrix
    % Finding the transformation Matrix between the speed in the point P
    % reference frame to the wheels angular speed
    %
    % Arguments
    % - theta: Current angle
    % - d: Distance between wheels
    % - l: Distance to point P
    % - r: Radius of the wheels
    %
    % Returns
    % - M: The transformation Matrix

    M = (1/(2*r))*[2*cos(theta)-d/l*sin(theta)     2*sin(theta)+d/l*cos(theta);
                    2*cos(theta)+d/l*sin(theta)     2*sin(theta)-d/l*cos(theta)];
end