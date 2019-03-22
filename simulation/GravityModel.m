function tau = GravityModel(jointAngle)
    robotParam = RobotParam;
    v_dot(:, 1) = [0; 0; -9.81];
    
    for i = 1:(robotParam.mActuatorCount)
        R(:,:,i) = tform2rotm(robotParam.mJointReferentiel(:, :, i)) * (rotz(jointAngle(i) + robotParam.mJointOffset(i)));
        P(:, i)  = tform2trvec(robotParam.mJointReferentiel(:, :, i));
        Pc(:, i) = robotParam.mLinkCenterOfMass(:, i);
    end
    
    R(:,:, robotParam.mActuatorCount + 1) = eye(3,3);
    P(:, robotParam.mActuatorCount + 1)   = [0;0;0];
    
    for i = 2:(robotParam.mActuatorCount + 1)
        v_dot(:, i) = R(:, :, i - 1)' * v_dot(:, i - 1);
        F(:, i - 1) = robotParam.mLinkMass(i - 1) * v_dot(:, i);
    end
    
    f(:, robotParam.mActuatorCount + 1) = [0; 0; 0];
    n(:, robotParam.mActuatorCount + 1) = [0; 0; 0];
    
    for i = (robotParam.mActuatorCount):-1:1
        f(:, i)   = R(:, :, i + 1) * f(:, i + 1) + F(:, i);
        n(:, i)   = R(:, :, i + 1) * n(:, i + 1) + cross(Pc(:, i), F(:, i)) + cross(P(:, i + 1), R(:, :, i + 1) * f(:, i + 1));
        tau(i) = n(3,i);
    end  
end