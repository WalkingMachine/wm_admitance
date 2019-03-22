function tau = InverseDynamic(jointVelocity, jointAngle, jointAcceleration)
    robotParam = RobotParam;
    v_dot  = zeros(3, 8);
    v_dotC = zeros(3, 8);
    w      = zeros(3, 8);
    ww     = zeros(3, 8);
    R      = zeros(3, 3, 8);
    P      = zeros(3, 8);
    Pc     = zeros(3, 7);
    F      = zeros(3, 7);
    N      = zeros(3, 7);
    f      = zeros(3, 8);
    n      = zeros(3, 8);
    tau    = zeros(7, 1);
    I      = zeros(3, 3, 7);
    
    
    for i = 1:(robotParam.mActuatorCount)
        R(:,:,i) = tform2rotm(robotParam.mJointReferentiel(:, :, i)) * (rotz(jointAngle(i) + robotParam.mJointOffset(i)));
        P(:, i)  = tform2trvec(robotParam.mJointReferentiel(:, :, i)) * (rotz(jointAngle(i) + robotParam.mJointOffset(i)));
        Pc(:, i) = robotParam.mLinkCenterOfMass(:, i);
        I(:,:,i) = robotParam.mLinkInertia(:,:,i);
    end
    
    R(:, :, robotParam.mActuatorCount + 1) = [1, 0, 0; 0, 1, 0; 0, 0, 1];
    P(:, robotParam.mActuatorCount + 1)   = [0;0;0];
    
    w(:,1)      = [0; 0; 0;];
    ww(:,1)     = [0; 0; 0;];
    v_dot(:, 1) = [0; 0; -9.81];
    
    for i = 2:(robotParam.mActuatorCount + 1)
        w(:, i)      = R(:, :, i - 1)' * w(:, i - 1) + [0; 0; jointVelocity(i - 1)];
        ww(:, i)     = R(:, :, i - 1)' * ww(:, i - 1) + cross(R(:, :, i - 1)' * w(:, i - 1), [0; 0; jointVelocity(i - 1)]) + [0; 0; jointAcceleration(i - 1)];
        v_dot(:, i)  = R(:, :, i - 1)' * (cross(ww(:, i - 1), P(:, i - 1)) + cross(w(:, i - 1), cross(w(:, i - 1), P(:, i - 1))) + v_dot(:, i - 1));
        v_dotC(:, i) = (cross(ww(:, i), Pc(:, i - 1)) + cross(w(:, i), cross(w(:, i), Pc(:, i - 1))) + v_dot(:, i));
        F(:, i - 1) = (robotParam.mLinkMass(i - 1))* v_dot(:, i);
        N(:, i - 1)   = I(:, :, i - 1) * ww(:, i) + cross(w(:, i), I(:, :, i - 1) * w(:, i));
    end
   
    f(:, robotParam.mActuatorCount + 1) = [0; 0; 0];
    n(:, robotParam.mActuatorCount + 1) = [0; 0; 0];
    
    for i = (robotParam.mActuatorCount):-1:1
        f(:, i)   = R(:, :, i + 1) * f(:, i + 1) + F(:, i);
        n(:, i)   = N(:, i) + R(:, :, i + 1) * n(:, i + 1) + cross(Pc(:, i), F(:, i)) + cross(P(:, i + 1), R(:, :, i + 1) * f(:, i + 1));
        tau(i) = n(3,i);
    end
end