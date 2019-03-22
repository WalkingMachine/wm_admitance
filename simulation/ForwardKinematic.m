classdef ForwardKinematic
    properties
        mRobotParam = RobotParam;
        mFrame;
        mJacobian = zeros(6,7);
        mLastFrame;
    end
    methods
        function obj = updateJointFrame(obj, jointAngles)
            obj.mFrame(:, :, 1) = obj.updateJoint(jointAngles, 1);
            
            for n = 2:obj.mRobotParam.mActuatorCount
                obj.mFrame(:, :, n) = obj.mFrame(:, :, n - 1) * obj.updateJoint(jointAngles, n);
            end
            obj.mLastFrame = obj.mFrame(:, :, obj.mRobotParam.mActuatorCount);
            obj.mJacobian  = obj.updateJacobian(obj.mFrame);
        end
        
        function T = updateJoint(obj, angle, joint)
            T = obj.mRobotParam.mJointReferentiel(:, :, joint) * rotm2tform(rotz(angle(joint) + obj.mRobotParam.mJointOffset(joint)));
        end
        
        function jacobian = updateJacobian(obj, frame)
            
            for k = 1:obj.mRobotParam.mActuatorCount
                rotMatrix = tform2rotm(frame(:, :, k));
                angulaVelocity = rotMatrix(:, 3).';
                linearVelocity = obj.computeLinearVelocity(rotMatrix, frame, k);                
                obj.mJacobian(:, k) = [linearVelocity, angulaVelocity];
            end
            
            jacobian = obj.mJacobian;
        end
        
        function v = computeLinearVelocity(obj, rotMatrix, frame, joint)
            Pi = tform2trvec(frame(:, :, joint)) - tform2trvec(frame(:, :, obj.mRobotParam.mActuatorCount));
            ai = rotMatrix(:, 3);
            v = cross(Pi, ai);
        end
    end
end