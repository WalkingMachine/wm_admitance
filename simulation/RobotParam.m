classdef RobotParam
    properties
        mActuatorCount = 7;
        mJointReferentiel = zeros(4, 4, 7);
        mLinkCenterOfMass = zeros(3, 7);
        mLinkInertia = zeros(3, 3, 7);
        
        mT01 = RobotParam.HomogeniousMatrix(0, -pi/2, 0, 0, -0.1437625, 0.01166);
        mT12 = RobotParam.HomogeniousMatrix(-pi/2, 0, 0, 0, -0.085, 0);
        mT23 = RobotParam.HomogeniousMatrix(pi/2, 0,  pi/2, -0.085, 0, 0);
        mT34 = RobotParam.HomogeniousMatrix(-pi/2, -pi/2, 0 , 0, 0, -0.31);
        mT45 = RobotParam.HomogeniousMatrix(0, 0, -pi/2 , 0, 0.1457, 0);
        mT56 = RobotParam.HomogeniousMatrix(pi/2, -pi/2, 0 , -0.0606, 0, 0);
        mT67 = RobotParam.HomogeniousMatrix(pi, 0, pi/2, 0, 0.284375, 0);
        
                 
        mJointOffset;            
        mLinkMass = [0.28506911; 0.28506911; 0.85240803; 0.28300045; 0.1567; 0.18747684; 0.02796];

    end
    methods
        function obj = RobotParam
            obj.mJointOffset = zeros(1, obj.mActuatorCount); 
            obj.mJointReferentiel(:,:,1) = obj.mT01;
            obj.mJointReferentiel(:,:,2) = obj.mT12;
            obj.mJointReferentiel(:,:,3) = obj.mT23;
            obj.mJointReferentiel(:,:,4) = obj.mT34;
            obj.mJointReferentiel(:,:,5) = obj.mT45;
            obj.mJointReferentiel(:,:,6) = obj.mT56;
            obj.mJointReferentiel(:,:,7) = obj.mT67;
            
            obj.mLinkCenterOfMass(:,1) = [0.0; -0.03225261; -0.01842117];
            obj.mLinkCenterOfMass(:,2) = [-0.03225261; 0; -0.01842117];
            obj.mLinkCenterOfMass(:,3) = [-0.00168302; 0.00155140; -0.13913505];
            obj.mLinkCenterOfMass(:,4) = [0.00000207; 0.03922132; -0.01628605];
            obj.mLinkCenterOfMass(:,5) = [-0.0195;    0.0001;   -0.0181];
            obj.mLinkCenterOfMass(:,6) = [0.00001243; 0.05721169; 0.0];
            obj.mLinkCenterOfMass(:,7) = [0.0; 0.0; -0.01570460];
            
%             obj.mLinkInertia(:,:,1) = [[0.00040678, 0, 0];[0, 0.00039958, 0];[0, 0, 0.00059825]];
%             obj.mLinkInertia(:,:,2) = [[0.00039958, 0, 0];[0, 0.00040678, 0];[0, 0, 0.00059825]]; 
%             obj.mLinkInertia(:,:,3) = [[0.00538199, 0, 0];[0, 0.00562525, 0];[0, 0, 0.00050159]]; 
%             obj.mLinkInertia(:,:,4) = [[0.00036109, 0, 0];[0, 0.00029872, 0];[0, 0, 0.00055069]]; 
%             obj.mLinkInertia(:,:,5) = [[0.54274952, 0, 0];[0, 1.03078569, 0];[0, 0, 0.00010905]]; 
%             obj.mLinkInertia(:,:,6) = [[0.00015391, 0, 0];[0, 0.00006260, 0];[0, 0, 0.00015218]]; 
%             obj.mLinkInertia(:,:,7) = [[0.00000889, 0, 0];[0, 0.00002569, 0];[0, 0, 0.00002493]]; 
            
            obj.mLinkInertia(:,:,1) = [1, 0, 0; 0, 1, 0; 0, 0, 1];
            obj.mLinkInertia(:,:,2) = [1, 0, 0; 0, 1, 0; 0, 0, 1]; 
            obj.mLinkInertia(:,:,3) = [1, 0, 0; 0, 1, 0; 0, 0, 1]; 
            obj.mLinkInertia(:,:,4) = [1, 0, 0; 0, 1, 0; 0, 0, 1]; 
            obj.mLinkInertia(:,:,5) = [1, 0, 0; 0, 1, 0; 0, 0, 1]; 
            obj.mLinkInertia(:,:,6) = [1, 0, 0; 0, 1, 0; 0, 0, 1]; 
            obj.mLinkInertia(:,:,7) = [1, 0, 0; 0, 1, 0; 0, 0, 1];
        end
    end
    
    methods(Static)
        function T = HomogeniousMatrix(r, p, y1, x, y, z)
            eul = [y1 p r];
            T = rotm2tform(eul2rotm(eul)) + [0 0 0 x; 0 0 0 y; 0 0 0 z; 0 0 0 0];   
        end
    end
end
