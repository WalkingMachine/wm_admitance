function acceleration = ForwardDynamicSolver(joint, jointVelocity, tau)
     M = zeros(7,7);
     
     C = InverseDynamic(jointVelocity, joint, [0; 0; 0; 0; 0; 0; 0]);
     
     for n = 1:7
        a = [0; 0; 0; 0; 0; 0; 0];
        a(n) = 1;
        s = (InverseDynamic(jointVelocity,joint ,a) - C);
        M(n, n) = s(n);
     end 
     
     acceleration = inv(M) * (tau - C);
end