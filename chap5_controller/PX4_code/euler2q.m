function q = euler2q(rotation)

% 欧拉角转四元数

roll= rotation(1);
pitch = rotation(2);
yaw = rotation(3);

q=zeros(4,1);

cosPhi_2=cos(roll/2);
sinPhi_2=sin(roll/2);
cosTheta_2=cos(pitch/2);
sinTheta_2=sin(pitch/2);
cosPsi_2=cos(yaw/2);
sinPsi_2=sin(yaw/2);

q(1) = cosPhi_2*cosTheta_2*cosPsi_2 + sinPhi_2*sinTheta_2*sinPsi_2;
q(2) = sinPhi_2*cosTheta_2*cosPsi_2 - cosPhi_2*sinTheta_2*sinPsi_2;
q(3) = cosPhi_2*sinTheta_2*cosPsi_2 + sinPhi_2*cosTheta_2*sinPsi_2;
q(4) = cosPhi_2*cosTheta_2*sinPsi_2 - sinPhi_2*sinTheta_2*cosPsi_2;

end