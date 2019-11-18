function R = q2dcm(q)

R=zeros(3,3);
aSq=q(1)*q(1);
bSq=q(2)*q(2);
cSq=q(3)*q(3);
dSq=q(4)*q(4);
q1=q(1);
q2=q(2);
q3=q(3);
q4=q(4);


R(1,1)=aSq+bSq-cSq-dSq;
R(1,2)=2*(q2*q3-q1*q4);
R(1,3)=2*(q1*q3+q2*q4);
R(2,1)=2*(q2*q3+q1*q4);
R(2,2)=aSq-bSq+cSq-dSq;
R(2,3)=2*(q3*q4-q1*q2);
R(3,1)=2*(q2*q4-q1*q3);
R(3,2)=2*(q1*q2+q3*q4);
R(3,3)=aSq-bSq-cSq+dSq;
end


