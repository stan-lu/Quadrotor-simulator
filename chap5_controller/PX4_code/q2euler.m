function eulerAngle=q2euler(q)
eulerAngle=zeros(3,1);

eulerAngle(1,1)=atan2(2*(q(1)*q(2)+q(3)*q(4)),1-2*(q(2)*q(2)+q(3)*q(3)));
eulerAngle(2,1)=asin(2*(q(1)*q(3)-q(4)*q(2)));
eulerAngle(3,1)=atan2(2*(q(1)*q(4)+q(2)*q(3)),1-2*(q(3)*q(3)+q(4)*q(4)));

end