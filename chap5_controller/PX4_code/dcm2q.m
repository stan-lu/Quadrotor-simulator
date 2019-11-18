function q= dcm2q(dcm)

q=zeros(4,1);

tr=dcm(1,1)+dcm(2,2)+dcm(3,3);

if tr>0
   s=sqrt(1+tr);
   q(1)=s*0.5;
   s=0.5/s;
   q(2)=(dcm(3,2)-dcm(2,3))*s;
   q(3)=(dcm(1,3)-dcm(3,1))*s;
   q(4)=(dcm(2,1)-dcm(1,2))*s;
else
    % 找到 dcm 对角线上最大的数并把（其位置-1）存放在 dcm_i 中
    dcm_i=0;
    for i=1:2
        if dcm(i+1,i+1)>dcm(dcm_i+1,dcm_i+1)
            dcm_i=i;
        end
    end
    
    dcm_j=mod(dcm_i+1,3);
    dcm_k=mod(dcm_i+2,3);
    s=sqrt(dcm(dcm_i+1,dcm_i+1)-dcm(dcm_j+1,dcm_j+1)-dcm(dcm_k+1,dcm_k+1)+1);
    q(dcm_i+2)=s*0.5;
    s=0.5/s;
    q(dcm_j+2)=(dcm(dcm_i+1,dcm_j+1)+dcm(dcm_j+1,dcm_i+1))*s;
    q(dcm_k+2)=(dcm(dcm_k+1,dcm_i+1)+dcm(dcm_i+1,dcm_k+1))*s;
    q(1)=(dcm(dcm_k+1,dcm_j+1)-dcm(dcm_j+1,dcm_k+1))*s;
end
end
