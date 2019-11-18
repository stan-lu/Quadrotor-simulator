function q = two_Vec2q(src, dst)

cr = cross(src, dst);
dt = dot(src, dst);
q = zeros(4,1);
if norm(cr) < 1e-5 && dt < 0
    cr = abs(src);
    if cr(1) < cr(2)
        if cr(1) < cr(3)
            cr = [1; 0; 0];
        else
            cr = [0; 0; 1];
        end
    else
        if cr(2) < cr(3)
            cr = [0; 1; 0];
        else
            cr = [0; 0; 1];
        end
    end
    q(1) = 0;
    cr = cross(src, cr);
else
    q(1) = dt + sqrt(dot(src, src) * dot(dst, dst));
end

q(2) = cr(1);
q(3) = cr(2);
q(4) = cr(3);
q = q / norm(q);

% 以上计算等价于 q = [cos(alpha/2);
%                    sin(alpha/2) * cross(src,dst) / norm(cross(src,dst))];

end

