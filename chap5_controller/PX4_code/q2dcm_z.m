function R_z = q2dcm_z(q)

R_z = zeros(3,1);
a = q(1); b = q(2); c = q(3); d = q(4);
R_z(1) = 2 * (a * c + b * d);
R_z(2) = 2 * (c * d - a * b);
R_z(3) = a * a - b * b - c * c + d * d;

end

