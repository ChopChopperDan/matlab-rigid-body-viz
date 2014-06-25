function R = q2R(q)
    R = eye(3)+2*q(1)*hat(q(2:4))+2*hat(q(2:4))*hat(q(2:4));
end