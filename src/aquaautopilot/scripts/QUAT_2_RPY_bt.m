function [rpy] = QUAT_2_RPY_bt(q)
    R = QUAT_2_ROTMAT_bt(q);
    rpy = [ 0 0 0 ];
    rpy2 = [ 0 0 0 ];
    
    if (abs(R(3, 1)) >= 1)
        delta = atan2(R(3, 2), R(3, 3));
        if (R(3, 1) < 0)  % gimbal locked down
            rpy(2)  = pi/2;
            rpy2(2) = pi/2;
            rpy(1)  = delta;
            rpy2(1) = delta;
            
        else  % gimbal locked up
            rpy(2)  = -pi/2;
            rpy2(2) = -pi/2;
            rpy(1)  = delta;
            rpy2(1) = delta;
        end
        
    else
        
        rpy(2)  = -asin(R(3,1));
        rpy2(2) = pi - rpy(2);
        
        rpy(1)  = atan2( R(3,2)/cos(rpy(2)), R(3,3)/cos(rpy(2))); 
        rpy2(1)  = atan2( R(3,2)/cos(rpy2(2)), R(3,3)/cos(rpy2(2)));
                
        rpy(3)  = atan2( R(2,1)/cos(rpy(2)), R(1,1)/cos(rpy(2))); 
        rpy2(3)  = atan2( R(2,1)/cos(rpy2(2)), R(1,1)/cos(rpy2(2)));        
    end
end

function [R] = QUAT_2_ROTMAT_bt(q)
    s = 2/(norm(q)*norm(q));
    
    xs = q(1) * s;
    ys = q(2) * s;
    zs = q(3) * s;
    
    wx = q(4) * xs;
    wy = q(4) * ys;
    wz = q(4) * zs;
    
    xx = q(1) * xs;
    xy = q(1) * ys;
    xz = q(1) * zs;
    
    yy = q(2) * ys;
    yz = q(2) * zs;
    
    zz = q(3) * zs;
    
    R = [ 1 - yy - zz,  xy - wz,   xz + wy; ...
          xy + wz, 1 - xx - zz, yz - wx; ...
          xz - wy, yz + wx, 1 - xx - yy ];    
end

function [q] = RPY_2_QUAT_bt(r, p, y)
    hy = y/2;
    hp = p/2;
    hr = r/2;
    
    cy = cos(hy);
    sy = sin(hy);
    
    cp = cos(hp);
    sp = sin(hp);
    
    cr = cos(hr);
    sr = sin(hr);
    
    q = [(sr*cp*cy - cr*sp*sy), ...
         (cr*sp*cy + sr*cp*sy), ...
         (cr*cp*sy - sr*sp*cy), ...
         (cr*cp*cy + sr*sp*sy)];
end


function [q] = AXISANGLE_2_QUAT_tf(axis, angle)
    d = norm(axis);
    s = sin(angle/2)/d;
    q = [axis(1)*s  axis(2)*s  axis(3)*s   cos(angle/2)];    
end
