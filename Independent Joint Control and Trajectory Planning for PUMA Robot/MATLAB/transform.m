function T = transform(d,thi,a,alpha)
T = [
     cos(thi),  -cos(alpha).*sin(thi),  sin(alpha).*sin(thi), a.*cos(thi)
     sin(thi),   cos(alpha).*cos(thi), -sin(alpha).*cos(thi), a.*sin(thi)
     0,          sin(alpha),            cos(alpha),           d
     0,          0,                     0,                    1
     ];
end