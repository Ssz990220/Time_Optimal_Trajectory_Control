function ok = mycheckb(x)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
[b,a,u] = mygetbau(x);
ok = ~(min(b>0)>0);


end

