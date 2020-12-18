function x = mygenx(b,a,u)
%GEN_X Summary of this function goes here
%   Detailed explanation goes here
global sgrid r p;
x = zeros(size(b,1)+size(a,1)+size(u,1),1);
for k = 1:size(b)
    x((k-1)*(r+2)+1:k*(r+2),1)=[b(k);a(k);u((k-1)*2+[1:r])];
end
    
end

