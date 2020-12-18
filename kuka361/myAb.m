function [A,b] = myAb(Mv,Cv,Dv,d_phi)
%MYA Summary of this function goes here
%   Detailed explanation goes here
global sgrid r p
A = sparse(sgrid*(p+1),sgrid*(2+r));
b = zeros(sgrid*(p+1),1);
for k = 1:sgrid
    A((k-1)*(p+1)+[1:p],(k-1)*(r+2)+[1:r+2])=[Cv(:,k)/2,Mv(:,k),eye(r)];
    A((k-1)*(p+1)+p+1,(k-1)*(r+2)+[1:2])=[-1,2*d_phi];
    b((k-1)*(p+1)+[1:p])=Dv(:,k);
end
for k = 2:sgrid
    A((k-1)*(p+1)+[1:p+1],(k-2)*(r+2)+1)=[Cv(:,k)/2;1];
end

    
end

