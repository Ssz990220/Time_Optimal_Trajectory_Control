function A = get_A(sgrid,Mv,Cv,Dv,R,angle,d_phi)
%GET_A Summary of this function goes here
%   Detailed explanation goes here
global Mu FN Wf;
% tic;
A = sparse(sgrid*3,sgrid*4);
A(1:2,2:4)=[Mv(:,1),R(:,:,1)];
A(3,1:2)=[-1,2*d_phi(1)];
for k=2:sgrid
    A((k-1)*3+1:(k-1)*3+2,(k-1)*4+2:(k-1)*4+4)=[Mv(:,k),-R(:,:,k)];
    A((k-1)*3+3,(k-1)*4+1:(k-1)*4+2)=[-1,2*d_phi];
    A((k-1)*3+3,(k-1)*4-3)=1;
end
% toc;
% fprintf('Function get_A finished.');
end

