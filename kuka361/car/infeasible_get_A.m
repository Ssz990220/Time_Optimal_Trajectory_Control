function A = infeasible_get_A(x,Mv,Cv,Dv,R,angle,d_phi)
%INFEASIBLE_GET_A Summary of this function goes here
%   Detailed explanation goes here
global Mu FN Wf;
A = get_A(x(1:end-1),Mv,Cv,Dv,R,angle,d_phi);
A = [A,zeros(size(A,1),1);zeros(1,size(A,2)),1];
end

