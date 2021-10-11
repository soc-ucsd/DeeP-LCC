function [U] = hankel_matrix(u,L)
% =========================================================================
%               Generate a Hankel matrix of order L
% =========================================================================

m = size(u,1);
T = size(u,2);

U = zeros(m*L,T-L+1);

for i = 1:L
   U((1+(i-1)*m):i*m,:) = u(:,i:(i+T-L));
end

end

