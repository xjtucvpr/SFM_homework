function gT = gT(a, b);
%   a  (vector) of size Fx3:
%   b  (vector) of size Fx3:
%   gT (vector) of size Fx6:
gT = [ a(:,1).*b(:,1) ...
    a(:,1).*b(:,2) + a(:,2).*b(:,1) ...
    a(:,1).*b(:,3) + a(:,3).*b(:,1) ...
    a(:,2).*b(:,2) ...
    a(:,2).*b(:,3) + a(:,3).*b(:,2) ...
    a(:,3).*b(:,3) ];
end