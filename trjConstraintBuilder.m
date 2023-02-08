function [c, ceq, gradc, gradceq] = trjConstraintBuilder(x, target)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
c = [];

if nargout > 2
    gradc = [];
    [ceq, gradceq] = trjConstraint(x, target);
else
    ceq = trjConstraint(x, target);
end

end