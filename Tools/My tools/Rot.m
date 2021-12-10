function R = Rot(psi,dim)

    if nargin < 2
        dim = 3;
    end

    if dim == 3
        R = [   cos(psi)   -sin(psi)    0
                sin(psi)    cos(psi)    0
                       0           0    1 ];
    elseif dim == 2

        R = [   cos(psi)   -sin(psi)
                sin(psi)    cos(psi)     ];
    end
end