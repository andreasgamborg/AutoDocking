function knots = toKnots(v,from)
%toKnots convert af speed to knots
%   toKnots(v,'m/s')
%   toKnots(v,'km/t')

    if (nargin == 1), from = 'm/s';  end

    NM = 1852;   %[m] a Nautical mile
    
    if(strcmp(from,'m/s'))
        knots = v*3600/NM;
    elseif(strcmp(from,'km/t'))
        knots = v/(NM/1000);
    end
end

