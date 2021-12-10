function bool = inrange(low,x,high)
%INRANGE Return true if x is between low and high
    bool = low<x && x<high;
end

