function latexeq(left,right,type)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
sympref('FloatingPointOutput',true);
sympref('MatrixWithSquareBrackets',true);

if nargin < 3
    type = 'equation*';
end

l = latex(sym(right));
eq = left + " = " + l;

formatSpec = "\\begin{"+type+"}\r \t%s \r\\end{"+type+"}";

leq = sprintf(formatSpec,eq);

disp(leq)
end

