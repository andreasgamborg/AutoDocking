function latexeq(left,right)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
sympref('FloatingPointOutput',true);
sympref('MatrixWithSquareBrackets',true);

l = latex(sym(right));
eq = left + " = " + l;

formatSpec = "\\begin{equation*}\r \t%s \r\\end{equation*}";

leq = sprintf(formatSpec,eq);

disp(leq)
end

