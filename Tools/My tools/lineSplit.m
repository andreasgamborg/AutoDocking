function lineOut = lineSplit(lineIn,lineLength)

    lineOut = '';
    offset = 0;
    
    i = lineLength;
    n = length(lineIn);
    if(n<i), return, end
    
    lineOut(1:lineLength) = lineIn(1:lineLength);
    while i < n
        j = i+offset;
        if(lineIn(i) == '-' || lineIn(i) == '+')
            lineOut(j:j+1) = '\\';
            offset = offset+2;
            lineOut(j+2:min(j+2+lineLength,n+offset)) = lineIn(i:min(i+lineLength,n));
            i = i + lineLength;
        else
            lineOut(j) = lineIn(i);
            i = i+1;
        end
    end
end