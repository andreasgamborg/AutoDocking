function ref = makeref(stepvalue,steplength,noisevar)
    %UNTITLED2 Summary of this function goes here
    %   Detailed explanation goes here
    
    n_steps = length(stepvalue);
    
    if nargin < 3
        noisevar = 0;
    end
    
    if isempty(steplength)
        steplength = floor(n/(n_steps));
    end
    n = sum(steplength);

    
%     stepvalue = sqrt(var)*randn(sections,1);
%     steplength = floor(n/(sections+1));

%     ref = zeros(1,n-steplength*sections); % zeros padding start
    ref = [];
    for it = 1:n_steps
        ref = [ref stepvalue(it)*ones(1,steplength(it))];
    end
    
    noise = sqrt(noisevar)*randn(1,n);
    ref = ref+noise;
end

