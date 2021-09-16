function ref = makeref(stepvalues,steplength,noisevar)
    %UNTITLED2 Summary of this function goes here
    %   Detailed explanation goes here
    
    [n_series,n_steps] = size(stepvalues);

    if nargin < 2
        steplength = 100;
    end
    if nargin < 3
        noisevar = 0;
    end

    
%     stepvalue = sqrt(var)*randn(sections,1);
%     steplength = floor(n/(sections+1));

%     ref = zeros(1,n-steplength*sections); % zeros padding start
    ref = [];
    for value = stepvalues
        ref = [ref value.*ones(n_series,steplength)];
    end
    
    noise = sqrt(noisevar)*randn(size(ref));
    ref = ref+noise;
end

