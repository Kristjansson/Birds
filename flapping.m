% Produces periodic thrust
% mode 1 = sinusoidal
% mode 2 = square wave
% wing beat frequency calculated form "Speeds and wingbeat frequencies of migrating birds compared with calculated benchmarks" BY C. J. PENNYCUICK

function flapthrust = flapping(T)
    flapBeatFrequency = 10;
%     typethrust =  [sin(2*pi*(flapBeatFrequency)*T)+1, square(2*pi*(flapBeatFrequency)*T)+1];
%     flapthrust = typethrust(mode); 
    flapthrust = sin(2*pi*(flapBeatFrequency)*T)+1;
end