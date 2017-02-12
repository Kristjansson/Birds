clear all; close all; clc;

testSize = 1000;
testA = rand(testSize, testSize, 3);
testB = rand(testSize, 3, testSize);
testC = rand(3, testSize, testSize);

testAout = zeros(testSize,testSize);
testBout = zeros(testSize,testSize);
testCout = zeros(testSize,testSize);

tic;
for itr=1:testSize
    for jtr=1:testSize
        testCout(itr, jtr) = sqrt(sum(testC(:, itr, jtr).^2));
    end
end
toc;

tic;
for itr=1:testSize
    for jtr=1:testSize
        testBout(itr, jtr) = sqrt(sum(testB(itr, :, jtr).^2));
    end
end
toc;

tic;
for itr=1:testSize
    for jtr=1:testSize
        testAout(itr, jtr) = sqrt(sum(testA(itr, jtr, :).^2));
    end
end
toc;