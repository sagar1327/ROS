clc;clear;
load("Data.mat")
load("Model.mat")

p1 = model.p1;p2 = model.p2;
p3 = model.p3;p4 = model.p4;
p5 = model.p5;p6 = model.p6;

syms x
for i = 1:size(distBetweenChassisCutouts,1)
    Eq = p1*x^5 + p2*x^4 + p3*x^3 + p4*x^2 + p5*x + p6 - distBetweenChassisCutouts(i,1);
    h(i,:) = abs(double(root(Eq,x)));
end