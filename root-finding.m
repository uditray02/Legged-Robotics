function example_roots
clc
close all

x = linspace(-6,6);
for i=1:length(x)
    f(i) = fn(x(i));
end

plot(x,f,'LineWidth',2);
title('f(x) = x^2 - x - 2');
xlabel('x', 'FontSize',12);
ylabel('f(x)', 'FontSize',12);
grid on;

%equation%
x0 = 3;
options = optimoptions('fsolve','Display','iter','MaxIterations',100, 'MaxFunctionEvaluations',300);
[x, fval, exitflag] = fsolve(@fn,x0,options)
x
fval
exitflag

function f = fn(x)
f = x^2 - x - 2;
