function out = control_ABFTISMC(input)
M  = [input(1), input(2), input(3);
      input(4), input(5), input(6);
      input(7), input(8), input(9)];
R  = [input(10), input(11), input(12);
      input(13), input(14), input(15);
      input(16), input(17), input(18)];
D  = [input(19), input(20), input(21);
      input(22), input(23), input(24);
      input(25), input(26), input(27)];
R_dot = [input(28), input(29), input(30);
         input(31), input(32), input(33);
         input(34), input(35), input(36)];
v  = [input(37), input(38), input(39)]';
c1 = input(40);
z1 = [input(41), input(42), input(43)]';
z1_dot = [input(44), input(45), input(46)]';
p  = input(47);
q  = input(48);
phi1 = input(49);
h1   = input(50);
h2   = input(51);
lambda2 = input(52);
h3      = input(53);
F_est   = [input(54), input(55), input(56)]';
lambda1 = input(57);
mu1 = input(58);

z2 = z1_dot+c1*z1;

s2_bar   = z1_dot +c1*z1+phi1*z1.^(p/q);

psi = zeros(3,1);
psi(1)= Psi(z1(1), s2_bar(1), p, q, mu1);
psi(2)= Psi(z1(2), s2_bar(2), p, q, mu1);
psi(3)= Psi(z1(3), s2_bar(3), p, q, mu1);

s2   = z1_dot + c1*z1 + phi1*psi;
%s2 = s2_bar;

tau = M * (R \ ( (R * (M \ D * v)) - R * F_est ...
     - R_dot*v - c1*z2 + c1^2*z1 - (p/q) * phi1 * z1.^(p/q-1)...
     - h1*s2 - h2 * abs(s2).^lambda1 .* sign(s2) - h3 * abs(s2).^lambda2 .* sign(s2)));

out = [tau; s2];

end


function psi = Psi(z1, s2_bar, p, q, mu1)
    if (s2_bar == 0) || (s2_bar ~= 0 && abs(z1) > mu1)
        psi = sign(z1) * abs(z1).^(p/q);  % Elevar a potencia sin generar números complejos
    else
        psi = z1;
    end
end
