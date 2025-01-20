function [x, p] = robusto(x, p, Q, gk, Fk, H, R, m, LambdaHat, Ef, Eg, N)

% Ruído
u = sqrt(Q)*randn(3,1);
v = sqrt(R)*randn(4,1);
 
% Simulação do sistema real
% a = 1;
% b = -1;
% delta =  a + (b-a).*rand(1);
%delta = 0; %-0.8508;
%deltaF = m * delta * Ef;
%deltaG = m * delta * Eg;

%x = (Fk + deltaF) * x + (gk + deltaG) * u;
%y = H * x + v;

invQhat = Q^-1 + LambdaHat * Eg' * (eye(2 * N + 3) + LambdaHat * Ef * p * Ef')^-1 * Eg;
QHat = (invQhat)^-1;
RHat = R - invLambdaHat*H*M*M'*H';
%PHat = (P^-1 +LambdaHat*Ef'*Ef)^-1;
PHat = P -P*Ef'*(invLambdaHat*eye(1)+Ef*P*Ef')^-1*Ef*P;
GHat = G - LambdaHat*F*PHat*Ef'*Eg;
FHat = (F-LambdaHat*GHat*QHat*Eg'*Ef)*(eye(2)-LambdaHat*PHat*Ef'*Ef);

xpHat = FHat*xHat;
e = (y - H*xpHat);
Re = RHat+H*P*H';
xHat = xpHat + P*H'*(Re)^-1*e;
%xHat = xpHat + P*H'*(H*P*H'+RHat)^-1*e;
Pp = F*PHat*F' + GHat*QHat*GHat';

P = Pp -Pp*H'*Re^-1*H*Pp;

xv(:,i) = x;
xHatv(:,i) = xHat;