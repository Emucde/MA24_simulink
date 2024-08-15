function bspline = bsplineCurveFitting(QQ, TT, Ind_deriv, alpha, p)
% Calculates the control points of a B-spline curve
% that passes through the given points QQ and fulfills the given
% tangential constraints TT.
%
% Inputs:
%   - QQ: a (n+1 x 3) matrix containing a set of 3D points 
%   - TT: a (l+1 x 3) matrix containing a set of 3D tangential vectors
%         (optional, if not provided, the tangential vectors are calculated
%         automatically)
%   - Ind_deriv: a (1 x l-2) vector containing the indices from QQ on that
%                the tangent in TT should be. Hint: Tangents on Index 0 and
%                n are always active and must not be in Ind_deriv!
%   - alpha: a (1 x (l+2)) vector containing the design paramters that
%            defines scaling of TT with the approximated chord length d.
%   - p: a scalar representing the order of the B-spline (default: 3)
%
% Outputs:
%   - bspline: a struct containing the bspline data:
%   - bspline.degree:         degree of bspline = p
%   - bspline.knot:           calculated knots for the bspline curve
%   - bspline.control_points: a (r+1 x 3) matrix containing the calculated
%                             control points P_i for the bspline curve 
%   - bspline.parameters : a (1 x (n+1)) vector containing the theta values
%                          that should be valid at each point in QQ
%
% Notes:
%   Ind_deriv: - can be empty []. Then only the tangent on index 0 and n
%                are set.
%              - it should not contain the index 0 and n.
%   l+1 = 2+length(Ind_deriv) are the number of tangents that should be
%   set. The relation l <= n is always valid as no more deviations as
%   points can be set.
%   r+1 = n+2+l is the number of calculated control points
%
% Example usage:
%   y = [1/2, -1/4, 1/4, -1/2]; % = Q_0, Q_1, ..., Q_n
%   z = [0 0 0 0];
%   x = [0 1/3 2/3 1];
%   QQ = [x; y; z]'; % e.g. measured data
%
%   % Use custom tang vek
%   T0 = [1 -1 0];
%   Tk = [1 0 0; 1 1 0]; % = [T1; T2]
%   Tn = [0 1 1];
%
%   TT = [T0; Tk; Tn];
%
%   % T1 = [1 0 0] active at Q1 = QQ(2,:), T2 = [1 1 0] active at Q2 = QQ(3,:)
%   Ind_deriv = [1, 2]; % index 1: T1, index 2: T2
%
%   % T0 and Tn are ignored (alpha(1) = alpha(end) = 0), T1 and T2 are scaled
%   % with approximated chord length 1*d. This means for this example:
%   % alpha(1) is active on T0 = TT(1, :),
%   % alpha(2) is active on T1 = TT(1+Ind_deriv(1), :)
%   % alpha(3) is active on T2 = TT(1+Ind_deriv(2), :)
%   % alpha(end) is active on Tn = TT(end, :)
%   alpha = [0 1 1 0];
%
%   p = 3; % spline order
%
%   PDi = bsplineCurveFitting(QQ, TT, Ind_deriv, alpha, p);
%
% References:
%   [1] Farin, G. (2002). Curves and surfaces for computer graphics: A practical introduction with C++. Morgan Kaufmann.
%   [2] Piegl, L. A., & Tiller, W. (1997). The NURBS book (2th ed.). Springer Science & Business Media.


n = length(QQ)-1; % length(Q_j) = n+1
l = length(Ind_deriv)+1;
%m = n+p+1;

if(l > n)
    error('foo:bar','l should not be bigger than n. l is defined as length(Ind_deriv)+1.\nHint: You have to specify n+1 points and max. l+1 deviations can be defined.\nl = %d, n = %d', l, n);
elseif(size(TT,1) ~= l+1)
    error('foo:bar','tangvec should be a (l+1) x 2 Matrix.\nsize(tangvec) should be: %d x %d. size(tangvec) is %d x %d', l+1, size(TT, 2), size(TT, 1), size(TT, 2));
elseif(size(Ind_deriv,2)+2 ~= l+1)
    error('foo:bar', 'Only l-1 Indices should be in Ind_dervi (0 and n excluded).\nIt should be l-1=%d indices. Ind_deriv=[%s]', l-1, num2str(Ind_deriv));
elseif(any(Ind_deriv == 0) || any(Ind_deriv == n))
    error('foo:bar', 'In Ind_deriv should be only the Indices between 1 and n-1.\nn-1 = %d, Ind_deriv=[%s]', n-1, num2str(Ind_deriv));
elseif p > n+1
    error('it is not possible to use a %d order bspline for only %d points! p <= n+1', p, n+1)
end

% Berechnen der approximierten chord length Parameter
[t_tilde, d] = chordLengthParametrization(QQ');
%t_tilde = x; % tw bringt chordlenght wenig!

% t_tilde ... Parameter
% d ... length of approximated arc len

%t_tilde = [0,0.1599, 0.3198, 0.5231, 0.6997, 1];
deBoorknots = deBoorKnotVector(t_tilde, p);

% Index der Knoten: Annahme: Knoten sind sortiert
Ind = 0:n;

% Ableitungen werden mit Einheitstangentialvektor oder direkt als Ableitung
% angegeben. Für jeden der n Punkte kann optional zusätzlich eine Ableitung
% definiert werden. In Ind_deriv steht der Index des k-ten Datenpunkts Q_k, an
% dem die hier definierte Ableitung gelen sill. Für D_0 und D_n muss kein
% Index angegeben werden, diese sind immer dabei und müssen immer gesetzt
% werden. Der Faktor alpha = [alpha_0, alpha_k, ... alpha_{k+l}, alpha_n]
% ist ein design paramater, er bestimmt wie star der gewählte
% Einheitstangentialvektor mit der geschätzten chord length d multipliziert
% wird. Statt einer Angabe des Tangentialsvektors kann D_k auch direkt
% gesetzt werden.

tangvec_norm = TT./vecnorm(TT',2)';

DD = d * tangvec_norm .* alpha';

[a,b] = missing_knots(t_tilde, p, Ind, Ind_deriv);
new_knots = sort([a,b]);
acc = 15;
knot_list = round(10^acc*unique([new_knots, deBoorknots]))/10^acc; % um rundungsfehler auszuschließen
knots = sort([deBoorknots(1+(0):1+(p-1)), unique(knot_list), deBoorknots(2+(end-p-1):1+(end-1))]);

%%
r = n+l+1;
q = r+1-4; % Anzahl der Zwischenpunkte und Ableitungen [Qj, Dj, ...] für j = 1,...,l-1

% Calculate Target Vektor R: (r+1) x 2 Vektor
DQ = zeros(r+1,size(QQ,2));
DQ(1,:) = QQ(1,:);

DQ(2,:) = DD(1,:);
dev_cnt = 0;% weil D0 bereits gesetzt
for i=1:n
    DQ(2+(i+dev_cnt),:) = QQ(1+(i),:);
    if(any(Ind_deriv == i))
        dev_cnt = dev_cnt + 1;
        DQ(2+(i+dev_cnt),:) = DD(1+(dev_cnt), :);
    end
end
DQ(end-1,:) = DD(end,:);
DQ(end,:) = QQ(end,:);

use_var1 = ~false; % Beide Varianten sind 1:1 äquivalent.
% Variant 1: uses formulas from Piegl 2008, right side of equation N*P=R is
% scaled with gamma_0 and gamma_n.
% Variant 2: uses formulas from Spline C(x) and C'(x). gamma_0 and gamma_n
% are implicit in N as 1/gamma_0 and 1/gamma_n scaling.

% Calculate N: (r+1) x (r+1) Matrix
if(use_var1)
    % Effizientere Variante, da die anderen Basisfunktionen ohnehin 0
    % sind. Diese Variante kann auch für höhere Ableitungen verwendet
    % werden.

    % D_0 = C'(u_0) = p/(u_{p+1} - u_0) * (P_1 - P_0) = 1/gamma_0 * (P_1 - P_0)
    gamma_0 = (knots(1+(p+1)) - knots(1+(0)))/p;
    % Typo in Paper Piegl 2008. Correct is:
    % D_r = C'(u_r) = p/(u_{r+p+1} - u_r) * (P_{r+1} - P_r) = 1/gamma_r * (P_{r+1} - P_r)
    gamma_r = (knots(1+(r+p+1)) - knots(1+(r)))/p;

    DQ(2,:) = gamma_0 * DQ(2,:);
    DQ(end-1,:) = gamma_r* DQ(end-1,:);
    NN = zeros(r+1, r+1);
    NN(1,1) = 1;
    NN(2,1:2) = [-1,1];
    cnt = 0;
    dev_cnt = 0;
    for j=1:n-1
        i = bspline_findspan_comb(t_tilde(1+(j)), knots, p);
        k = double(any(Ind_deriv == j));
        NN(2+j+cnt:2+cnt+j+k, 1+(i-p):1+(i)) = bspline_basisfunction_comb(t_tilde(1+(j)),i,k,knots, p);
            cnt = cnt + k;
    end

    NN(1+(r-1),end-1:end) = [-1,1];
    NN(1+(r),end) = 1;
else
    NN = zeros(r+1, r+1);
    [Ni, dNi] = arrayfun(@(i) bsplineBasis(i,p,t_tilde(1+(0)),knots), 0:r);
    NN(1,:) = Ni;
    NN(2,:) = dNi;

    dev_cnt = 0;
    for j=0:n-2
        [Ni, dNi] = arrayfun(@(i) bsplineBasis(i,p,t_tilde(1+(j+1)),knots), 0:r);
        NN(3+(2*j-dev_cnt), :) = Ni;
        if(any(Ind_deriv == j+1))
            NN(3+(2*j+1-dev_cnt), :) = dNi;
        else
            dev_cnt = dev_cnt + 1;
        end
    end
    
    [Ni, dNi] = arrayfun(@(i) bsplineBasis(i,p,t_tilde(1+(n)),knots), 0:r);
    NN(1+(r-1),:) = dNi;
    NN(1+(r),:) = Ni;
end

%[NN, DQ]

% Caclulate Control Points:
PP = NN\DQ;

bspline.degree = p;
bspline.knot = knots;
bspline.control_points = PP;
bspline.tangents = DD;
bspline.parameters = t_tilde;