function [bsplines] = acin_text_spline_test(pos, xscale, yscale)

    %Rx = @(al) [1 0 0; 0 cos(al) -sin(al); 0 sin(al) cos(al)];
    %Ry = @(al) [cos(al) 0 sin(al); 0 1 0; -sin(al) 0 cos(al)];
    %Rz = @(al) [cos(al) -sin(al) 0;sin(al) cos(al) 0; 0 0 1];
    
    QQ_A = [34 115; 43 93; 69 22; 95 93; 69 93; 43 93; 69 93; 95 93; 103 115];
    QQ_C = [255 90; 225 115; 195 90; 195 68.5; 195 47; 225 22; 255 47];
    QQ_I = [378 22; 378 115];
    QQ_N = [506 115; 506 22; 565 115; 565 22];

    
    %% A
    QQ = [pos(1)*ones(size(QQ_A,1),1) QQ_A];
    QQ = [QQ(1:2,:); (QQ(3,:)+QQ(2,:))/2; QQ(3,:); (QQ(4,:)+QQ(3,:))/2;  QQ(4:end,:)];
    QAmin = min(QQ(:,2:3));
    QQ(:,2:3) = (QQ(:,2:3) - QAmin);
    QAmax = max(QQ);
    QQ(:,2:3) = ((QQ(:,2:3) ./ QAmax(2:3)) .* [xscale yscale]);
    
    Ind_deriv = [   2,  3,  4,    5,   6,  8,   9   ]; % Lä?nge ist = l-2
    alpha =     [1  2  0.1  2  0.15  1.8  1.6  0.2 1];
    
    % p ... spline Ordnung, n ... Anzahl der Punkte (ohne Q_0)
    p = 5;
    
    T0 = [QQ(2,:)-QQ(1,:)];
    Tk = [QQ(4,:)-QQ(3,:); 0 1 0; QQ(6,:)-QQ(5,:); 0 -1 0; 0 -1 0; 0 1 0; 0 1 0];
    Tn = [QQ(end,:)-QQ(end-1,:)];
    TT = [T0; Tk; Tn];
    
    % y und z vertauschen:
    QQ(:,2:3) = QQ(:,[3 2]) + pos(2:3);
    TT(:,2:3) = TT(:,[3 2]);

    bspline1 = bsplineCurveFitting(QQ, TT, Ind_deriv, alpha, p);
    %plot_spline(bspline1, QQ, TT, alpha, Ind_deriv, 0:0.01:1, 0.1);
    
    %% C
    QQ = [pos(1)*ones(size(QQ_C,1),1) QQ_C];
    QQ(:,2:3) = (QQ(:,2:3) - QAmin);
    QQ(:,2:3) = ((QQ(:,2:3) ./ QAmax(2:3)) .* [xscale yscale]);
    
    Ind_deriv = [3]; % Lä?nge ist = l-2
    alpha = [1 0.95 1];
    
    % p ... spline Ordnung, n ... Anzahl der Punkte (ohne Q_0)
    p = 5;
    
    T0 = [0 0 1];
    Tk = [0 0 -1];
    Tn = [0 0 1];
    TT = [T0; Tk; Tn];

    % y und z vertauschen:
    QQ(:,2:3) = QQ(:,[3 2]) + pos(2:3);
    TT(:,2:3) = TT(:,[3 2]);
    
    bspline2 = bsplineCurveFitting(QQ, TT, Ind_deriv, alpha, p);
    %plot_spline(bspline2, QQ, TT, alpha, Ind_deriv, 0:0.01:1, 0.2);
    
    %% I
    QQ = [pos(1)*ones(size(QQ_I,1),1) QQ_I];
    QQ = [QQ(1,:); repmat([pos(1), 378],3,1) (1:3)'*(QQ(1,3) + QQ(2,3))/4; QQ(end,:)];
    QQ(:,2:3) = (QQ(:,2:3) - QAmin);
    QQ(:,2:3) = ((QQ(:,2:3) ./ QAmax(2:3)) .* [xscale yscale]);
    
    Ind_deriv = []; % Lä?nge ist = l-2
    alpha = [1 1];
    
    % p ... spline Ordnung, n ... Anzahl der Punkte (ohne Q_0)
    p = 5;
    
    T0 = [0 0 1];
    Tk = [];
    Tn = [0 0 1];
    TT = [T0; Tk; Tn];

    % y und z vertauschen:
    QQ(:,2:3) = QQ(:,[3 2]) + pos(2:3);
    TT(:,2:3) = TT(:,[3 2]);
    
    bspline3 = bsplineCurveFitting(QQ, TT, Ind_deriv, alpha, p);
    %plot_spline(bspline3, QQ, TT, alpha, Ind_deriv, 0:0.01:1, 0.1);
    
    %% N
    QQ = [pos(1)*ones(size(QQ_N,1),1) QQ_N];
    QQ = [QQ(1,:); (QQ(1,:)+QQ(2,:))/2; QQ(2,:); (QQ(2,:)+QQ(3,:))/2; QQ(3,:); (QQ(3,:)+QQ(4,:))/2; QQ(end,:)];
    QQ(:,2:3) = (QQ(:,2:3) - QAmin);
    QQ(:,2:3) = ((QQ(:,2:3) ./ QAmax(2:3)) .* [xscale yscale]);
    
    Ind_deriv = [1,2,3,4,5]; % Lä?nge ist = l-2
    alpha = [2 1 0.1 1.6 0.1 1 2];
    
    % p ... spline Ordnung, n ... Anzahl der Punkte (ohne Q_0)
    p = 5;
    
    T0 = [0 0 -1];
    Tk = [0 0 -1; 0 1 0; QQ(5,:)-QQ(3,:); 0 1 0; 0 0 -1];
    Tn = [0 0 -1];
    TT = [T0; Tk; Tn];

    % y und z vertauschen:
    QQ(:,2:3) = QQ(:,[3 2]) + pos(2:3);
    TT(:,2:3) = TT(:,[3 2]);
    
    bspline4 = bsplineCurveFitting(QQ, TT, Ind_deriv, alpha, p);
    %plot_spline(bspline4, QQ, TT, alpha, Ind_deriv, theta_arr, 20);
    
    bsplines = [bspline1, bspline2, bspline3, bspline4];

end