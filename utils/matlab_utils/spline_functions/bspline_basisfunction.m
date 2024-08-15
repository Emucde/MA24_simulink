    function ders  = bspline_basisfunction(theta,knotNum,derOrd,bspline) 
  % B-Spline Basisfunktionen und deren Ableitungen an theta berechnen
  % Algorithmus vom A2.3 Nurbs Book (S. 72f) verwendet
  % Indexierung muss angepasst werde (Index startet in C bei 0 und in Matlab
  % bei 1)
  % Inputs:
  %     theta       : the path parameter
  %     knotNum     : the knot number where the path parameter lies in
  %     derOrd      : the number of derivatives that are calculated
  %     bspline     : parameter struct for the bspline settings
  %
  % Outputs:
  %     ders        : Basis functions and their derivatives
  %     

  i = knotNum;
  p = bspline.degree;
  u = theta;
  U = bspline.knot;
  n = derOrd; % bis zur derOrd-ten Ableitung berechnen
  ndu = zeros(p+1,p+1); % Basisfunktionen und Knot Differenz
  ndu(1,1) = 1;
  a = zeros(2,p+1); % Zwischenspeicher
  left = zeros(p+1,1);
  right = zeros(p+1,1);
  ders = zeros(n+1,p+1); % Basisfunktionen und deren Ableitungen
  for j=1:p
      left(j+1) = u-U(i+1-j+1); % Index!!
      right(j+1) = U(i+j+1)-u;  % Index!!
      saved = 0;
      for r=0:j-1 
          % Lower triangle
          ndu(j+1,r+1)=right(r+2)+left(j-r+1); % Index!!
          temp = ndu(r+1,j)/ndu(j+1,r+1);

          % Upper triangle
          ndu(r+1,j+1) = saved+right(r+2)*temp;
          saved = left(j+1-r)*temp;
      end
      ndu(j+1,j+1)=saved; % Index!!
  end
  ders(1,:) = ndu(:,p+1)'; % Speichern der Basisfunktionen

  % Berechnung der Ableitungen
  for r=0:p
      s1=0;
      s2=1;
      a(1,1) = 1;   
      % Berechnung der k-ten Ableitung
      for k=1:n
          d = 0;
          rk = r-k;
          pk = p-k;
          if r >= k
              a(s2+1,1) = a(s1+1,1)/ndu(pk+2,rk+1);
              d = a(s2+1,1)*ndu(rk+1,pk+1);
          end
          if rk>=-1
              j1=1;
          else
              j1=-rk;
          end
          if r-1<=pk
              j2=k-1;
          else
              j2=p-r;
          end        
          for j=j1:j2
              a(s2+1,j+1) = (a(s1+1,j+1)-a(s1+1,j))/ndu(pk+2,rk+j+1); % Index!!
              d = d + a(s2+1,j+1)*ndu(rk+j+1,pk+1); % Index!!
          end
          if r<=pk
              a(s2+1,k+1) = -a(s1+1,k)/ndu(pk+2,r+1); % Index!!
              d = d + a(s2+1,k+1)*ndu(r+1,pk+1); % Index!!
          end
          ders(k+1,r+1) = d;
          j = s1; s1=s2; s2=j; % Switch rows
      end
  end
  % Multiply through by the correct factors (Eq 2.9)
  r=p;
  for k=1:n
      for j=0:p
          ders(k+1,j+1) = r*ders(k+1,j+1);
      end
      r = r*(p-k);
  end
end
