function [omega, amp_einseitig_db] = amp_fft_db(sig_in, T_a)
%amp_fft_db: gibt amplitudenfrequenzgang in db zurück
%
% Argumente: 
% sig_in: Muss geradzahliger Länge sein!
% T_a : Abtastzeit
%
% Output
% omega
% Einseitiger Amplitudenfrequenzgang ohne DC
N = length(sig_in);
omega = 2*pi*(1:(N/2-1))/(N*T_a); % bei nyquist können wir nicht messen
Y = fft(sig_in); % scheinbar braucht die matlab fft keine vieflachen von 2
amp_beidseitig = abs(Y/N);
amp_einseitig = 2*amp_beidseitig(2:N/2);
amp_einseitig_db = 20*log10(amp_einseitig);

end