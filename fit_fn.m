function P = fit_fn(X,Xt,uc)

pnr=Xt.data(:,1);
per=Xt.data(:,2);
pdr=Xt.data(:,3);

T=Xt.time;

pn=X.data(:,1);
pe=X.data(:,2);
pd=X.data(:,3);

phi_c=uc.data(:,2);
theta_c=uc.data(:,3);
r_c=uc.data(:,4);

n = size(T,1);
s = (10/T(end))^3;
% fprintf('T = %0.2f\n',T(end));

t = min(T):0.01:max(T);
ph = interp1(T,phi_c,t);
th = interp1(T,theta_c,t);
r = interp1(T,r_c,t);

Ts = t(2)-t(1);
Fs = 1/Ts;
f = Fs*(0:length(t)/2)/length(t);

Yth = fft(th);
Yph = fft(ph);
Yr = fft(r);

P2th = abs(Yth/length(Yth));
P1th = P2th(1:length(P2th)/2 + 1);
[~,locsth] = findpeaks(P1th,f,'MinPeakheight',10e-4);

P2ph = abs(Yph/length(Yph));
P1ph = P2ph(1:length(P2ph)/2 + 1);
[~,locsph] = findpeaks(P1ph,f,'MinPeakheight',10e-4);

P2r = abs(Yr/length(Yr));
P1r = P2r(1:length(P2r)/2 + 1);
[~,locsr] = findpeaks(P1r,f,'MinPeakheight',10e-4);

max_freq = max([locsth,locsph,locsr]);
if isempty(max_freq)
    max_freq = 1;
end

P = sqrt(((pn-pnr)'*(pn-pnr)+(pe-per)'*(pe-per)+(pd-pdr)'*(pd-pdr))/n)*s*max_freq;
% fprintf('P = %0.5f\n',P);


end
