

clc; clear all;
syms Force(t) q0(t) q1(t) q2(t) q3(t) qe0(t) qe1(t) qe2(t) qe3(t) t dF dq0 dq1 dq2 dq3 dqe0 dqe1 dqe2 dqe3 F m

Params.m = m 

Params.dt = 0.1;
Params.E3 = [0 0 1]';
Params.Skew = @(x)[0 -x(3) x(2); x(3) 0 -x(1); -x(2) x(1) 0]
Params.Quat2Rot = @(q)eye(3) - 2*q(1)*Params.Skew(q(2:4)) + 2*Params.Skew(q(2:4))^2
Params.Iota = @(q)[Params.Skew(q(2:4)) + q(1)*eye(3); -q(2:4)]
Params.xdt.Quat2Rot = @(qe,dqe)(-2*dqe(1)*Params.Skew(qe(2:4)) -2*qe(1)*Params.Skew(dqe(2:4)) + 2*(Params.Skew(qe(2:4))*Params.Skew(dqe(2:4)) + Params.Skew(dqe(2:4))*Params.Skew(qe(2:4))))


qinert = [q0(t) q1(t) q2(t) q3(t)]
qerror = [qe0(t) qe1(t) qe2(t) qe3(t)]

% v = transpose(q(2:4))
% v*transpose(v) - norm(v)^2*eye(3)

funcW = (Force/Params.m)*(qerror(1)*eye(3) - Params.Skew(qerror(2:4)))*Params.Skew(Params.Quat2Rot(qinert)*Params.E3)
dw = diff(funcW,t)

syms dF dq_0 dq_1 dq_2 dq_3 dqe_0 dqe_1 dqe_2 dqe_3
l1 = [diff(Force(t), t), diff(q0(t), t), diff(q1(t), t), diff(q2(t), t), diff(q3(t), t), diff(qe0(t), t), diff(qe1(t), t), diff(qe2(t), t), diff(qe3(t), t)]
l2 = [dF, dq_0, dq_1, dq_2, dq_3, dqe_0, dqe_1, dqe_2, dqe_3] 
dw1 = subs(dw,l1,l2)

%clear Force(t) q0(t) q1(t) q2(t) q3(t) qe0(t) qe1(t) qe2(t) qe3(t)
syms  F q_0 q_1 q_2 q_3 qe_0 qe_1 qe_2 qe_3

l3 = [Force(t), q0(t), q1(t), q2(t), q3(t), qe0(t), qe1(t), qe2(t), qe3(t)]
l4 = [F q_0 q_1 q_2 q_3 qe_0 qe_1 qe_2 qe_3]

dw2 = subs(dw1,l3,l4)

%latex1 = latex(dw2)
%replace(latex1,{'dq', 'dF', 'qe'},{'\dot{q}', '\dot{F}', '\tilde{q}'})

%%
clc; clear all;

% syms F m ve1 ve2 ve3 ve4 dve1 dve2 dve3 qew qex qey qez dve4 qw qx qy qz dqeW dqeX dqeY dqeZ
% verror = [ve1 ve2 ve3 ve4]
% dverror = [dve1 dve2 dve3 dve4]
% qinert = [qw qx qy qz]
% qe = [qew qex qey qez]
% dqe = [dqeW dqeX dqeY dqeZ]

syms Force(t) m t  ve1(t) ve2(t) ve3(t) ve4(t) q0(t) q1(t) q2(t) q3(t) qe0(t) qe1(t) qe2(t) qe3(t) p(t) q(t) r(t) K3 H 
verror = [ve1(t); ve2(t); ve3(t)]
qinert = [q0(t) q1(t) q2(t) q3(t)]
qe = [qe0(t) qe1(t) qe2(t) qe3(t)]
omega.body = [p(t); q(t); r(t)]

Params.m = m 

Params.dt = 0.1;
Params.E3 = [0 0 1]';
Params.Skew = @(x)[0 -x(3) x(2); x(3) 0 -x(1); -x(2) x(1) 0]
Params.Quat2Rot = @(q)eye(3) - 2*q(1)*Params.Skew(q(2:4)) + 2*Params.Skew(q(2:4))^2
Params.Iota = @(q)[-q(2:4); Params.Skew(q(2:4)) + q(1)*eye(3)]
%Params.xdt.Quat2Rot = @(qe,dqe)(-2*dqe(1)*Params.Skew(qe(2:4)) -2*qe(1)*Params.Skew(dqe(2:4)) + 2*(Params.Skew(qe(2:4))*Params.Skew(dqe(2:4)) + Params.Skew(dqe(2:4))*Params.Skew(qe(2:4))))

funcW = (Force/m)*(qe(1)*eye(3) - Params.Skew(qe(2:4)))*Params.Skew(Params.Quat2Rot(qinert)*Params.E3)
%Params.xdt.WT = @(t,F,dF,dq_0,dq_1,dq_2,dq_3,dqe_0,dqe_1,dqe_2,dqe_3,m,q_0,q_1,q_2,q_3,qe_0,qe_1,qe_2,qe_3)reshape([(F.*qe_2.*(dq_0.*q_1.*2.0+dq_1.*q_0.*2.0+dq_2.*q_3.*2.0+dq_3.*q_2.*2.0))./m-(F.*dqe_3.*(q_1.^2.*2.0+q_2.^2.*2.0-1.0))./m-(F.*qe_3.*(dq_1.*q_1.*4.0+dq_2.*q_2.*4.0))./m+(F.*dqe_2.*(q_0.*q_1.*2.0+q_2.*q_3.*2.0))./m-(dF.*qe_3.*(q_1.^2.*2.0+q_2.^2.*2.0-1.0))./m+(dF.*qe_2.*(q_0.*q_1.*2.0+q_2.*q_3.*2.0))./m,-(F.*qe_1.*(dq_0.*q_1.*2.0+dq_1.*q_0.*2.0+dq_2.*q_3.*2.0+dq_3.*q_2.*2.0))./m-(F.*dqe_0.*(q_1.^2.*2.0+q_2.^2.*2.0-1.0))./m-(F.*qe_0.*(dq_1.*q_1.*4.0+dq_2.*q_2.*4.0))./m-(F.*dqe_1.*(q_0.*q_1.*2.0+q_2.*q_3.*2.0))./m-(dF.*qe_0.*(q_1.^2.*2.0+q_2.^2.*2.0-1.0))./m-(dF.*qe_1.*(q_0.*q_1.*2.0+q_2.*q_3.*2.0))./m,-(F.*qe_0.*(dq_0.*q_1.*2.0+dq_1.*q_0.*2.0+dq_2.*q_3.*2.0+dq_3.*q_2.*2.0))./m+(F.*dqe_1.*(q_1.^2.*2.0+q_2.^2.*2.0-1.0))./m+(F.*qe_1.*(dq_1.*q_1.*4.0+dq_2.*q_2.*4.0))./m-(F.*dqe_0.*(q_0.*q_1.*2.0+q_2.*q_3.*2.0))./m+(dF.*qe_1.*(q_1.^2.*2.0+q_2.^2.*2.0-1.0))./m-(dF.*qe_0.*(q_0.*q_1.*2.0+q_2.*q_3.*2.0))./m,(F.*qe_2.*(dq_0.*q_2.*2.0+dq_2.*q_0.*2.0-dq_1.*q_3.*2.0-dq_3.*q_1.*2.0))./m+(F.*dqe_0.*(q_1.^2.*2.0+q_2.^2.*2.0-1.0))./m+(F.*qe_0.*(dq_1.*q_1.*4.0+dq_2.*q_2.*4.0))./m+(F.*dqe_2.*(q_0.*q_2.*2.0-q_1.*q_3.*2.0))./m+(dF.*qe_0.*(q_1.^2.*2.0+q_2.^2.*2.0-1.0))./m+(dF.*qe_2.*(q_0.*q_2.*2.0-q_1.*q_3.*2.0))./m,-(F.*qe_1.*(dq_0.*q_2.*2.0+dq_2.*q_0.*2.0-dq_1.*q_3.*2.0-dq_3.*q_1.*2.0))./m-(F.*dqe_3.*(q_1.^2.*2.0+q_2.^2.*2.0-1.0))./m-(F.*qe_3.*(dq_1.*q_1.*4.0+dq_2.*q_2.*4.0))./m-(F.*dqe_1.*(q_0.*q_2.*2.0-q_1.*q_3.*2.0))./m-(dF.*qe_3.*(q_1.^2.*2.0+q_2.^2.*2.0-1.0))./m-(dF.*qe_1.*(q_0.*q_2.*2.0-q_1.*q_3.*2.0))./m,-(F.*qe_0.*(dq_0.*q_2.*2.0+dq_2.*q_0.*2.0-dq_1.*q_3.*2.0-dq_3.*q_1.*2.0))./m+(F.*dqe_2.*(q_1.^2.*2.0+q_2.^2.*2.0-1.0))./m+(F.*qe_2.*(dq_1.*q_1.*4.0+dq_2.*q_2.*4.0))./m-(F.*dqe_0.*(q_0.*q_2.*2.0-q_1.*q_3.*2.0))./m+(dF.*qe_2.*(q_1.^2.*2.0+q_2.^2.*2.0-1.0))./m-(dF.*qe_0.*(q_0.*q_2.*2.0-q_1.*q_3.*2.0))./m,(F.*qe_0.*(dq_0.*q_1.*2.0+dq_1.*q_0.*2.0+dq_2.*q_3.*2.0+dq_3.*q_2.*2.0))./m+(F.*qe_3.*(dq_0.*q_2.*2.0+dq_2.*q_0.*2.0-dq_1.*q_3.*2.0-dq_3.*q_1.*2.0))./m+(F.*dqe_0.*(q_0.*q_1.*2.0+q_2.*q_3.*2.0))./m+(F.*dqe_3.*(q_0.*q_2.*2.0-q_1.*q_3.*2.0))./m+(dF.*qe_0.*(q_0.*q_1.*2.0+q_2.*q_3.*2.0))./m+(dF.*qe_3.*(q_0.*q_2.*2.0-q_1.*q_3.*2.0))./m,(F.*qe_0.*(dq_0.*q_2.*2.0+dq_2.*q_0.*2.0-dq_1.*q_3.*2.0-dq_3.*q_1.*2.0))./m-(F.*qe_3.*(dq_0.*q_1.*2.0+dq_1.*q_0.*2.0+dq_2.*q_3.*2.0+dq_3.*q_2.*2.0))./m+(F.*dqe_0.*(q_0.*q_2.*2.0-q_1.*q_3.*2.0))./m-(F.*dqe_3.*(q_0.*q_1.*2.0+q_2.*q_3.*2.0))./m+(dF.*qe_0.*(q_0.*q_2.*2.0-q_1.*q_3.*2.0))./m-(dF.*qe_3.*(q_0.*q_1.*2.0+q_2.*q_3.*2.0))./m,-(F.*qe_1.*(dq_0.*q_2.*2.0+dq_2.*q_0.*2.0-dq_1.*q_3.*2.0-dq_3.*q_1.*2.0))./m+(F.*qe_2.*(dq_0.*q_1.*2.0+dq_1.*q_0.*2.0+dq_2.*q_3.*2.0+dq_3.*q_2.*2.0))./m-(F.*dqe_1.*(q_0.*q_2.*2.0-q_1.*q_3.*2.0))./m+(F.*dqe_2.*(q_0.*q_1.*2.0+q_2.*q_3.*2.0))./m-(dF.*qe_1.*(q_0.*q_2.*2.0-q_1.*q_3.*2.0))./m+(dF.*qe_2.*(q_0.*q_1.*2.0+q_2.*q_3.*2.0))./m],[3,3])


omega.desired = K3 * H * transpose(qe(2:4)) - funcW * verror + Params.Quat2Rot(qe) * omega.body
dw = diff(omega.desired,t)

%dw = diff(funcW,t)

syms dF dq_0 dq_1 dq_2 dq_3 dqe_0 dqe_1 dqe_2 dqe_3
l1 = [diff(Force(t), t), diff(q0(t), t), diff(q1(t), t), diff(q2(t), t), diff(q3(t), t), diff(qe0(t), t), diff(qe1(t), t), diff(qe2(t), t), diff(qe3(t), t)]
l2 = [dF, dq_0, dq_1, dq_2, dq_3, dqe_0, dqe_1, dqe_2, dqe_3] 
dw1 = subs(dw(t),l1,l2)



%clear Force(t) q0(t) q1(t) q2(t) q3(t) qe0(t) qe1(t) qe2(t) qe3(t)
syms  F q_0 q_1 q_2 q_3 qe_0 qe_1 qe_2 qe_3 ve_1 ve_2 ve_3

l3 = [Force(t), q0(t), q1(t), q2(t), q3(t), qe0(t), qe1(t), qe2(t), qe3(t) ve1(t), ve2(t), ve3(t)]
l4 = [F q_0 q_1 q_2 q_3 qe_0 qe_1 qe_2 qe_3 ve_1 ve_2 ve_3]

dw2 = subs(dw1,l3,l4)

syms w_1 w_2 w_3 dw_1 dw_2 dw_3  dve_1  dve_2  dve_3

l5 = [diff(p(t), t) diff(q(t), t) diff(r(t), t) diff(ve1(t), t) diff(ve2(t), t) diff(ve3(t), t)]
l6 = [  dw_1          dw_2           dw_3          dve_1            dve_2            dve_3     ]
dw3 = subs(dw2,l5,l6)


l7 = [p(t) q(t) r(t)]
l8 = [w_1  w_2  w_3 ]
dw4 = subs(dw3,l7,l8)


% verror = [ve1; ve2(t); ve3(t)]
% dverror = [dve1; dve2; dve3]
% 
% qinert = [q0(t) q1(t) q2(t) q3(t)]
% qe = [qe0(t) qe1(t) qe2(t) qe3(t)]
% omega = [w_1 w_2 w_3]
% domega = [dw_1 dw_2 dw_3]

