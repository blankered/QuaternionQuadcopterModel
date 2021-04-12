clc; clear all;

syms qw1 qx1 qy1 qz1 qw2 qx2 qy2 qz2 p q r

omega.body = [p q r]
Quat.full = [qw1 qx1 qy1 qz1]
xdot.Quat = [qw2 qx2 qy2 qz2] 

eq1 = quatdivide(2*xdot.Quat,Quat.full)

