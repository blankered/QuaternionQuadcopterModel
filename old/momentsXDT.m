
clc; clear all;

syms Jx Jy Jz pddt qddt rddt pdt qdt rdt p q  r

J = [Jx 0 0; 0 Jy 0; 0 0 Jz]

xddt.omega = [pddt; qddt; rddt]
xdt.omega  = [pdt;  qdt;  rdt]
omega      = [p;    q;    r]

xddt.torque = J*xddt.omega + cross(xdt.omega,J*omega) +  cross(J*xdt.omega,omega)

xdt.m = xddt.torque(1)
xdt.n = xddt.torque(2)
xdt.l = xddt.torque(3)
