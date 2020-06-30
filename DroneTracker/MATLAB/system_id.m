syms a b t s

A = [0 1;
     0 a];
 
B = [0;
     b];
 
 C = [1 0];
 
ExpAt_s = simplify(expm(A * (t-s)))

simplify(int(C * ExpAt_s*B, s, 0, t))

