
clc; 
clear all;
close all;

%% Definizione matrici del problema di regolazione

g = 9.81;

A = [
    0 1 0 0; 
    0 -1 0 0; 
    0 0 0 1; 
    0 1 g 0
    ];

B = [0; 1; 0; -1];

PH = [
    0 0 0; 
    1 0 0; 
    0 0 0; 
    -1 0 0
    ];

C = [1 0 0 0; 0 0 1 0];

Cs = [1 0 0 0];

Q = [0 -1 0];

% Frequenza sinusoide
w = 0.1;

S = [0, 0, 0;
     0, 0, w;
     0,-w, 0];

%% Verifica che Assunzione 1, e che S genera segnali giusti

% Calcolo degli autovalori e autovettori della matrice S
autoVal = eig(S);

% Le condizioni iniziali scelte per generare d1(t)=d1, d2(t)=α*sin(w*t)
alpha = 1;  % ampiezza del segnale sinusoidale
d10 = 1;    % valore costante del disturbo
d20 = 0;    % d2(0)=0 per avere sin(w*t) puro
d30 = alpha; 

d0 = [d10; d20; d30];  

% Definisco e risolvo ode
t_span = 0:0.01:100;
odefun = @(t,d) S*d;
[t_num, d_num] = ode45(odefun, t_span, d0);

figure()
plot(t_num, d_num(:,1), 'b-', 'LineWidth', 2);
hold on;
plot(t_num, d_num(:,2), 'r-', 'LineWidth', 2);
xlabel('t');
ylabel('f(t)');
title('Primi due segnali generati da S');
legend('d1(t)', 'd2(t)');
hold off;

%% Verifica assunzione 2 (coppia (A,B) raggiungibile)

% Verifica la raggiungibilità della coppia (A,B)
rankAB = rank([B A*B A^2*B A^3*B]);
disp("La matrice ha rango pieno quindi sistema controllabile/raggiungibile");

% Metodo automatico
Co = ctrb(A, B);
rank_Co = rank(Co);

%% FULL INFORMATION...

%% Verifica Lemma di Hautus

condizione_soddisfatta = true;
for k = 1:length(autoVal)
    s = autoVal(k);
    
    % Matrice di Hautus per il problema di regolazione
    % rank([sI-A, B; Cs, 0]) = n + p per tutti gli s appartenenti allo spettro di S
    n = size(A,1);  % n = 4
    p = size(Cs,1); % p = 1
    
    M = [s*eye(n)-A, B; Cs, zeros(p, size(B,2))];
    rango = rank(M);
    
    fprintf('s = %.4f%+.4fi: rank = %d (richiesto: %d)', ...
            real(s), imag(s), rango, n+p);
    
    if rango == n+p
        fprintf('Condizione soddisfatta\n');
    else
        fprintf('Condizione non soddisfatta\n');
        condizione_soddisfatta = false;
    end
end

%% Trovo K (quando d=0)

% Vettore poli desiderati
v = [-1.5,-1,-1.2,-0.8];

% Poli desiderati per A+BK (distinti per evitare errore place)
K = place(A,-B,v);


% Verifica della stabilità
A_cl = A + B*K;
eig_cl = eig(A_cl);
if all(real(eig_cl) < 0)
    fprintf('Sistema a ciclo chiuso stabilizzato\n');
else
    fprintf('Sistema NON stabilizzato\n');
end


%% Soluzione delle FBI equations
% Funzione per risoluzione equazioni di sylvester generalizzate (fbi equations)
function [X] = generalized_sylvester(A,B,C,D,E)
% GENERALIZED_SYLVESTER X = generalized_sylvester(A,B,C,D,E) returns the
% solution X to the matrix equation AXB + CXD = E, if it exists
% 
%   Returns the solution X to the matrix equation AXB + CXD = E where A and
%       C are square matrices of dimension n times n, B and D are square
%       matrices of dimension m times m, and E is a matrix of dimension
%       n times m, so long as such a solution exists and is unique.
% 
%   INPUTS
%   A,C: square matrices of dimension n times n, such that (A,C) form a
%       regular matrix pencil;
%   B,D: square matrices of dimension m times m, such that (B,D) form a
%       regular matrix pencil, and the generalized eigenvalues of (A,C) and
%       (-B,D) are disjoint;
%   E: a matrix of dimension n times m;
% 
%   OUTPUTS
%   X: a matrix of dimension n times m satisfying AXB + CXD = E;
% 
% 
%     Created by Joel D. Simard
% 
%     Get dimensions of the matrices A and B. These will be used to validate consistency of all inputs A, B, C, D, and E.
    n = length(A);
    m = length(B);

    % All inputs must be numeric
    % A and C must be square matrices of the same size 
    % B and D must be square matrices of the same size
    % E must be a matrix with number of rows being the same as A and C,
    % and number of columns being the same as B and D
    required_classes = {'numeric'};
    required_attributes_A_C = {'nonempty','finite','nonnan','square','size',[n,n]};
    required_attributes_B_D = {'nonempty','finite','nonnan','square','size',[m,m]};
    required_attributes_E = {'nonempty','finite','nonnan','size',[n,m]};
    
    % Validate the properties described above
    validateattributes(A,required_classes,required_attributes_A_C,'','A',1);
    validateattributes(B,required_classes,required_attributes_B_D,'','B',2);
    validateattributes(C,required_classes,required_attributes_A_C,'','C',3);
    validateattributes(D,required_classes,required_attributes_B_D,'','D',4);
    validateattributes(E,required_classes,required_attributes_E,'','E',5);

    % NOTE ON OBTAINING THE SOLUTION: Here I use a vectorization approach to easily determine the solution. However, this approach requires inverting a nm times nm matrix, which could be problematic for large problems. To avoid this, a generalized eigenvalue/eigenvector approach should be implemented in the future.
    % To do this, add a check for regularity and disjoint generalized spectra of the matrix penxils (A + x C), (D - x B), then apply generalized eigenvectors to calculate X element by element.

    % Calculate the vectorization of the solution X by using the kronecker product (for any matrices A, B, C, vec(A*B*C) = kron(transpose(C),A)*vec(B))
    kronecker_vectorization_factor = kron(transpose(B),A) + kron(transpose(D),C);
    
    % Before trying to calculate the inverse, check that it is invertible. If it is not invertible, then there does not exist a unique solution to the generalized Sylvester equation (there is either no solution, or there is an infinite set solutions)
    validateattributes(det(kronecker_vectorization_factor),required_classes,{'nonempty','finite','nonnan','nonzero'},'','the determinant of the Kronecker vectorization factor (required to be invertible for the existence of a unique solution to the generalized Sylvester equation)');
    vecX = kronecker_vectorization_factor\E(:);
    
    % Check that vec(X) is acceptable before reshaping (numeric, nonempty, finite, nonnan, and has size [n*m,1])
    required_attributes_vecX = {'nonempty','finite','nonnan','size',[n*m,1]};
    validateattributes(vecX,required_classes,required_attributes_vecX,'','the solution to the generalized Sylvester equation, X,');
    
    %Reshape vec(X) into the n times m matrix X
    X = reshape(vecX,n,m);
end

%AXB + CXD = E
A_ext = [A, B; Cs, zeros(size(Cs,1),size(B,2))];
B_ext = eye(size(S));
C_ext = -[eye(size(A,1)) zeros(size(A,1), size(B,2)); zeros(size(Cs,1),size(A,1)+size(B,2))];
D_ext = S;
E_ext = -[PH; Q];

X = generalized_sylvester(A_ext, B_ext, C_ext, D_ext, E_ext);

% Estraggo Pi e Gamma
Pi = X(1:size(A,1), :);
Gamma = X(size(A,1)+1:end, :);



% Verifica equazioni FBI
err1 = A*Pi + B*Gamma + PH - Pi*S;
err2 = Cs*Pi + Q;

%% Creazione controllore

% So che la formula è u=K*x+L*d, dove K è quella calcolata prima e 
% L = Gamma - K * Pi

L = Gamma - K * Pi;
%L = [-1.0000   -0.1391   -0.0545]; %QUELLA GIUSTA per w=0.1!
%L = [-1.0000    0.4644   -0.0833]; %QUELLA GIUSTA per w=1!


