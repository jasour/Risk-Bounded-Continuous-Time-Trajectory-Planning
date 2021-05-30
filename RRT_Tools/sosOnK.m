% [ prgout, tk ] = sosOnK( prg, p, x, h, d );
%
% prg -- spotsosprg.
% p   -- 1-by-1 msspoly in x.
% x   -- n-by-1 free msspoly
% h   -- m-by-1 msspoly in x. (semialgebraic constraints defining K)
% d   -- scalar integer, d > deg(g).
%
% prgout --  new program with constraints
%       s(i) SOS, p - s'*h SOS
%       s(i) of maximal degree s.t. deg(s'*h) <= d.
% tk -- token associated with p - s'*h SOS.
%
function [ prg, tk ] = sosOnK( prg, p, x, h, d )
    m = size( h, 1 ); % total number of h's
    S = msspoly( zeros( m, 1 ) ); % create this many multiplier variables
    bases = monomials( x, 0:d );
    for i = 1:m
        [ prg, S( i ) ] = prg.newFreePoly( bases ); % make a polynomial whose coefficients are free variables
        prg = prg.withSOS( S( i ) ); % ensure that the polynomial is SOS
    end
    [ prg, tk ] = prg.withSOS( p - S'*h ); % ensure that p is SOS on K 
end