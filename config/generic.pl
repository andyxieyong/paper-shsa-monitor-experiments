%
% SHSA Knowledge Base for Fault Injection Tests
%
% Denise Ratasich
% 2019-03-09
%

:- use_module(library(shsa)).

% example function
function(y, double, [x]).

% to create executable substitutions: define implementations of the relations
implementation(double, "
y.v = 2 * x.v
y.t = x.t
").

% hard-coded provided itoms
itomsOf(x, ["/a/data", "/b/data", "/c/data"]).
