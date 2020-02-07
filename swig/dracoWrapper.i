/* File: dracoWrapper.i */

%module(directors = "1") DracoModule

%{
//"pass-through" include directives
#include "DracoWrapper.hpp"
%}

%include "DracoWrapper.hpp"