/* -*- c++ -*- */

#define MST_API

%include "gnuradio.i"			// the common stuff

//load generated python docstrings
%include "MST_swig_doc.i"

%{
#include "MST/route.h"
%}


%include "MST/route.h"
GR_SWIG_BLOCK_MAGIC2(MST, route);
