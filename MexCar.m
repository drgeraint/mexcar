function retval = MexCar(subfunc, varargin)
  try
    s = ["OctCar_", subfunc];
    f = str2func(s);
    retval = f(all_va_args);
  catch
    subfunc, varargin, s, f
    error('something is broken')
  end_try_catch
endfunction

    