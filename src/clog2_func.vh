// =============================================================================
// FILE: clog2_func.vh  (include fragment, NOT a standalone module)
//
// Single shared implementation of the compile-time bit-width calculator.
// Include this INSIDE a module body. Returns the number of bits needed to
// represent the value 'depth' (f_clog2(0) = 0, f_clog2(1) = 1, f_clog2(99999) = 17).
// Referenced by: system_clk_rst.v, debounce_filter.v
// All comments in pure ASCII English.
// =============================================================================

    function integer f_clog2;
        input integer depth;
        integer temp;
        begin
            temp = depth;
            for (f_clog2 = 0; temp > 0; f_clog2 = f_clog2 + 1)
                temp = temp >> 1;
        end
    endfunction
