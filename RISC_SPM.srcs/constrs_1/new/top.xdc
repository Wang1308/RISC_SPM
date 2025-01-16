



create_clock -period 10.000 -name clk_top -waveform {0.000 5.000} -add [get_ports clk_P]








group_path -name {clk_P} -from [get_ports clk_P] -through [get_nets {address[0]}] -to [get_ports {data_out[0]}]
