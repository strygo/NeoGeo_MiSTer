# Worst setup paths after the fit (run by ngplus_fit.yml): the plain-text
# sta.rpt has no path detail, so name the endpoints here.
project_open NeoGeo
create_timing_netlist
read_sdc
update_timing_netlist
report_timing -setup -npaths 8 -detail full_path -file output_files/sta_paths.txt
report_timing -setup -npaths 4 -detail full_path -to_clock [get_clocks {emu|pll|pll_inst|altera_pll_i|cyclonev_pll|counter[0].output_counter|divclk}] -file output_files/sta_paths_96m.txt
project_close
