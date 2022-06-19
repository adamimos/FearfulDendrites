import sys
print(sys.argv[1])

from neuron import h
load_file("nrngui.hoc")
h('n_run = 2')

h('load_file("shai_load.hoc")')
h.tstop = 100
h.run()