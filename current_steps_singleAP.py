﻿#!/usr/bin/env python
# -*- coding: utf-8 -*-

# This code opens neuron and runs a single simulation and modified the number of
# CS+ tuft synaptic inputs

import sys
import neuron
import numpy as np
from neuron import h, gui
import matplotlib.pyplot as plt
from tqdm import tqdm
import pickle

#PARAMETERS
iStep = float(sys.argv[1])/100.0 # divide input by 100 because input can only be an int
batch_name = "current_steps_singleAP"+str(float(sys.argv[1])+90000)+".p"

print('running with ', iStep, ' nA')

h('load_file("nrngui.hoc")')

h('''
load_file("import3d.hoc")
objref L5PC

strdef morphology_file
morphology_file = "./Lucy23Cell1.asc"

load_file("./L5PCbiophys3.hoc")
load_file("./L5PCtemplate.hoc")
L5PC = new L5PCtemplate(morphology_file)
''')

h('''
xopen("DefineSections.hoc")

xopen("ChangeBiophys.hoc")

xopen("AccessoryFunctions.hoc")

xopen("distSynsUniform.hoc")
xopen("distSynsCluster2.hoc")
''')

h.nrncontrolmenu()

for i in h.L5PCtemplate[0].soma:
    soma = i


def distSynsUniform(numSyns, sectionList, gmax=0, ntar=1.0):
    # pythonify the sectionList
    sectionList = [sec for sec in sectionList]

    # get list of lengths and total length
    lengthList, totalL = [], 0
    for i in sectionList:
        lengthList.append(i.L)
        totalL += i.L

    # get cumulative length list
    cumLList = np.cumsum(lengthList)

    # make a uniform vector of random numbers across the lengths
    randomVec = np.random.uniform(low=0,high=totalL,size = numSyns)
    randomVec = np.sort(randomVec)

    synList = []
    for r in randomVec:
        # find which place to put it on!
        secInd = np.where(cumLList>r)[0][0]

        # find where to put it
        location = (cumLList[secInd] - r)/lengthList[secInd]
        syn = h.glutamate1(sectionList[secInd](location))
        syn.gmax = gmax
        syn.ntar = ntar
        synList.append(syn)

    return synList

def zero_all_gmax(secList):
    for syn in secList:
        syn.gmax = 0.0
        syn.ntar = 1.0

def turn_on_background(background_list, t0 = 2000):
    print('the currently accessed section is',  h.cas())
    h.distance(0)
    for syn in background_list:
            where = syn.get_segment() # location of synapse
            dist = h.distance(where) # distance from soma
            syn.gmax = 0.2683 + 0.0625*np.exp(dist/158.0158)
            syn.ntar = 1.0
            syn.del2 = t0 + np.random.uniform(-500,150)
            syn.Tspike = np.random.uniform(20,100)
            syn.Nspike = 200

def turn_on_syns(synapse_list, t0 = 2000):
    for syn in synapse_list:
            syn.gmax = 1
            syn.ntar = 1.0
            syn.del2 = t0 + np.random.uniform(1000,1500)
            syn.Nspike = 1
            syn.Tspike = 50

def apply_APV(synList, secList):
    secList = [seg for seg in secList]

    for syn in syns_background[0]:
        syn.get_loc()
        if h.cas() in secList:
            # take away NMDA with ampa current = gmax/ntar
            syn.ntar = 0.0001
            syn.gmax = syn.gmax*syn.ntar
        h.pop_section()

# create IClamp
stimobj = h.IClamp(0.5, sec=soma)


v_data, t_data = [], []
d_v_data, b_v_data = [], []


cvode_stop = 1000 # ms to initialize with cvode

#ps = h.Shape()
#[ps.point_mark(syn,4) for syn in syns_background[repeat_i]]
#[ps.point_mark(syn,2) for syn in syns_CSplus_tuft[repeat_i]]
#[ps.point_mark(syn,3) for syn in syns_CSplus_basal[repeat_i]]
# 500 of initialization, then 500 of rest, then 500 of stim, then 500 of rest = 2000 total
# with 1000-1500 stim

d1, d2 = h.L5PC.apic[22](0.5), h.L5PC.apic[33](0.8)
d3, d4 = h.L5PC.apic[38](0.8), h.L5PC.apic[51](0.8)
d5, d6 = h.L5PC.apic[26](0.96), h.L5PC.apic[56](0.75)

b1, b2 = h.L5PC.dend[38](0.45), h.L5PC.dend[45](0.54)
b3, b4 = h.L5PC.dend[65](0.64), h.L5PC.dend[7](0.62)

# ---------- CVODE RUN
h.finitialize(-89.5)
print(h.t,soma.v)

# ---------- SIMULATION RUN
time = np.arange(0,cvode_stop + 1200,.05)
h.dt = 0.05

stimobj.dur = 2 # 2 seconds
stimobj.amp = iStep
stimobj.delay = 1100

v, t_v  = [], []
d_v, b_v = [], []
for t in tqdm(time):
    h.fadvance()
    v.append(soma.v)
    t_v.append(h.t)
    d_v.append([d1.v, d2.v, d3.v, d4.v, d5.v, d6.v])
    b_v.append([b1.v, b2.v, b3.v, b4.v])

v_data = v
t_data = t_v
d_v_data = d_v
b_v_data = b_v


from scipy.signal import find_peaks
print('the number of spikes is',len(find_peaks(v,height=0)[0]))


output = {'time':t_data, 'soma_v':v_data, 'apic_v':d_v_data, 'basal_v':b_v_data,
         'num_spikes':len(find_peaks(v,height=0)[0])}

try:
    batch_results = pickle.load(open(batch_name,"rb"))
    print('loaded previous file with ', len(batch_results), ' entries.')
except:
    print('creating new output file!')
    batch_results = []
batch_results.append(output)
pickle.dump(batch_results, open(batch_name, "wb" ) )
h.quit()
