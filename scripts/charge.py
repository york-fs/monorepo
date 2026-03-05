#!/usr/bin/env python3

import numpy as np
import pybamm


# model = pybamm.lithium_ion.DFN(options={
#     'lithium plating': 'reversible',
#     'thermal': 'lumped',
#     'SEI': 'reaction limited',
# })
model = pybamm.lithium_ion.DFN()
params = pybamm.ParameterValues('OKane2022')
params.update({
    'Current function [A]': '[input]',
    'Nominal cell capacity [A.h]': 4.5,
    'Upper voltage cut-off [V]': 5.0,
})

# scale = 4.5 / 5.0
# param.update({
#     "Positive electrode thickness [m]":
#         param["Positive electrode thickness [m]"] * scale,
#     "Negative electrode thickness [m]":
#         param["Negative electrode thickness [m]"] * scale,
# })
# param.update({
#     "Negative electrode diffusivity [m2.s-1]":
#         param["Negative electrode diffusivity [m2.s-1]"] * 1.5,
#     "Positive electrode diffusivity [m2.s-1]":
#         param["Positive electrode diffusivity [m2.s-1]"] * 1.5,
# })
# param.update({
#     "Negative electrode reaction rate constant [m.s-1]":
#         param["Negative electrode reaction rate constant [m.s-1]"] * 1.3,
#     "Positive electrode reaction rate constant [m.s-1]":
#         param["Positive electrode reaction rate constant [m.s-1]"] * 1.3,
# })
# param.update({
#     "Electrolyte conductivity [S.m-1]":
#         param["Electrolyte conductivity [S.m-1]"] * 1.2,
# })
# param.update({
#     "Contact resistance [Ohm]": 0.005,
# })
# param.update({
#     "Total heat transfer coefficient [W.m-2.K-1]": 10,
# })

simulation = pybamm.Simulation(model, parameter_values=params)
simulation.set_initial_state(0.95)

time = 0
dt = 1
while time < 3600 or True:
    solution = simulation.step(dt, inputs={'Current function [A]': -2})
    voltage = solution['Voltage [V]'].entries[-1]
    soc = 1 - solution['Discharge capacity [A.h]'].entries[-1] / 5
    temperature = np.max(solution['Cell temperature [K]'].entries[-1])
    print(f'{time}: {voltage} {soc * 100}% {temperature:2f}')
    time += dt

# simulation.solve([0, 3600])
# simulation.plot_voltage_components()
# simulation.plot(['Current [A]', 'Voltage [V]', 'Cell temperature [K]'])
