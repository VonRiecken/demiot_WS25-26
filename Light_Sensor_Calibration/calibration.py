#  %% imports
import pandas as pd
import os
import numpy as np
from scipy.stats import linregress

#  %% import data
df = pd.read_csv('calibration_data.csv', delimiter='\t')


#  %% formulae
'''
R = AE^-gamma
    R = resistance
    E = Illuminance (Lux)
    A = Constant (depends on LDR type)

R_LDR = R_fixed x (Vin/Vout - 1)
'''

# %% get LDR resistance
r_fixed = 9900
lux = df['LUX (luxmeter)']
v_in = 3.3
df['LDR_Resistance'] = r_fixed * (v_in/df['Volts']-1)

# %% get Ln
df['lnR'] = np.log(df['LDR_Resistance'])
df['lnlux'] = np.log(df['LUX (luxmeter)'])

# %% get regression
slope, intercept, r_value, p_value, std_err = linregress(df['lnlux'], df['lnR'])

gamma = -slope
a = np.exp(intercept)

print("gamma =", gamma)
print("A =", a)

# %% validate
df['E_pred'] = (a/ df['LDR_Resistance']) ** (1 / gamma)
