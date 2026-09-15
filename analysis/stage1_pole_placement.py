import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np
from scipy.signal import place_poles
from simulation.plant import linearised_matrices

A, B = linearised_matrices()

# Print open loop poles — natural behaviour of the system
eigenvalues = np.linalg.eigvals(A)
print("Open loop poles:")
print(eigenvalues)

# Place closed loop poles where we want them
desired_poles = np.array([-15, -18, -0.7, -0.9])
K = place_poles(A, B, desired_poles).gain_matrix
print("\nGain matrix K:")
print(K)
