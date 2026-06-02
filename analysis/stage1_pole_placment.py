import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np
from control import place
from simulation.plant import linearised_matrices

A, B = linearised_matrices()

# Print open loop poles — natural behaviour of the system
eigenvalues = np.linalg.eigvals(A)
print("Open loop poles:")
print(eigenvalues)

# Place closed loop poles where we want them
desired_poles = [-3, -4, -5, -6]
K = place(A, B, desired_poles)
print("\nGain matrix K:")
print(K)