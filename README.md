# L1CostOptimizer
Fast Solution of Optimal Control Problems With L1 Cost

This repository contains the solver described in the paper: ###LINK TO PAPER. Its provides all the details regarding the parameters used for the experiments presented in the paper. Notebooks are also provided to allow for quick replication of the paper's results.

This solver is designed for optimal control problem with L1-norm control cost. It is written in Julia and relies on the general trajectory optimization solver [ALTRO (Augmented Lagrangian TRajectory Optimizer)](https://rexlab.stanford.edu/papers/altro-iros.pdf). 

## Replicating the results
The results presented in the paper can be easily replicated by running the dedicated notebooks. For instance, executing `L1CostOptimizer_nonlinear_dynamics_control_constraints.ipynb` will produce solutions for the spacecraft rendezvous problem with nonlinear dynamics and control constraints.

![alt text](https://raw.githubusercontent.com/RoboticExplorationLab/L1CostOptimizer.jl/master/readme/constrained_nonlinear_dynamics.png?token=AJJ5BNDZFOLKZSVBU4DROYS5KIIYI)

## 3D visualization of the spacecrafts trajectories
The results presented in the paper can be easily replicated by running the dedicated notebooks. For instance, executing `L1CostOptimizer_nonlinear_dynamics_control_constraints.ipynb` will produce solutions for the spacecraft rendezvous problem with nonlinear dynamics and control constraints.

![alt text](https://raw.githubusercontent.com/RoboticExplorationLab/L1CostOptimizer.jl/master/readme/visualization_rendezvous.png?token=AJJ5BNHPFBKRNGXTJ7OUYL25KIISE)



