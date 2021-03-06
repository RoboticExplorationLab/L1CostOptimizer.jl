# L1CostOptimizer
Fast Solution of Optimal Control Problems With L1 Cost

This repository contains the solver described in the paper: [Fast Solution of Optimal Control Problems With L1 Cost](https://rexlab.stanford.edu/papers/l1-cost-optimizer.pdf). It provides all the details regarding the parameters used for the experiments presented in the paper. Notebooks are also provided to allow for quick replication of the paper's results.

This solver is designed for optimal control problem with L1-norm control cost. It is written in Julia and relies on the general trajectory optimization solver [ALTRO (Augmented Lagrangian TRajectory Optimizer)](https://rexlab.stanford.edu/papers/altro-iros.pdf). 

## Replicating the results
The results presented in the paper can be easily replicated by running the dedicated notebooks. For instance, executing `L1CostOptimizer_nonlinear_dynamics_control_constraints.ipynb` in the `notebook` folder, will produce solutions for the spacecraft rendezvous problem with nonlinear dynamics and control constraints. This is an example of the solutions that you can obtain.

![alt text](https://raw.githubusercontent.com/RoboticExplorationLab/L1CostOptimizer.jl/master/readme/constrained_nonlinear_dynamics.png?token=AJJ5BNDZFOLKZSVBU4DROYS5KIIYI)

## 3D visualization of the spacecrafts trajectories
In order to get a better understanding of the state and control trajectories computed using the optimizer, we have made available a notebook that generates 3D visualizations. In the `notebook` folder, executing `trajectory_visualization.ipynb` will produce animations of the 2 statellites trajectories around Earth along with a visualization of the controls. This is a screenshot of the 3D visualization using MeshCat.

![alt text](https://raw.githubusercontent.com/RoboticExplorationLab/L1CostOptimizer.jl/master/readme/visualization_rendezvous.png?token=AJJ5BNHPFBKRNGXTJ7OUYL25KIISE)



