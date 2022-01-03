# PDM-drone-control

PDM project: Determine a path for a drone while avoiding obstacles

# Installation

This application requires Python 3.7 or 3.8. Additionally, it requires IBM's CPLEX solver as a backend for cvxpy. This
solver can be downloaded for free for academic use as part of IBM ILOG CPLEX Optimization Studio
(https://www.ibm.com/products/ilog-cplex-optimization-studio). Simply create an account with your TU Delft email
address, and you will be able to download the academic version of CPLEX Optimization Studio. Once CPLEX Optimization
Studio is installed, install the Python API with
`python /my-install-dir/ibm/ILOG/CPLEX_Studio201/python/setup.py install` in your Python environment. Cvxpy will now be
able to the CPLEX solver.

In summary:

- create an IBM account with your TU Delft email address
- download and install the academic version of CPLEX Optimization Studio
- OPTIONAL: in your Python environment, uninstall the community edition of CPLEX with `pip uninstall cplex docplex`
- in your Python environment, install the Python API
  with `python /my-install-dir/ibm/ILOG/CPLEX_Studio201/python/setup.py install`
