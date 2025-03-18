# About the Project
This is a python package built to analyze suspension kinematics. It is designed to be modular with the goal to allow the user to configure any possible suspension geometry (five link, double A arm, Macpherson Strut, etc.). The current plan is to create an extensive library of suspension components that can be configured to model the kinematics of various suspensions, then create jupyter notebooks to allow users to easily design and analyze the most common suspension designs. 


# Motivation
There are various existing tools for analyzing vehicle suspension, however these tools are either behind a paywall or limited to specific suspension designs. I want to create an accessible suspension analysis tool that is easy to use and capable of producing accurate results.

# Project Status
<ins>Recent Progress</ins>

-The existing code has been majorly overhauled to add mypy type checking, separate visualizations into its own module, and generally clean up a lot of ugly code

<ins>Todo List</ins>
- [x] Fix indexing in kinematic_model class so that it can work with various types of links
- [x] Use least squares method to improve initial guesses for numerical analysis
- [x] Add type annotations to existing code
- [ ] Improve test coverage of the suspension components
- [ ] Refactor matrix generation to reduce copying of arrays (Possibly using array slices)
- [ ] Create auxiliary pickups for push rods, sway bars, coilovers, etc.
- [ ] Add ability to import and save suspension geometries and simulation results in a file (.csv or .mat files might be good for this)
- [ ] Add methods for A-arm and H-arm suspension components
- [ ] Add curve fitting algorithms to optimize suspension geometry




