# About the Project
This is a python package built to analyze suspension kinematics. It is designed to be modular to allow the user to configure any possible suspension geometry (five link, double A arm, Macpherson Strut, etc.). The current goal is to create an extensive library of suspension components that can be configured to model the kinematics of various suspensions, then create jupyter notebooks to allow users to easily design and analyze the most common suspension designs. 


# Motivation
There are various existing tools for analyzing vehicle suspension, however these tools are either behind a paywall or limited to specific suspension designs. I want to create a FOSS suspension analysis tool that is powerful enough to be used in grassroots motorsports, fleible enough to be used with various different designs, and easy enough to use for a gear head with a bit of programming knowledge.

# Project Status
<ins>Recent Progress</ins>

-A Jupyter Notebook has been written to perform kinematic analysis on a 5-link suspension 
-3d animations of the suspension movement have been added using vpython

<ins>Todo List</ins>
- [x] Fix indexing in kinematic_model class so that it can work with various types of links
- [ ] Use least squares method to improve inital guesses for numerical analysis
- [ ] Improve test coverage of the suspension componenets
- [ ] Refactor matrix generation to reduce copying of arrays
- [ ] Create auxiliary pickups for push rods, sway bars, and coilovers
- [ ] Figure out why the jacobian test is failing
- [ ] Add ability to import and save as csv file
- [ ] Add methods for A-arm and H-arm suspension components
- [ ] Add curve fitting algorithms


