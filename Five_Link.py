import numpy as np
import scipy as sp
import sympy 
import csv 



class Five_Link:

    def __init__(self):
    
        self.link_lengths = np.zeros(5)
        self.frame_pickups = np.array([5,3])
        self.upright_pickups = np.zeros([5,3])

    def __init__(self, link_lengths, frame_pickups, upright_pickups):
    
        self.link_lengths = link_lengths
        self.frame_pickups = frame_pickups
        self.upright_pickups = upright_pickups

        self.P = self.gen_P()
        self.A = self.gen_A()

    def __init__(self, file_name):
        
        self.link_lengths = np.zeros(5)
        self.frame_pickups = np.array([5,3])
        self.upright_pickups = np.zeros([5,3])


        with open(file_name, newline='') as csvfile:
            reader = csv.DictReader(csvfile, delimiter="", quotechar="|")
            #next(reader) #skips over the headders on the spreadsheet
            for i, row in enumerate(reader):
                self.link_lengths[i] = row["Length"]

                self.frame_pickups[i,0] = row["Frame_x"]
                self.frame_pickups[i,1] = row["Frame_y"]
                self.frame_pickups[i,2] = row["Frame_z"]

                self.upright_pickups[i,0] = row["Upright_x"]
                self.upright_pickups[i,1] = row["Upright_y"]
                self.upright_pickups[i,2] = row["Upright_z"]
   
    variable_index_lookup = {"W_x": 0,
                            "W_y": 1,
                            "W_z": 2 }
    
    def __gen_P_W__():
        #output = np.zeros((5,1), dtype=np.ndarray)

        output = [[np.identity(3)]*1]*5

        #for i, output_val in enumerate(output):
            #output[i, 0] = -1*np.identity(3)

        return np.block(output)

    def __gen_P_R__(upright_pickups):
        output = np.zeros((5,3), dtype=np.ndarray)
        #output = [[np.identity(3)] * 3] *5

        #each block is a pickup coordinate * the identity matrix
        for i, pickup_n in enumerate(upright_pickups) :
            for j, pickup_coord in enumerate(pickup_n):
                #print(output[i, j])
                output[i, j] = pickup_coord*np.identity(3)

        #combines the block matrix
        return np.block(output.tolist())

    def __gen_P_AB__(lengths):
        output = np.zeros(lengths.size * 3)
        #output = [] * (lengths.size*3)

        for i, length in enumerate(lengths):
            for j in range(3*i, 3*i+3):
                output[j] = -1*length
        # swap colums arorund
        return np.diag(output)

    def gen_P(self):
        return np.block([self.__gen_P_W__(), self.__gen_P_R__(self.upright_pickups), self.__gen_P_AB__(self.link_lengths)])

    def gen_A(self):    
        return np.atleast_2d(np.ravel(self.frame_pickups)).T

    def solve_linear(P, A):
        #driving_variable = variable_index_lookup[driving_variable]

        #saves then copies the colum of the driving variable from the equation matrix
        #driving_colum = np.array(P[:, [driving_variable]])
        #P = np.delete(P, driving_variable, 1)

        l, u = sp.linalg.lu(P, permute_l=True, overwrite_a=True)
        l = np.linalg.inv(l)

        A = np.dot(l, A)
        #driving_colum = np.dot(l, driving_colum)

        #print(l, u, A)
        return u, A

    def gen_AB_nonlinear_eq(vars):
        alpha, beta = vars
    
        AB_x = sympy.cos(alpha) * sympy.cos(beta)
        AB_y = sympy.sin(alpha) * sympy.cos(beta)
        AB_z = sympy.sin(beta)
        
        return AB_x, AB_y, AB_z

    def gen_R_nonlinear_eq(vars):
        theta, phi, gamma = vars

        r_x = np.array([[1, 0, 0],
                        [0, sympy.cos(theta), -1*sympy.sin(theta)],
                        [0, sympy.sin(theta), sympy.cos(theta)]])
        r_y = np.array([[sympy.cos(phi), 0, sympy.sin(phi)],
                        [0, 1, 0],
                        [-1*sympy.sin(phi), 0, sympy.cos(phi)]])
        r_z = np.array([[sympy.cos(gamma), -1*sympy.sin(gamma), 0],
                        [sympy.sin(gamma), sympy.cos(gamma), 0],
                        [0, 0, 1]])

        r = r_x.dot(r_y).dot(r_z)

        return r.T.flatten()

    def gen_systems_of_equations(self, x, driving_var, value):
        x_linear = np.zeros(27)
        
        x_linear[0:3] = x[0:3]

        x_linear[3:12] = self.gen_R_nonlinear_eq(x[3:6])
        #print(x_linear)
        #print("\n")

        ab_vecs = x_linear[12:]
        ab_angles = x[6:]

        for i in range(5):
            ab_vecs[3*i:3*i+3] = self.gen_AB_nonlinear_eq((ab_angles[2*i], ab_angles[2*i+1]))
        
        #print(x_linear)
        #print("\n")

        
        linear_equations = ((np.dot(self.P, x_linear)).T - self.A.T)[0]

        driving_expression = x[driving_var] - value

        #might want to change conactenatateoiwjep to np.append
        return np.concatenate((linear_equations, np.array([driving_expression])))

