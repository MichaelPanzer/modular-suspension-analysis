import numpy as np



def __gen_P_R__(upright_pickups):
    p_r = np.zeros((5,3))

    for pickup_n in upright_pickups:
        print(pickup_n)

    for n in p_r:
        print(n)

    for pickup_n, p_r_row in upright_pickups, p_r:
        for pickup_coord, p_r_index in pickup_n, p_r_row:
            p_r_index = pickup_coord*np.identity(3)

    return p_r


mat = __gen_P_R__(np.array([[1,2,3],
                    [1,2,3],
                    [1,2,3],
                    [1,2,3],
                    [1,2,3],]))

print(mat)
        
