
class Kinematic_Model:
    def __init__(self, linkages, wheel_carrier):
        self.linkages = linkages
        self.wheel_carrier = wheel_carrier

        #check mobility and thow error if not properly defined

        self.linear_system, self.frame_pickups = self.__generate_linear_system__()

    @classmethod
    def from_file(self, file_name):
        pass
        
    def __generate_linear_system__(self):
        pass

    def __generate_nonlinear_equations__(self):
        pass
    
    def full_system_of_equations(self, driving_var, driving_value):
        pass

    def render(self):
        pass

    