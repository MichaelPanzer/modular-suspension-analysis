from suspmatics import components
import itertools
import vpython#type: ignore

class Renderer:
    def __init__(self, components: list[components.Component]):
        self.components = components
        self.input_var_indices: list[int] = list(itertools.accumulate([c.input_count for c in self.components])) #there has to be a better way to do this
        self.vp_objects: list[vpython.compound] = [comp.create_vp_object() for comp in components]

    
    def render(self, vars: components.array32) -> None:
        for (comp, vp_obj, end_index) in zip(self.components, self.vp_objects,self.input_var_indices):
            comp.update_vp_position(vp_obj, vars[end_index-comp.input_count:end_index])

        return

    def sae_basis_vectors(self) -> None:
        vpython.arrow(axis=vpython.vector(0,0,50), color=vpython.color.blue)
        vpython.arrow(axis=vpython.vector(-50,0,0), color=vpython.color.green)
        vpython.arrow(axis=vpython.vector(0,-50,0), color=vpython.color.red)
        return
    
    def animate(self, solution_matrix: components.array32, loops:int=5, rate:int=50) -> None:
        for i in range(loops):
            for pos in solution_matrix:
                vpython.rate(rate)
                self.render(pos)
            for pos in reversed(solution_matrix):
                vpython.rate(rate)
                self.render(pos)

        return


#TODO create plotting