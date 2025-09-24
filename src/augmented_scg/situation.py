from typing import List, Tuple
import augmented_scg.problem as Problem
import utility.auxiliary as aux
class Situation:
    """
    Represents a situation in the system with its associated transitions.
    """
    def __init__(self, name: str, transitions: List[tuple] = None, problem: Problem=None):
        self.name = name
        self.problem = problem
        self.dtmc_file = f"{self.problem.output_folder}/{self.name}_dtmc.prism"
        self.transitions: List[Tuple[str, float]] = transitions if transitions is not None else []
        self.dtmc = ""
        
    def get_dtmc(self, problem: Problem, ignore_states: List[str]=[]) -> str:
        '''If ignore_states is given, transitions from those states will be removed.'''

        self.problem = problem
        # get all situations and failures from the problem
        all_situations_n_failures = self.problem.states
        
        s = ""
        s+= 'dtmc\n\n'
        for i_state, situation in enumerate(all_situations_n_failures):
            s+= f'  const int {situation} = {i_state}; \n'
        s+= f'\nconst int init_situation = {self.name};\n'
        s+= '\nmodule System\n'
        s+= f'  s : [0..{len(all_situations_n_failures)-1}] init init_situation;\n\n'
        for i in sorted(self.problem.situations.keys()):
            if i not in ignore_states:    
                s+= f'  // Situation: {i}\n'
                s+= f'  [ ] s={i} -> '
                for next_situation, prob in self.problem.situations[i].transitions:
                    s+= f' {prob}:(s\'={next_situation}) + '
                s = s.rstrip(' + ') + ';\n\n'
        s+= 'endmodule\n'
        self.dtmc = s
        return s
    
    def save_dtmc(self):
        if not self.dtmc:
            raise ValueError("DTMC is empty. Please generate it using get_dtmc() before saving.")
        with open(self.dtmc_file, 'w') as f:
            f.write(self.dtmc)
        

    def display(self):
        print(f"Situation(name={self.name}, transitions={self.transitions})")
        
    