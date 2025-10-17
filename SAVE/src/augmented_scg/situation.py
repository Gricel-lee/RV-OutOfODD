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
        self.transitions: List[Tuple[str, float]] = transitions if transitions is not None else []
        self.i_state = self.problem.states.index(name) if name in self.problem.states else -1 # integer value of state variable in the DTMC
        

    def display(self):
        print(f"Situation(name={self.name}, transitions={self.transitions})")
        
    