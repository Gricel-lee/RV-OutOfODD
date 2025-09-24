import json
from typing import List, Dict, Any
import pandas as pd
import augmented_scg.situation as situation
import utility.auxiliary as aux
from pathlib import Path
import config.config as config


class Problem:

    def __init__(self, output_folder: str = "tN", csv_path: str = config.CSV_PATH, ignore_states: List[str] = []):
        '''
        @param 
        '''
        if not isinstance(csv_path, str) or not csv_path.endswith('.csv'):
            raise ValueError("csv_path must be a string ending with .csv")
        # paths
        self.csv_path = csv_path
        self.output_folder = aux.create_folder(str(Path(self.csv_path).parent), "output_"+output_folder)
        # vars
        self.df_data = self._load_data_from_csv()
        self.ignore_states = ignore_states
        self.properties = self._parse_properties()
        self.prop_bounds = self.get_props_bounds() # follow the order of properties
        self.states = self._get_states()
        self.failures = self._get_failures()
        self.transitions = self._get_transitions()
        self.situations = self._get_situations()
        # verification results
        self.verification_results = {}
        
    def _parse_properties(self) -> list:
        # read PCTL props
        properties = []
        try:
            with open(config.PROPERTIES_PATH, 'r') as f:
                for line in f:
                    line = line.strip()
                    if line:  # skip empty lines
                        properties.append(line)
        except Exception as e:
            print(f"Error reading properties file: {e}")
        return properties
        
    

    def _load_data_from_csv(self) -> Dict[str, Any]:
        try:
            df = pd.read_csv(self.csv_path)
            return df
        except FileNotFoundError:
            print(f"Error: The file at '{self.json_path}' was not found.")
            return {}
        except json.JSONDecodeError:
            print(f"Error: Could not decode JSON from the file at '{self.json_path}'.")
            return {}
        except Exception as e:
            print(f"An unexpected error occurred while reading the file: {e}")
            return {}

    def _get_states(self) -> List[str]:
        # Get all model states
        situations = self.df_data['Situation'].unique()
        next_situations = self.df_data['Next'].unique()
        return sorted(list(set(situations).union(set(next_situations))))
    
    
    def _get_failures(self) -> List[str]:
        return [state for state in self.states if state.lower().startswith('f')]
    
    def _get_transitions(self) -> Dict[str, List[tuple]]:
        transitions = {}
        for _, row in self.df_data.iterrows():
            s = row['Situation']
            next = row['Next']
            prob = row['Probability']
            if s not in transitions.keys():
                transitions[s] = []
            transitions[s].append((next, prob))
        return transitions
    
    def _get_situations(self) -> List[str]:
        s_names = [state for state in self.states if state.lower().startswith('s')]
        situations = {}
        # set situations
        for s in s_names:
            sit = situation.Situation(name=s, transitions=self.transitions.get(s, []), problem=self)
            situations[s] = sit
        self.situations = situations
        # *after* setting all situations, get DTMCs
        return self._get_dtmcs(s_names, situations)
        
    
    def _get_dtmcs(self, s_names, situations):
        # Note: ignore states are needed at runtime for adaptation 
        for s in s_names:
            situations[s].get_dtmc(self, self.ignore_states)
        return situations
        

    
    def save_dtmc_files(self: str):
        for situation in self.situations.values():
            situation.save_dtmc()
            print(f"DTMC for situation '{situation.name}' saved to '{situation.dtmc_file}'")
    
    def get_props_bounds(self) -> Dict[str, float]:
        prop_bounds = []
        with open(config.PROP_BOUND_PATH, 'r') as f:
            for line in f:
                line = line.strip()
                if line:  # skip empty lines
                    prop_bounds.append(line)
        print(f"Property bounds loaded: {prop_bounds}")
        return prop_bounds
        
    def get_pmc_results(self):
        print("Solving using PRISM...")
        # create df
        verif_results = pd.DataFrame(columns=['Situation', 'Property', 'Result','Violation','Error'])
        
        # for each situation, for each property, run prism and store result
        for situation in self.situations.values():
            for i_prop in range(len(self.properties)):
                prop = self.properties[i_prop]
                bound = self.prop_bounds[i_prop]
                # get pmc result
                result = aux.run_prism_command(situation.dtmc_file, prop)
                
                # violation if bound not satisfied
                print(situation.dtmc_file, prop)
                print(f"Evaluating: {result} {bound}")
                print(eval(str(result) + bound))
                violation = not( eval(str(result) + bound) )
                # calculate error
                
                if violation:
                    if '<=' in bound:
                        error = result - float(bound.split('<=')[1])
                    elif '>=' in bound:
                        error = float(bound.split('>=')[1]) - result
                    elif '<' in bound:
                        error = result - float(bound.split('<')[1]) + 1e-6
                    elif '>' in bound:
                        error = float(bound.split('>')[1]) - result + 1e-6
                else:
                    error = None

                verif_results = pd.concat([verif_results, pd.DataFrame({'Situation': [situation.name], 'Property': [prop], 'Result': [result], 'Violation': [violation], 'Error': [error]})])
                print(f"Situation: {situation.name}, Property: {prop}, Result: {result}, Violation: {violation}, Error: {error}")

        self.verification_results = verif_results
        return self.verification_results

    def display(self):
        """
        Prints a summary of the problem's attributes to the console.
        """
        print("--- Problem Details ---")
        print(f"CSV Path: {self.csv_path}")
        print(f"Total States: {len(self.states)}")
        print(f"Situations ({len(self.situations)}): {[s.name for s in self.situations.values()]}")
        print(f"Failures ({len(self.failures)}): {self.failures}")
        print(f"Transitions: {self._get_transitions()}")
        print(f"Output folder: {self.output_folder}")
        print(f"Props: {self.properties}")
        
    