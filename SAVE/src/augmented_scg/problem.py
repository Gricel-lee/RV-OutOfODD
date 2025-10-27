import json
from typing import List, Dict, Any
import pandas as pd
import augmented_scg.situation as situation
import utility.auxiliary as aux
from pathlib import Path
import config.config as config


class SAVEProblem:

    def __init__(self, output_folder: str = "tN", csv_path: str = config.CSV_PATH, ignore_states: List[str] = []):
        '''
        @param 
        '''
        if not isinstance(csv_path, str) or not csv_path.endswith('.csv'):
            raise ValueError("csv_path must be a string ending with .csv")
        # paths
        self.csv_path = csv_path
        # self.output_folder = aux.create_folder(str(Path(self.csv_path).parent), "output_"+output_folder)
        output_folder_path=output_folder.split("/")
        if len(output_folder_path)<2:
            output_folder_path="."
        else:
            output_folder=output_folder_path[-1]
            output_folder_path_str="/".join(output_folder_path[:-1])
        self.output_folder = aux.create_folder(str(Path(output_folder_path_str)), "output_"+output_folder)
        self.dtmc_file = f"{self.output_folder}/dtmc.prism"
        # vars
        self.df_data = self._load_data_from_csv()
        self.ignore_states = ignore_states
        self.properties = self._parse_properties()
        self.prop_bounds = self.get_props_bounds() # follow the order of properties
        self.states = self._get_states()
        self.failures = self._get_failures()
        self.transitions = self._get_transitions()
        self.situations = self._get_situations()
        self.close_state_mod=config.TTC_BINS
        # get DTMC
        self.dtmc = self._get_dtmc(ignore_states=ignore_states)
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
        #Note: must be called after getting states and transitions
        #Note: situations (are states that are not failures)
        s_names = [state for state in self.states if state.lower().startswith('s')]
        situations = {}
        # set situations
        for s in s_names:
            sit = situation.Situation(name=s, transitions=self.transitions.get(s, []), problem=self)
            situations[s] = sit
        self.situations = situations
        # *after* setting all situations, get DTMCs
        return situations
        
            
    def _get_dtmc(self, ignore_states: List[str]=[]) -> str:
        s = aux.get_dtmc_model(self, ignore_states=ignore_states)
        return s
    

    def save_dtmc_file(self):
        if not self.dtmc:
            raise ValueError("[problemClass] DTMC is empty. Please generate it using get_dtmc() before saving.")
        with open(self.dtmc_file, 'w') as f:
            f.write(self.dtmc)
            if config.VERBOSE: print(f"[problemClass] DTMC saved to '{self.dtmc_file}'")

    def get_props_bounds(self) -> Dict[str, float]:
        prop_bounds = []
        with open(config.PROP_BOUND_PATH, 'r') as f:
            for line in f:
                line = line.strip()
                if line:  # skip empty lines
                    prop_bounds.append(line)
        return prop_bounds
        


    def get_pmc_results(self):
        """
        Runs PRISM for each situation and property, evaluates the results, and returns them as a single DataFrame.
        """
        results_list = []

        # For each situation, for each property, run prism and store result
        for situation in self.situations.values():
            for i_prop in range(len(self.properties)):
                prop = self.properties[i_prop]
                bound = self.prop_bounds[i_prop]

                # Get pmc result
                result = aux.run_prism_command(self.dtmc_file, prop, situation.i_state)

                # Determine if the bound is satisfied
                violation = not( eval(str(result) + bound) ) 

                # Calculate error to bound
                error = None
                if violation:
                    # Simplified and safer error calculation
                    if '<=' in bound:
                        error = result - float(bound.replace('<=', '').strip())
                    elif '>=' in bound:
                        error = float(bound.replace('>=', '').strip()) - result
                    elif '<' in bound:
                        # Add a small epsilon for strict inequalities
                        error = result - float(bound.replace('<', '').strip()) + 1e-9
                    elif '>' in bound:
                        error = float(bound.replace('>', '').strip()) - result + 1e-9
                
                # 2. Append a dictionary of results to the list
                results_list.append({
                    'Situation': situation.name,
                    'Property': prop,
                    'Result': result,
                    'Violation': violation,
                    'Error': error
                })
                
                print(f"[PMCresults] '{situation.name}', property '{prop}', result {result}, bound {bound}, error: {error}.")

        # 3. Create the final DataFrame from the list of results
        self.verification_results = pd.DataFrame(results_list)
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
        print(f"Properties: {self.properties}")
        print(f"Property bounds: {self.prop_bounds}")

