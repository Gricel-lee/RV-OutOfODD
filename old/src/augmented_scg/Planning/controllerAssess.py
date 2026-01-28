import augmented_scg.problem as problem
import utility.auxiliary as aux
from pathlib import Path
import pandas as pd
import os
from typing import List

class ControllerAssessment:
    
    def __init__(self, problem: problem):
        self.problem = problem
        self.worst_situation = None
        self.worst_prop = None
        self.output_folder = ""

    def get_violations_exist(self):
        return self.problem.verification_results['Violation'].any()
    
    def get_worst_violation(self):
        if not self.get_violations_exist():
            return None
        # remove rows with "Violation" == False
        df = self.problem.verification_results[self.problem.verification_results['Violation']]
        df = df.sort_values(by='Error', ascending=False)
        worst_row = df.iloc[0] if not df.empty else None
        # add info
        self.worst_situation = worst_row["Situation"]
        self.worst_prop = worst_row["Property"]
        return self.worst_situation, self.worst_prop


    def create_new_csv_data(self, worst_situations: List[str], output_folder: str) -> str:
        self.output_folder = aux.create_folder(str(Path(self.problem.csv_path).parent), "output_"+output_folder)

        # Load the original CSV
        original_csv_path = self.problem.csv_path
        df = pd.read_csv(original_csv_path)
        
        # Change all transitions for the worst_situation to 0.0
        df.loc[df['Situation'].isin(worst_situations), 'Probability'] = 0.0
        
        # Save the new CSV
        df.to_csv(os.path.join(self.output_folder, "updated_DTMC_data.csv"), index=False)

        print(f"New CSV with updated transitions created at: {os.path.join(self.output_folder, 'updated_DTMC_data.csv')}")
        return os.path.join(self.output_folder, "updated_DTMC_data.csv")