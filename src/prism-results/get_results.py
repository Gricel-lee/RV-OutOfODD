import csv

# Input and output file names
input_file = "prisml_results.txt"
output_file = "prisml_results.csv"

# Lists to hold the relevant lines
model_checking_lines = []
result_lines = []

# Read the text file
with open(input_file, "r") as file:
    for line in file:
        line = line.strip()
        if line.startswith("Model checking: "):
            model_checking_lines.append(line[len("Model checking: "):])
        elif line.startswith("Result: "):
            line = line.replace(" (exact floating point)", "")
            result_lines.append(line[len("Result: "):])

# Ensure both lists are the same length
min_length = min(len(model_checking_lines), len(result_lines))
model_checking_lines = model_checking_lines[:min_length]
result_lines = result_lines[:min_length]

# Write to CSV
with open(output_file, "w", newline="") as csvfile:
    writer = csv.writer(csvfile)
    writer.writerow(["Model Checking", "Result"])
    for model, result in zip(model_checking_lines, result_lines):
        writer.writerow([model, result])

print(f"CSV file '{output_file}' has been created.")
