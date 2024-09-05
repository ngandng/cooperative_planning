import re

# Open and read the LaTeX file
with open('ucurve.txt', 'r') as file:
    content = file.read()

# Regular expression to match coordinates and labels for goals (dotstyle=*)
goal_pattern = r'\\psdots\[dotstyle=\*,linecolor=.*?\]\(([\d.-]+),([\d.-]+)\)'

# Regular expression to match coordinates and labels for tasks (dotstyle=square*)
task_pattern = r'\\psdots\[dotstyle=square\*,dotangle=.*?,linecolor=.*?\]\(([\d.-]+),([\d.-]+)\)'

# Find all matches for goals and tasks
goals = re.findall(goal_pattern, content)
tasks = re.findall(task_pattern, content)

# Prepare the text content
output_lines = ["GOAL"]

# Process goals
for x, y in goals:
    output_lines.append(f"{float(x):.2f}\t{float(y):.2f}")

output_lines.append("NODE_COORD_SECTION")

# Process tasks (assigning an arbitrary ID starting from 1)
for i, (x, y) in enumerate(tasks, start=1):
    output_lines.append(f"{i}\t{int(float(x))}\t{int(float(y))}")

# Write to a text file
with open('ucurve_out.txt', 'w') as output_file:
    output_file.write("\n".join(output_lines))

print("Data saved to output.txt")
