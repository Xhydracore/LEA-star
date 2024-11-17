import matplotlib.pyplot as plt
import numpy as np

# Set font to Times New Roman
plt.rcParams["font.family"] = "Palatino Linotype"

# Data for edge evaluations (based on the table you've provided)
obstacles = ['o=8', 'o=18', 'o=28']
A_star = [2205.2, 2263.0, 13362.2]  # A* algorithm
Lazy_SP = [58.2, 78.4, 112.7]  # Lazy SP algorithm
LEA_star = [103.9, 133.1, 567.9]  # LEA* algorithm
LWA_star = [103.9, 133.1, 567.9]  # LWA* algorithm
LRA_star = [293.8, 370.9, 460.0]  # LRA* algorithm


# Setting the bar width
barWidth = 0.15

# Set position of bar on X axis
r1 = np.arange(len(obstacles))
r2 = [x + barWidth for x in r1]
r3 = [x + barWidth for x in r2]
r4 = [x + barWidth for x in r3]
r5 = [x + barWidth for x in r4]

# Create figure and axis
fig, ax = plt.subplots(figsize=(6, 6))  # 1:1 aspect ratio figure size

# Make the bar plot for edge evaluations
bars1 = ax.bar(r1, A_star, color='red', width=barWidth, edgecolor='grey', label='A*')
bars2 = ax.bar(r2, Lazy_SP, color='blue', width=barWidth, edgecolor='grey', label='LazySP')
bars3 = ax.bar(r3, LEA_star, color='green', width=barWidth, edgecolor='grey', label='LEA*')
bars4 = ax.bar(r4, LWA_star, color='cyan', width=barWidth, edgecolor='grey', label='LWA*')
bars5 = ax.bar(r5, LRA_star, color='magenta', width=barWidth, edgecolor='grey', label='LRA*')

# Add value labels on top of each bar
for bar in bars1:
    yval = bar.get_height()
    ax.text(bar.get_x() + bar.get_width()/2, yval + 10, round(yval, 1), ha='center', va='bottom', fontsize=8)

for bar in bars2:
    yval = bar.get_height()
    ax.text(bar.get_x() + bar.get_width()/2, yval + 10, round(yval, 1), ha='center', va='bottom', fontsize=8)

for bar in bars3:
    yval = bar.get_height()
    ax.text(bar.get_x() + bar.get_width()/2, yval + 10, round(yval, 1), ha='center', va='bottom', fontsize=8)

for bar in bars4:
    yval = bar.get_height()
    ax.text(bar.get_x() + bar.get_width()/2, yval + 10, round(yval, 1), ha='center', va='bottom', fontsize=8)

for bar in bars5:
    yval = bar.get_height()
    ax.text(bar.get_x() + bar.get_width()/2, yval + 10, round(yval, 1), ha='center', va='bottom', fontsize=8)

# Add xticks on the middle of the group bars
ax.set_xlabel('Obstacles', fontweight='bold',fontsize=14)
ax.set_ylabel('Number of Edge Evaluations', fontweight='bold', fontsize=14)  # Y-axis label as edge evaluations
ax.set_xticks([r + 2 * barWidth for r in range(len(obstacles))])
ax.set_xticklabels(obstacles)

# Create legend & Title
ax.set_title('Edge Evaluations N=20000', fontweight='bold',fontsize=14)
ax.legend()

# Ensure equal aspect ratio
ax.set_box_aspect(1)  # Equal aspect ratio

# Adjust layout to be tight
plt.tight_layout()

# Save figure with 800 DPI and 1:1 aspect ratio
plt.savefig('edge_evaluations_e2_20000.png', dpi=800, format='png', bbox_inches='tight')

# Show plot
plt.show()
