import matplotlib.pyplot as plt
import numpy as np

# Set font to Times New Roman
plt.rcParams["font.family"] = "Palatino Linotype"

# Data for average time (based on the provided values)
obstacles = ['o=8', 'o=18', 'o=28']
A_star = [0.0818, 0.0848, 0.4818]  # A* algorithm
LEA_star = [0.0188, 0.0201, 0.1073]  # LEA* algorithm
Lazy_SP = [0.2988, 0.3652, 2.7435]  # Lazy SP algorithm
LWA_star = [0.0210, 0.0223, 0.1622]  # LWA* algorithm
LRA_star = [6.7845, 12.2751, 18.1397]  # LRA* algorithm



# Setting the bar width
barWidth = 0.15

# Set position of bar on X axis
r1 = np.arange(len(obstacles))
r2 = [x + barWidth for x in r1]
r3 = [x + barWidth for x in r2]
r4 = [x + barWidth for x in r3]
r5 = [x + barWidth for x in r4]

# Create figure and axis
fig, ax = plt.subplots(figsize=(6, 6))

# Make the bar plot for average time
bars1 = ax.bar(r1, A_star, color='red', width=barWidth, edgecolor='grey', label='A*')
bars2 = ax.bar(r2, Lazy_SP, color='blue', width=barWidth, edgecolor='grey', label='LazySP')
bars3 = ax.bar(r3, LEA_star, color='green', width=barWidth, edgecolor='grey', label='LEA*')
bars4 = ax.bar(r4, LWA_star, color='cyan', width=barWidth, edgecolor='grey', label='LWA*')
bars5 = ax.bar(r5, LRA_star, color='magenta', width=barWidth, edgecolor='grey', label='LRA*')

# Add value labels on top of each bar with adjusted vertical positioning
for bar in bars1:
    yval = bar.get_height()
    ax.text(bar.get_x() + bar.get_width()/2, yval + 0.01, round(yval, 4), ha='center', va='bottom', fontsize=6)

for bar in bars2:
    yval = bar.get_height()
    ax.text(bar.get_x() + bar.get_width()/2, yval + 0.01, round(yval, 4), ha='center', va='bottom', fontsize=6)

for bar in bars3:
    yval = bar.get_height()
    ax.text(bar.get_x() + bar.get_width()/2, yval + 0.01, round(yval, 4), ha='center', va='bottom', fontsize=6)

for bar in bars4:
    yval = bar.get_height()
    ax.text(bar.get_x() + bar.get_width()/2, yval + 0.01, round(yval, 4), ha='center', va='bottom', fontsize=6)

for bar in bars5:
    yval = bar.get_height()
    ax.text(bar.get_x() + bar.get_width()/2, yval + 0.01, round(yval, 4), ha='center', va='bottom', fontsize=6)

# Add xticks on the middle of the group bars
ax.set_xlabel('Obstacles', fontweight='bold', fontsize=14)
ax.set_ylabel('Average Time (s)', fontweight='bold', fontsize=14)  # Y-axis label for time
ax.set_xticks([r + 2 * barWidth for r in range(len(obstacles))])
ax.set_xticklabels(obstacles)

# Create legend & Title
ax.set_title('Average Time Planning N=20000', fontweight='bold', fontsize=14)
ax.legend(loc='best')

# Adjust layout to be tight
plt.tight_layout()

# Save figure
plt.savefig('average_time_plot_e2_20000.png', dpi=800, format='png', bbox_inches='tight')

# Show plot
plt.show()
