import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.axes_grid1.inset_locator import zoomed_inset_axes, mark_inset

# Set font to Times New Roman
plt.rcParams["font.family"] = "Times New Roman"

# Data for Average Time Planning (based on the table you've provided)
obstacles = ['o=8', 'o=18', 'o=28']

# Data for each algorithm for N=20000 (ε = 1)
A_star = [0.6687, 0.7351, 0.8507]  # A* algorithm
Lazy_SP = [2.7566, 7.3536, 10.7309]  # Lazy SP algorithm
LEA_star = [0.1357, 0.1542, 0.1891]  # LEA* algorithm
LWA_star = [0.1553, 0.1832, 0.2312]  # LWA* algorithm
LRA_star = [16.6354, 25.7300, 36.7992]  # LRA* algorithm

# Setting the bar width
barWidth = 0.15

# Set position of bars on X axis
r1 = np.arange(len(obstacles))
r2 = [x + barWidth for x in r1]
r3 = [x + barWidth for x in r2]
r4 = [x + barWidth for x in r3]
r5 = [x + barWidth for x in r4]

# Create figure and axis
fig, ax = plt.subplots(figsize=(10, 6))  # Adjusted aspect ratio for clarity

# Make the bar plot for average time planning
bars1 = ax.bar(r1, A_star, color='red', width=barWidth, edgecolor='grey', label='A*')
bars2 = ax.bar(r2, Lazy_SP, color='blue', width=barWidth, edgecolor='grey', label='LazySP')
bars3 = ax.bar(r3, LEA_star, color='green', width=barWidth, edgecolor='grey', label='LEA*')
bars4 = ax.bar(r4, LWA_star, color='cyan', width=barWidth, edgecolor='grey', label='LWA*')
bars5 = ax.bar(r5, LRA_star, color='magenta', width=barWidth, edgecolor='grey', label='LRA*')

# Add value labels on top of each bar
for bars in [bars1, bars2, bars3, bars4, bars5]:
    for bar in bars:
        yval = bar.get_height()
        ax.text(bar.get_x() + bar.get_width()/2, yval + 0.5, round(yval, 4), ha='center', va='bottom', fontsize=8)

# Add xticks on the middle of the group bars
ax.set_xlabel('Obstacles', fontweight='bold', fontsize=14)
ax.set_ylabel('Average Time (s)', fontweight='bold', fontsize=14)  # Y-axis label as Average Time
ax.set_xticks([r + 2 * barWidth for r in range(len(obstacles))])
ax.set_xticklabels(obstacles)

# Create legend & Title
ax.set_title('Average Time Planning (ε = 1, N=20000)', fontweight='bold', fontsize=14)
ax.legend(loc='best')  # Adjust legend to upper right

# Create zoomed inset 1 (for o=8)
axins1 = zoomed_inset_axes(ax, 1.5, loc='center left', bbox_to_anchor=(120,180), borderpad=2)  # Smaller size
axins1.bar(r1, A_star, color='red', width=barWidth, edgecolor='grey')
axins1.bar(r2, Lazy_SP, color='blue', width=barWidth, edgecolor='grey')
axins1.bar(r3, LEA_star, color='green', width=barWidth, edgecolor='grey')
axins1.bar(r4, LWA_star, color='cyan', width=barWidth, edgecolor='grey')
axins1.bar(r5, LRA_star, color='magenta', width=barWidth, edgecolor='grey')
axins1.set_xlim(-0.08, 0.5)
axins1.set_ylim([0, 2])  # Adjust limits to focus on the o=8 bars
axins1.set_xticks([])
#axins1.set_yticks([0, 0.25, 0.5, 0.75, 1])  # Set Y-ticks for zoom inset 1
#axins1.set_ylabel('Time (s)', fontsize=10)  # Add Y-axis label to inset 1

# Connect the first inset with the main plot
mark_inset(ax, axins1, loc1=1, loc2=3, fc="none", ec="0.5")

# Create zoomed inset 2 (for o=18)
axins2 = zoomed_inset_axes(ax, 1.5, loc='center')  # Smaller size
axins2.bar(r1, A_star, color='red', width=barWidth, edgecolor='grey')
axins2.bar(r2, Lazy_SP, color='blue', width=barWidth, edgecolor='grey')
axins2.bar(r3, LEA_star, color='green', width=barWidth, edgecolor='grey')
axins2.bar(r4, LWA_star, color='cyan', width=barWidth, edgecolor='grey')
axins2.bar(r5, LRA_star, color='magenta', width=barWidth, edgecolor='grey')
axins2.set_xlim(0.92, 1.52)
axins2.set_ylim(0, 2)  # Adjust limits to focus on the o=18 bars
axins2.set_xticks([])
axins2.set_yticks([0,2])  # Set Y-ticks for zoom inset 2
#axins2.set_ylabel('Time (s)', fontsize=10)  # Add Y-axis label to inset 2

# Connect the second inset with the main plot
mark_inset(ax, axins2, loc1=2, loc2=4, fc="none", ec="0.5")

# Create zoomed inset 3 (for o=28)
axins3 = zoomed_inset_axes(ax, 1.5, loc='center right')  # Smaller size
axins3.bar(r1, A_star, color='red', width=barWidth, edgecolor='grey')
axins3.bar(r2, Lazy_SP, color='blue', width=barWidth, edgecolor='grey')
axins3.bar(r3, LEA_star, color='green', width=barWidth, edgecolor='grey')
axins3.bar(r4, LWA_star, color='cyan', width=barWidth, edgecolor='grey')
axins3.bar(r5, LRA_star, color='magenta', width=barWidth, edgecolor='grey')
axins3.set_xlim(1.9, 2.52)
axins3.set_ylim(0, 2)  # Adjust limits to focus on the o=28 bars
axins3.set_xticks([])
axins3.set_yticks([0,2])  # Set Y-ticks for zoom inset 3
#axins3.set_ylabel('Time (s)', fontsize=10)  # Add Y-axis label to inset 3

# Connect the third inset with the main plot
mark_inset(ax, axins3, loc1=2, loc2=4, fc="none", ec="0.5")

# Adjust layout to be tight
#plt.tight_layout()

# Save figure with 800 DPI
plt.savefig('average_time_planning_e1_20000.png', dpi=800, format='png', bbox_inches='tight')

# Show plot
plt.show()
