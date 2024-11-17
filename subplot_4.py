import matplotlib.pyplot as plt
import numpy as np

# Set font to Times New Roman
plt.rcParams["font.family"] = "Times New Roman"

# Data for edge evaluations for different N values
obstacles = ['o=8', 'o=18', 'o=28']
data = {
    1000: {
        'A_star': [931.6, 674.1, 1585.4],
        'Lazy_SP': [19.7, 20.8, 53.0],
        'LEA_star': [53.5, 44.1, 120.7],
        'LWA_star': [53.5, 44.1, 120.7],
        'LRA_star': [37.6, 32.9, 88.1]
    },
    5000: {
        'A_star': [3156.8, 6792.7, 6197.1],
        'Lazy_SP': [31.5, 93.5, 78.9],
        'LEA_star': [130.1, 310.8, 290.0],
        'LWA_star': [130.1, 310.8, 290.0],
        'LRA_star': [89.3, 218.0, 204.9]
    },
    10000: {
        'A_star': [12024.9, 10309.2, 12071.3],
        'Lazy_SP': [60.5, 94.9, 146.2],
        'LEA_star': [450.8, 418.2, 545.8],
        'LWA_star': [450.8, 418.2, 545.8],
        'LRA_star': [298.1, 290.2, 391.6]
    },
    20000: {
        'A_star': [18112.0, 19707.1, 23161.4],
        'Lazy_SP': [106.0, 174.3, 223.4],
        'LEA_star': [637.4, 751.8, 912.3],
        'LWA_star': [637.4, 751.8, 912.3],
        'LRA_star': [433.7, 522.0, 646.3]
    }
}

# Setting the bar width
barWidth = 0.15

# Create figure and 4 subplots
fig, axs = plt.subplots(2, 2, figsize=(12, 10))  # 2 rows, 2 columns of subplots
fig.suptitle('Edge Evaluations for Different N Values', fontsize=16)

# Iterate over the different N values
for i, (N_value, alg_data) in enumerate(data.items()):
    row = i // 2
    col = i % 2
    ax = axs[row, col]

    # Set position of bar on X axis
    r1 = np.arange(len(obstacles))
    r2 = [x + barWidth for x in r1]
    r3 = [x + barWidth for x in r2]
    r4 = [x + barWidth for x in r3]
    r5 = [x + barWidth for x in r4]

    # Plot the bars for each algorithm
    bars1 = ax.bar(r1, alg_data['A_star'], color='red', width=barWidth, edgecolor='grey', label='A*')
    bars2 = ax.bar(r2, alg_data['Lazy_SP'], color='blue', width=barWidth, edgecolor='grey', label='LazySP')
    bars3 = ax.bar(r3, alg_data['LEA_star'], color='green', width=barWidth, edgecolor='grey', label='LEA*')
    bars4 = ax.bar(r4, alg_data['LWA_star'], color='cyan', width=barWidth, edgecolor='grey', label='LWA*')
    bars5 = ax.bar(r5, alg_data['LRA_star'], color='magenta', width=barWidth, edgecolor='grey', label='LRA*')

    # Add value labels on top of each bar
    for bars in [bars1, bars2, bars3, bars4, bars5]:
        for bar in bars:
            yval = bar.get_height()
            ax.text(bar.get_x() + bar.get_width()/2, yval + 10, round(yval, 1), ha='center', va='bottom', fontsize=8)

    # Add xticks on the middle of the group bars
    ax.set_xlabel('Obstacles', fontweight='bold', fontsize=12)
    ax.set_ylabel('Number of Edge Evaluations', fontweight='bold', fontsize=12)
    ax.set_xticks([r + 2 * barWidth for r in range(len(obstacles))])
    ax.set_xticklabels(obstacles)
    ax.set_title(f'N = {N_value}', fontweight='bold', fontsize=14)

    # Create the legend for each subplot in the best position
    ax.legend(loc='best', fontsize=10)

# Adjust layout to be tight
plt.tight_layout()

# Save figure with 800 DPI
#plt.savefig('edge_evaluations_subplots.png', dpi=800, format='png', bbox_inches='tight')

# Show plot
plt.show()
