import pandas as pd
import matplotlib.pyplot as plt
import os

def plot_scenario(scenario_name):
    filename = f'scenario_{scenario_name}_metrics.csv'
    if not os.path.exists(filename):
        print(f"File {filename} not found.")
        return

    df = pd.read_csv(filename)
    
    plt.figure(figsize=(10, 6))
    plt.plot(df['Time'], df['Conveyor_1'], label='Conveyor 1', marker='o')
    plt.plot(df['Time'], df['Conveyor_2'], label='Conveyor 2', marker='s')
    if 'Telega' in df.columns and df['Telega'].max() > 0:
        plt.plot(df['Time'], df['Telega'], label='Telega', marker='^')
    
    plt.title(f'Objects Distributed over Time (Scenario {scenario_name})')
    plt.xlabel('Time (s)')
    plt.ylabel('Total Objects Placed')
    plt.legend()
    plt.grid(True)
    plt.savefig(f'scenario_{scenario_name}_plot.png')
    print(f"Saved scenario_{scenario_name}_plot.png")
    
    # Calculate and print stats
    print(f"\n--- Statistics for Scenario {scenario_name} ---")
    print(f"Total Objects Moved: {df['Objects_Moved'].iloc[-1]}")
    print(f"Errors: {df['Errors'].iloc[-1]}")
    print(f"Avg Cycle Time: {df['Avg_Cycle_Time'].iloc[-1]:.2f} s")

if __name__ == '__main__':
    print("Generating plots for Scenarios A, B, and C...")
    for sc in ['A', 'B', 'C']:
        plot_scenario(sc)
