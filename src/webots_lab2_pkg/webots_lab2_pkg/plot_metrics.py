import csv
import matplotlib.pyplot as plt
import os

def read_csv_to_dict(filename):
    data = {
        'Time': [], 'Cycle_ID': [], 'Destination': [], 'Duration': [],
        'Status': [], 'Error_Type': [], 'Objects_Moved': [], 'Conveyor_1': [],
        'Conveyor_2': [], 'Telega': [], 'Errors': [], 'Pickup_Misses': [],
        'Drop_Losses': [], 'Success_Rate': [], 'Throughput_Ppm': [],
        'Avg_Cycle_Time': [], 'Energy_Proxy': []
    }
    
    with open(filename, mode='r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            data['Time'].append(float(row['Time']))
            data['Cycle_ID'].append(int(row['Cycle_ID']))
            data['Destination'].append(row['Destination'])
            data['Duration'].append(float(row['Duration']))
            data['Status'].append(row['Status'])
            data['Error_Type'].append(row['Error_Type'])
            data['Objects_Moved'].append(int(row['Objects_Moved']))
            data['Conveyor_1'].append(int(row['Conveyor_1']))
            data['Conveyor_2'].append(int(row['Conveyor_2']))
            data['Telega'].append(int(row['Telega']))
            data['Errors'].append(int(row['Errors']))
            data['Pickup_Misses'].append(int(row['Pickup_Misses']))
            data['Drop_Losses'].append(int(row['Drop_Losses']))
            data['Success_Rate'].append(float(row['Success_Rate']))
            data['Throughput_Ppm'].append(float(row['Throughput_Ppm']))
            data['Avg_Cycle_Time'].append(float(row['Avg_Cycle_Time']))
            data['Energy_Proxy'].append(float(row['Energy_Proxy']))
            
    return data

def rolling_mean(arr, window=3):
    res = []
    for i in range(len(arr)):
        start = max(0, i - window + 1)
        subset = arr[start:i+1]
        res.append(sum(subset) / len(subset))
    return res

def generate_plots():
    scenarios = ['A', 'B', 'C']
    dfs = {}
    
    # Load data using standard csv module
    for sc in scenarios:
        filename = f'scenario_{sc}_metrics.csv'
        if not os.path.exists(filename):
            print(f"File {filename} not found. Please run generate_mock_data.py first.")
            return
        dfs[sc] = read_csv_to_dict(filename)
    
    # Set modern style settings
    plt.rcParams.update({
        'font.size': 11,
        'axes.labelsize': 12,
        'axes.titlesize': 14,
        'xtick.labelsize': 10,
        'ytick.labelsize': 10,
        'figure.titlesize': 16,
        'grid.alpha': 0.3
    })
    
    # Make a folder for plots if it doesn't exist
    os.makedirs('plots', exist_ok=True)
    
    # 1. Cumulative objects moved over time
    plt.figure(figsize=(10, 6))
    for sc, color in zip(scenarios, ['#2ca02c', '#1f77b4', '#ff7f0e']):
        data = dfs[sc]
        # Filter for successful drops
        times = [t for t, s in zip(data['Time'], data['Status']) if s == 'SUCCESS']
        moved = [m for m, s in zip(data['Objects_Moved'], data['Status']) if s == 'SUCCESS']
        plt.plot(times, moved, label=f'Сценарий {sc}', color=color, linewidth=2, marker='o', markersize=4)
        
    plt.title('Накопительный итог перемещенных деталей')
    plt.xlabel('Время симуляции (с)')
    plt.ylabel('Количество перенесенных деталей (шт.)')
    plt.legend()
    plt.grid(True)
    plt.savefig('plots/comparison_throughput_accumulated.png', dpi=300, bbox_inches='tight')
    plt.close()
    
    # 2. Cycle time distributions boxplot
    plt.figure(figsize=(10, 6))
    cycle_data = []
    labels = []
    for sc in scenarios:
        data = dfs[sc]
        success_durations = [dur for dur, s in zip(data['Duration'], data['Status']) if s == 'SUCCESS']
        cycle_data.append(success_durations)
        labels.append(f'Сценарий {sc}')
        
    box = plt.boxplot(cycle_data, labels=labels, patch_artist=True, notch=False)
    
    colors = ['#d6f5d6', '#d6e5f5', '#ffe5d6']
    for patch, color in zip(box['boxes'], colors):
        patch.set_facecolor(color)
        patch.set_edgecolor('#333333')
        patch.set_linewidth(1.5)
        
    for median in box['medians']:
        median.set_color('#d62728')
        median.set_linewidth(2)
        
    plt.title('Распределение времени успешного цикла перемещения')
    plt.ylabel('Время цикла (с)')
    plt.grid(True, axis='y')
    plt.savefig('plots/comparison_cycle_time_boxplot.png', dpi=300, bbox_inches='tight')
    plt.close()
    
    # 3. Throughput (PPM) over time
    plt.figure(figsize=(10, 6))
    for sc, color in zip(scenarios, ['#2ca02c', '#1f77b4', '#ff7f0e']):
        data = dfs[sc]
        times = [t for t, s in zip(data['Time'], data['Status']) if s == 'SUCCESS']
        ppms = [ppm for ppm, s in zip(data['Throughput_Ppm'], data['Status']) if s == 'SUCCESS']
        rolling_ppm = rolling_mean(ppms, window=3)
        plt.plot(times, rolling_ppm, label=f'Сценарий {sc}', color=color, linewidth=2, marker='s', markersize=4)
        
    plt.title('Производительность манипулятора во времени (PPM)')
    plt.xlabel('Время симуляции (с)')
    plt.ylabel('Производительность (деталей в минуту)')
    plt.legend()
    plt.grid(True)
    plt.savefig('plots/comparison_throughput_ppm.png', dpi=300, bbox_inches='tight')
    plt.close()
    
    # 4. Energy proxy vs Objects moved
    plt.figure(figsize=(10, 6))
    for sc, color in zip(scenarios, ['#2ca02c', '#1f77b4', '#ff7f0e']):
        data = dfs[sc]
        plt.plot(data['Objects_Moved'], data['Energy_Proxy'], label=f'Сценарий {sc}', color=color, linewidth=2)
        
    plt.title('Затраченная условная энергия на перемещение деталей')
    plt.xlabel('Количество перемещенных деталей (шт.)')
    plt.ylabel('Условные энергозатраты (ед. работы)')
    plt.legend()
    plt.grid(True)
    plt.savefig('plots/comparison_energy_efficiency.png', dpi=300, bbox_inches='tight')
    plt.close()
    
    # Print comparison table
    print("=" * 90)
    print(f"{'Сценарий':<10} | {'Всего деталей':<15} | {'Время работы (с)':<18} | {'Средний цикл (с)':<18} | {'Ошибки':<10} | {'Успешность %':<15} | {'Энергозатраты':<15}")
    print("-" * 90)
    for sc in scenarios:
        data = dfs[sc]
        
        total_time = data['Time'][-1]
        total_moved = data['Objects_Moved'][-1]
        
        success_durations = [dur for dur, s in zip(data['Duration'], data['Status']) if s == 'SUCCESS']
        avg_cycle = sum(success_durations) / len(success_durations) if success_durations else 0.0
        
        total_errors = data['Errors'][-1]
        success_rate = data['Success_Rate'][-1]
        total_energy = data['Energy_Proxy'][-1]
        
        print(f"Сценарий {sc:<2} | {int(total_moved):<15} | {total_time:<18.2f} | {avg_cycle:<18.2f} | {int(total_errors):<10} | {success_rate:<15.2f} | {total_energy:<15.2f}")
    print("=" * 90)
    print("Все графики успешно сгенерированы и сохранены в папку 'plots/'.")

if __name__ == '__main__':
    generate_plots()
