import csv
import json
import random
import os

class MockSimulation:
    def __init__(self, scenario):
        self.scenario = scenario
        self.state = 'PICK'
        self.current_time = 0.0
        self.step_start_time = 0.0
        self.cycle_start_time = 0.0
        
        self.objects_moved = 0
        self.conveyor_1_count = 0
        self.conveyor_2_count = 0
        self.telega_count = 0
        
        self.pickup_misses = 0
        self.drop_losses = 0
        self.total_errors = 0
        
        self.cycle_id = 0
        self.energy_proxy = 0.0
        self.last_cycle_time = 0.0
        self.cycle_time_sum = 0.0
        
        self.set_next_destination()
        self.calculate_durations()
        
        self.records = []

    def set_next_destination(self):
        if self.scenario == 'A':
            self.target_conveyor = 1
        elif self.scenario == 'B':
            self.target_conveyor = 1 if (self.objects_moved % 2 == 0) else 2
        elif self.scenario == 'C':
            mod = self.objects_moved % 3
            self.target_conveyor = mod + 1

    def calculate_durations(self):
        # Base durations reflecting physical distance
        # Conveyor 1: closest, Conveyor 2: medium, Telega: furthest
        self.t_pick = 1.2 + random.uniform(-0.1, 0.1)
        self.t_drop = 0.5 + random.uniform(-0.05, 0.05)
        
        if self.target_conveyor == 1:
            self.t_move = 1.5 + random.uniform(-0.15, 0.15)
            self.t_return = 1.0 + random.uniform(-0.1, 0.1)
            self.move_energy = 1.5
        elif self.target_conveyor == 2:
            self.t_move = 2.2 + random.uniform(-0.2, 0.2)
            self.t_return = 1.5 + random.uniform(-0.15, 0.15)
            self.move_energy = 2.5
        else: # Telega
            self.t_move = 3.0 + random.uniform(-0.3, 0.3)
            self.t_return = 2.0 + random.uniform(-0.2, 0.2)
            self.move_energy = 3.5

    def get_metrics_dict(self, status, error_type="none"):
        total_attempts = self.objects_moved + self.total_errors
        success_rate = (self.objects_moved / max(1, total_attempts)) * 100.0
        throughput_ppm = (self.objects_moved / max(0.1, self.current_time)) * 60.0
        avg_cycle_time = self.cycle_time_sum / max(1, self.objects_moved)
        
        return [
            round(self.current_time, 2),
            self.cycle_id,
            f"conveyor_{self.target_conveyor}" if self.target_conveyor < 3 else "telega",
            round(self.last_cycle_time, 2),
            status,
            error_type,
            self.objects_moved,
            self.conveyor_1_count,
            self.conveyor_2_count,
            self.telega_count,
            self.total_errors,
            self.pickup_misses,
            self.drop_losses,
            round(success_rate, 2),
            round(throughput_ppm, 2),
            round(avg_cycle_time, 2),
            round(self.energy_proxy, 2)
        ]

    def run(self):
        dt_step = 0.016  # 16ms time step
        while self.objects_moved < 30:
            self.current_time += dt_step
            dt = self.current_time - self.step_start_time
            
            if self.state == 'PICK':
                if dt >= self.t_pick:
                    # 3% chance of pickup miss
                    if random.random() < 0.03:
                        self.pickup_misses += 1
                        self.total_errors += 1
                        self.energy_proxy += 0.5 + 1.0  # pickup failure energy
                        self.last_cycle_time = (self.current_time - self.cycle_start_time)
                        self.cycle_id += 1
                        self.records.append(self.get_metrics_dict('FAILED_PICK', 'pickup_miss'))
                        
                        self.state = 'RETURN'
                        self.step_start_time = self.current_time
                        self.calculate_durations()
                    else:
                        self.state = 'MOVE'
                        self.step_start_time = self.current_time
                        
            elif self.state == 'MOVE':
                if dt >= self.t_move:
                    # 2% chance of dropping during move
                    if random.random() < 0.02:
                        self.drop_losses += 1
                        self.total_errors += 1
                        self.energy_proxy += self.move_energy * 0.5 + self.move_energy
                        self.last_cycle_time = (self.current_time - self.cycle_start_time)
                        self.cycle_id += 1
                        self.records.append(self.get_metrics_dict('FAILED_MOVE', 'drop_loss'))
                        
                        self.state = 'RETURN'
                        self.step_start_time = self.current_time
                        self.calculate_durations()
                    else:
                        self.state = 'DROP'
                        self.step_start_time = self.current_time
                        
            elif self.state == 'DROP':
                if dt >= self.t_drop:
                    self.objects_moved += 1
                    if self.target_conveyor == 1:
                        self.conveyor_1_count += 1
                    elif self.target_conveyor == 2:
                        self.conveyor_2_count += 1
                    else:
                        self.telega_count += 1
                    
                    self.energy_proxy += self.move_energy
                    self.last_cycle_time = (self.current_time - self.cycle_start_time)
                    self.cycle_time_sum += self.last_cycle_time
                    self.cycle_id += 1
                    self.records.append(self.get_metrics_dict('SUCCESS'))
                    
                    self.state = 'RETURN'
                    self.step_start_time = self.current_time
                    
            elif self.state == 'RETURN':
                if dt >= self.t_return:
                    self.state = 'PICK'
                    self.step_start_time = self.current_time
                    self.cycle_start_time = self.current_time
                    
                    self.set_next_destination()
                    self.calculate_durations()
                    
        return self.records

def save_to_csv(filename, records):
    header = [
        'Time', 'Cycle_ID', 'Destination', 'Duration', 'Status', 'Error_Type',
        'Objects_Moved', 'Conveyor_1', 'Conveyor_2', 'Telega', 'Errors',
        'Pickup_Misses', 'Drop_Losses', 'Success_Rate', 'Throughput_Ppm',
        'Avg_Cycle_Time', 'Energy_Proxy'
    ]
    with open(filename, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(header)
        writer.writerows(records)
    print(f"Generated {filename} with {len(records)} records.")

if __name__ == '__main__':
    random.seed(42)  # For reproducible data
    for sc in ['A', 'B', 'C']:
        sim = MockSimulation(sc)
        records = sim.run()
        save_to_csv(f'scenario_{sc}_metrics.csv', records)
