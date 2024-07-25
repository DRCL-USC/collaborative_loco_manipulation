import rosbag
import matplotlib
import matplotlib.pyplot as plt
from datetime import datetime
from ocs2_msgs.msg import mpc_observation

matplotlib.rcParams['pdf.fonttype'] = 42
matplotlib.rcParams['ps.fonttype'] = 42
# plt.rcParams["font.family"] = "Times New Roman"
plt.rcParams["font.size"] = 36

def read_rosbag_data(bag_file, topic, tstart=0, tend=1000):
    # Open the bag file
    bag = rosbag.Bag(bag_file)
    
    # Initialize lists to store timestamps and specific data fields
    timestamps = []
    input = []
    state = []
    
    # Read messages from the specified topic
    for topic, msg, t in bag.read_messages(topics=[topic]):
        # Append the timestamp and data fields
        if tstart <= t.to_sec() <= tend:
            timestamps.append(t.to_sec())
            input.append(msg.input.value)
            state.append(msg.state.value) 
        
    # Close the bag file
    bag.close()

    # Normalize timestamps to start from zero
    start_time = timestamps[0]
    normalized_timestamps = [ts - start_time for ts in timestamps]
    
    return normalized_timestamps, input, state

def plot_input(timestamps, input):
    fig, axs = plt.subplots(len(input[0]), 1, figsize=(15, 5*len(input[0])))
    
    # Plot each input
    for i in range(len(input[0])):
        axs[i].plot(timestamps, [inp[i] for inp in input], label=f'input[{i}]')
        axs[i].set_xlabel('Time')
        axs[i].set_ylabel(f'input[{i}]')
        axs[i].set_title(f'Input[{i}] over Time')
        axs[i].legend()

def plot_state(timestamps, state):
    fig, axs = plt.subplots(len(state[0]), 1, figsize=(15, 5*len(state[0])))
    
    for i in range(len(state[0])):
        axs[i].plot(timestamps, [s[i] for s in state], label=f'state[{i}]')
        axs[i].set_xlabel('Time')
        axs[i].set_ylabel(f'state[{i}]')
        axs[i].set_title(f'State[{i}] over Time')
        axs[i].legend()

def plot_Force(timestamps1, input1, timestamps2, input2):
    # Plot each input
    for i in range(2):
        fig, axs = plt.subplots(figsize=(20, 6))
        axs.plot(timestamps1, [inp[i] for inp in input1], label=f'with Adaptive Controller', linewidth= 6)
        axs.plot(timestamps2, [inp[i] for inp in input2], label=f'w/o Adaptive Controller', linewidth= 6)
        axs.set_xlabel('Time [s]')
        axs.set_ylabel('Force [N]')
        axs.legend(loc='upper right', fontsize=24)
        plt.tight_layout()
        plt.savefig(f'Comparing_Forces_{i+1}.pdf' , dpi = 900)

if __name__ == "__main__":
    # Specify the path to your ROS bag file and topic name
    bag_file_adaptive = '/home/mohsen/Projects/manipulation_ws/bagfiles/adaptive.bag'
    bag_file_nonadaptive = '/home/mohsen/Projects/manipulation_ws/bagfiles/non_adaptive.bag'
    topic = '/object_mpc_observation'
    
    # Read the data from the bag file
    tstart = 8; tend = 50
    timestamps_adaptive, input_adaptive, state_adaptive = read_rosbag_data(bag_file_adaptive, topic, tstart, tend)
    timestamps_nonadaptive, input_nonadaptive, state_nonadaptive = read_rosbag_data(bag_file_nonadaptive, topic, tstart, tend)
    
    # Plot the data
    plot_Force(timestamps_adaptive, input_adaptive, timestamps_nonadaptive, input_nonadaptive)
    plt.show()
