import rosbag
import matplotlib.pyplot as plt
from datetime import datetime
from ocs2_msgs.msg import mpc_observation

def read_rosbag_data(bag_file, topic):
    # Open the bag file
    bag = rosbag.Bag(bag_file)
    
    # Initialize lists to store timestamps and specific data fields
    timestamps = []
    input = []
    state = []
    
    # Read messages from the specified topic
    for topic, msg, t in bag.read_messages(topics=[topic]):
        # Append the timestamp and data fields
        timestamps.append(t.to_sec())
        input.append(msg.input.value)
        state.append(msg.state.value)  # Assuming 'state' is a list or array-like
        
    # Close the bag file
    bag.close()

    # Normalize timestamps to start from zero
    start_time = timestamps[0]
    normalized_timestamps = [ts - start_time for ts in timestamps]
    
    return normalized_timestamps, input, state

def plot_input(timestamps, input):
    # Create the plot for input
    timestamps = timestamps[::100]
    input = input[::100]
    plt.figure(figsize=(15, 5))
    plt.plot(timestamps, input, label='input')
    plt.xlabel('Time')
    plt.ylabel('input')
    plt.title('Input over Time')
    plt.legend()
    plt.tight_layout()
    plt.show()

def plot_state(timestamps, state):
    # Create subplots for each component of the state
    fig, axs = plt.subplots(len(state), 1, figsize=(15, 5*len(state)))
    
    # Plot each component of the state
    for i, component in enumerate(state):
        axs[i].plot(timestamps, component, label=f'state[{i}]')
        axs[i].set_xlabel('Time')
        axs[i].set_ylabel(f'state[{i}]')
        axs[i].set_title(f'State[{i}] over Time')
        axs[i].legend()
    
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    # Specify the path to your ROS bag file and topic name
    bag_file = '/home/mohsen/Projects/manipulation_ws/bagfiles/adaptive.bag'
    topic = '/object_mpc_observation'
    
    # Read the data from the bag file
    timestamps, input, state = read_rosbag_data(bag_file, topic)
    
    # Plot the data
    plot_input(timestamps, input)
    # plot_state(timestamps, state)
