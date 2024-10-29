import matplotlib.pyplot as plt

# Function to read execution times from the text file
def read_execution_times(filename):
    execution_times = []
    with open(filename, 'r') as file:
        for line in file:
            parts = line.split('::')
            if len(parts) >= 3:
                # Convert the execution time to a float and append it to the list
                execution_time = float(parts[2].strip())
                execution_times.append(execution_time)
    return execution_times

# Function to plot the execution times
def plot_execution_times(execution_times):
    plt.figure(figsize=(10, 5))
    plt.plot(execution_times, marker='o')
    plt.title('Execution Times')
    plt.xlabel('SLAM Message Index')
    plt.ylabel('Execution Time (seconds)')

    # Set x-ticks to every nth message index
    n = 20  # Change this to control the frequency of x-tick labels
    plt.xticks(range(0, len(execution_times), n))  # Show every n-th index
    plt.grid()
    plt.show()

# Main function
def main():
    filename = 'SLAM_execution_time.txt'  # Change this to your file path
    execution_times = read_execution_times(filename)
    plot_execution_times(execution_times)

if __name__ == '__main__':
    main()

# Main functio