import matplotlib.pyplot as plt
import numpy as np


def plot_live_points(points):
    # Create a figure and axis object
    fig, ax = plt.subplots()

    # Initialize x and y arrays
    x = []
    y = []

    # Start a loop to continuously update the plot
    
    # Append the new points to the arrays
    while(True):    
        for point in points:
            x.append(point[0])
            x.append(point[2])
            y.append(point[1])
            y.append(point[3])

        # Clear the axis and plot the points
        ax.clear()
        ax.scatter(x, y)

        # Set the axis limits and display the plot
        ax.set_xlim(0, 1000)
        ax.set_ylim(0, 1000)
        plt.draw()
        plt.pause(0.1)
if __name__ == "__main__":
    
    loaded_array = np.load('my_array.npy',allow_pickle=True)
    print(loaded_array)
    #np.savetxt('txtfile.csv',loaded_array,delimiter=',')
    #print(loaded_array)
    #plot_live_points(loaded_array)