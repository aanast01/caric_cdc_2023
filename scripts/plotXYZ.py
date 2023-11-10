import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
 
# Function to read coordinates from a text file
def read_xyz_from_file(file_path):
    data = []
    with open(file_path, 'r') as file:
        for line in file:
            # Strip whitespace and then split the line into components based on comma
            parts = line.strip().split(',')
            try:
                # Convert string parts to float and append to data list
                if len(parts) == 3:  # Ensure there are exactly three components
                    data.append([float(part) for part in parts])
            except ValueError as e:
                print(f"Error converting line to floats: {line}. Error: {e}")
                continue  # Skip lines that cannot be converted to floats
    return data
 
# Function to plot the coordinates
def plot_xyz_with_color(file_path):
    # Read coordinates from the file
    coordinates = read_xyz_from_file(file_path)
    
    # Create a new figure for the 3D plot
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
 
    # Separate the coordinates into x, y, and z for plotting
    xs, ys, zs = zip(*coordinates)
    
    # Map the z-values to colors
    colors = plt.cm.jet((zs-min(zs))/(max(zs)-min(zs)))
 
    # Create a scatter plot
    scatter = ax.scatter(xs, ys, zs, c=colors, cmap='jet')
    
    # Add a color bar which maps values to colors
    cbar = plt.colorbar(scatter, shrink=0.5, aspect=5)
    cbar.set_label('Z value')
 
    # Set labels
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
 
    # Show the plot
    plt.show()
 
# Path to the text file containing the coordinates
file_path = 'test2.txt'  # Replace with your file path
 
# Call the function to plot the coordinates
plot_xyz_with_color(file_path)