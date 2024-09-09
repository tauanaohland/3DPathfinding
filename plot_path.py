import numpy as np
import plotly.graph_objects as go
import sys

class VoxeMapLoader:

    def __init__(self, map_filename):
        self.limits = []
        self.map = self.__load_map_from_voxel(map_filename)
        
    def __load_path_from_file(self, path_filename):
        """
        This function loads a path from a file.
        The 'path' attribute is a list of coordinates representing the path.
        """

        path = []

        try:
            with open(path_filename, 'r') as file:
                line = file.readline()
                while line:
                    (x, y, z) = line.split(' ')
                    path.append([int(x), int(y), int(z)])
                    line = file.readline()

        except BaseException as be:
            path = None
            print(be)

        print("Path loaded successfully.")
        return path

    def __load_map_from_voxel(self, map_filename):
            """
            This function loads a 3D map from a voxel file.
            The 'map' attribute is a list of coordinates representing the voxels in the map.
            The 'limits' attribute is a list representing the dimensions of the map.
            """

            map = []

            try:
                with open(map_filename, 'r') as file:
                    # First line contains the dimensions of the map.
                    line = file.readline()  
                    (_, width, height, depth) = line.split(' ')
                    self.limits = [int(width), int(height), int(depth)]

                    line = file.readline()
                    while line:
                        (x, y, z) = line.split(' ')
                        map.append([int(x), int(y), int(z)])
                        line = file.readline()

            except BaseException as be:
                map = None
                print(be)

            print("Map loaded successfully.")
            return map
    
    def sweap_axis(self):
        """
        Sweaps x by y axis.
        """

        if not self.map:
            raise ValueError("The map object is empty.")
        
        aux = [[y, x, z] for x, y, z in self.map]
        self.map = aux
        self.limits[0], self.limits[1] = self.limits[1], self.limits[0]

        print("Map axis sweaped successfully.")

        

    def convert_map_obj_to_blocked_set(self):
        """
        Converts the map object to a set of blocked coordinates.
        """

        if not self.map:
            raise ValueError("The map object is empty.")

        return {(x, y, z) for x, y, z in self.map}
    
    def convert_map_obj_to_numpy_matrix(self):
        """
        Converts the map object to a numpy matrix, where the voxels are represented as 1s.
        """

        if not self.map:
            raise ValueError("The map object is empty.")

        limits = self.limits
        width = limits[0]
        height = limits[1]
        depth = limits[2]

        matrix = np.zeros((width, height, depth), dtype=np.float32)

        for x, y, z in self.map:
            matrix[x, y, z] = 1.0

        return matrix

    def plot_path_from_file(self, path_filename):
        """
        Plots the path from a file on the map.
        """


        path = self.__load_path_from_file(path_filename)
        self.plot_path(path)
    
    def plot_path(self, path):
        """
        Plots the path on the map.
        """

        path_x = [point[0] for point in path]
        path_y = [point[1] for point in path]
        path_z = [point[2] for point in path]

        blocked_set = self.convert_map_obj_to_blocked_set()
        obstacle_x, obstacle_y, obstacle_z = [], [], []

        for blocked in blocked_set:
            (x,y,z) = blocked
            obstacle_x.append(x)
            obstacle_y.append(y)
            obstacle_z.append(z)


        # Create plotly 3D scatter plot for obstacles
        obstacle_trace = go.Scatter3d(
            x=obstacle_x,
            y=obstacle_y,
            z=obstacle_z,
            mode='markers',
            marker=dict(
                size=2,
                color='blue',
                opacity=0.5
            ),
            name='Obstacles'
        )

        # Create plotly 3D scatter plot for the path
        path_trace = go.Scatter3d(
            x=path_x,
            y=path_y,
            z=path_z,
            mode='lines+markers',
            line=dict(
                color='red',
                width=2
            ),
            marker=dict(
                size=4,
                color='red'
            ),
            name='Path'
        )

        # Create plotly 3D scatter plot for start and end points
        start_trace = go.Scatter3d(
            x=[path_x[0]],
            y=[path_y[0]],
            z=[path_z[0]],
            mode='markers',
            marker=dict(
                size=6,
                color='green'
            ),
            name='Start'
        )

        end_trace = go.Scatter3d(
            x=[path_x[-1]],
            y=[path_y[-1]],
            z=[path_z[-1]],
            mode='markers',
            marker=dict(
                size=6,
                color='red'
            ),
            name='End'
        )

        # Layout for the plot
        layout = go.Layout(
            title='3D Path Planning',
            scene=dict(
                xaxis=dict(title='X'),
                yaxis=dict(title='Y'),
                zaxis=dict(title='Z'),
            )
        )

        # Create figure and plot
        fig = go.Figure(data=[obstacle_trace, path_trace, start_trace, end_trace], layout=layout)
        fig.show()
       
def main():

    #find map_filenap string from the terminal arguments
    map_filename = sys.argv[1]
    path_filename = 'path.csv'

    map_loader = VoxeMapLoader(map_filename)
    map_loader.plot_path_from_file(path_filename)
    
if __name__ == '__main__':
    main()