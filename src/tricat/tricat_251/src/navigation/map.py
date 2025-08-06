#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
import matplotlib.pyplot as plt
import guidance.guidance as guidance

class OccupancyMap():
    '''
    2차원 점유 그리드 map
    점유 그리드의 각 셀에는 해당 셀의 점유 확률을 나타내는 값 존재
    1에 가까운 값은 셀에 장애물이 있을 확률이 높음을 나타내며,
    0에 가까운 값은 셀이 점유되지 않았고 장애물이 없을 확률이 높음을 나타냄
    
    NED 좌표, 그리드 인덱스를 지원
    인덱스가 (0,0)인 첫 번째 그리드 위치는 그리드의 왼쪽 위 코너에서 시작
    '''
    
    def __init__(self, *args):
        '''
        width: 맵 너비 [m]
        height: 맵 높이 [m]
        resolution: 그리드 해상도 [m당 셀 개수]
        '''
        if len(args) == 2:
            # Case: OccupancyMap(width, height)
            self.width = args[0]
            self.height = args[1]
            self.resolution = 1  # default resolution

        elif len(args) == 3:
            # Case: OccupancyMap(width, height, resolution)
            self.width = args[0]
            self.height = args[1]
            self.resolution = args[2]

        elif len(args) == 1 and isinstance(args[0], OccupancyMap):
            # Case: OccupancyMap(sourcemap)
            source_map = args[0]
            self.width = source_map.width
            self.height = source_map.height
            self.resolution = source_map.resolution

        # TODO Resampling 함수 필요
        elif len(args) == 2 and isinstance(args[0], OccupancyMap):
            # Case: OccupancyMap(sourcemap, resolution)
            source_map = args[0]
            self.width = source_map.width
            self.height = source_map.height
            self.resolution = args[1]
            self.grid_type = source_map

        else:
            raise ValueError("Invalid arguments for OccupancyMap constructor.")

        self.rows = self.width*self.resolution
        self.cols = self.height*self.resolution
        self.grid = np.zeros((self.rows, self.cols))
        self.origin = (0, 0)

    def set_probability(self, grid_x, grid_y, probability):
        self.grid[grid_y, grid_x] = probability
        
    def obstacle_update(self, obstacles):
        for x, y in obstacles:
            self.set_probability(x, y, 1)

    def grid2ned(self, grid_x, grid_y):
        '''
        그리드 좌표를 NED 좌표로 변환
        로컬 좌표계의 원점 (0, 0)은 맵의 왼쪽 아래 코너
        '''
        x_ned = self.origin[0] + grid_x * self.resolution
        y_ned = self.origin[1] + (self.rows - 1 - grid_y) * self.resolution
        return (x_ned, y_ned)
    
    def map2ned(self):
        '''
        Converts the entire grid matrix to NED coordinates and returns it
        as a list of tuples (x_ned, y_ned, occupancy_probability).
        The origin can be set if the NED system is not starting at (0, 0).
        '''
        ned_list = []

        # Loop through the grid matrix
        for grid_y in range(self.rows):
            for grid_x in range(self.cols):
                # Get the occupancy probability
                occupancy_probability = self.grid[grid_y, grid_x]

                # Convert grid coordinates to NED coordinates
                x_ned, y_ned = self.grid2ned(grid_x, grid_y, self.origin)

                # Append the result as a tuple
                ned_list.append((x_ned, y_ned, occupancy_probability))
        
        return ned_list

    def matplot_plot(self, colormap='Greys'):
        '''
        Plots the occupancy grid using matplotlib with customizable colormap.
        '''
        plt.figure(figsize=(10, 10))
        plt.imshow(self.grid, cmap=colormap, origin='upper')
        plt.colorbar(label='Occupancy Probability')
        plt.grid(True)
        plt.xticks(np.arange(0, self.grid.shape[1], 2))  # Adjust tick frequency as needed
        plt.yticks(np.arange(0, self.grid.shape[0], 2))  # Adjust tick frequency as needed
        plt.xlabel('Grid X')
        plt.ylabel('Grid Y')
        plt.title('Occupancy Map')
        plt.show()

map = OccupancyMap(10, 10)
obstacles = [
    (3, 3),
    (4, 5),
    (6, 8),
    (9, 2)
]
map.obstacle_update(obstacles)

a_star = guidance.GlobalPathPlanner(map)
start = (0, 0)
end = (10, 10)
a_star.a_star_search(start, end)
print(map.grid)