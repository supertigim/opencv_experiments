# -*- coding:utf-8 -*-
import numpy as np
import math 
from cv_utils import rotate_image, rotate_pixel
from cv_utils import remove_particles, make_binary_img
from cv_utils import draw_grid_on_map, path_on_map
from cv_utils import load_grey_img, save_image

######################################################################################
PIXEL_FROM_GRID_LINE = 2        # How close the available pixel is to the grid line
MIN_OCCUPANCY_RATE = 0.04       # 4%
REMOVABLE_PARTICLE_SCALE = 10   # How big particle can be removed 

DEBUG_INFO = True

ROTATED_MAP_IMG = './debug/rotated_img.jpg' if DEBUG_INFO else None
FM_MAP_IMG      = './debug/FM_map_img.jpg' if DEBUG_INFO else None
REMOVED_MAP_IMG = './debug/removed_particle_map_img.jpg' if DEBUG_INFO else None
GRID_MAP_IMG    = './debug/grid_map_img.jpg' if DEBUG_INFO else None
PATH_ON_MAP_IMG = './debug/path_on_map.jpg' if DEBUG_INFO else None

#######################################################################################

def find_optimal_starting_pos(map, robot_pixels, debug_msg=False):
    (height, width) = map.shape[:2]

    start_h = np.zeros((robot_pixels,))
    start_w = np.zeros((robot_pixels,))

    for h in range(height):
        for w in range(width):
            if map[h][w] == 0: continue
            start_h[h%robot_pixels] += 1
            start_w[w%robot_pixels] += 1

    start_h = np.argmin(start_h)
    start_w = np.argmin(start_w)

    if debug_msg: print('start_h:',start_h, ' start_w:', start_w)
    return start_h, start_w

def find_nearest_valid_pixel_from_center(area):
    (height, width)= area.shape[:2]

    c_h = height//2
    c_w = width//2

    q = []
    q.append((c_h,c_w))
    visited = set()

    while len(q):
        (h, w) = q.pop(0)
        if area[h][w] >= 1: 
            return h, w 
        d = math.sqrt((c_h-h)**2+(c_w-w)**2)
        for h_i in [h, h-1, h+1]:
            for w_i in [w, w-1, w+1]:
                if h_i < 0 or h_i >= height: continue
                if w_i < 0 or w_i >= width: continue
                d_i = math.sqrt((c_h-h_i)**2+(c_w-w_i)**2)
                if d_i <= d: continue
                s = str(h_i)+'-'+str(w_i)
                if s in visited: continue
                visited.add(s)
                q.append((h_i,w_i))
    return -1, -1

def check_fm_map_feasibility(fm_map, info):
    modified_fm_map = np.copy(fm_map)
    height, width = fm_map.shape[:2]
    for y in range(1, height-1):
        for x in range(1, width-1):
            if fm_map[y][x] == 0: continue  # not consider when this cell is obstacle 
            (top, bottom, left, right, occupancy_rate) = info[y][x]
            if occupancy_rate > 0.5: continue # this cell is always available 
            if fm_map[y-1][x] != 0 and top == 0: # Upper cell is ON but top side is not connected 
                    if left == 1 and right == 1: continue # To-Do: Need to improve code quality
                    modified_fm_map[y][x] = 0
            elif fm_map[y+1][x] != 0 and bottom == 0: # Lower cell is ON but bottom side is not connected 
                    if left == 1 and right == 1: continue
                    modified_fm_map[y][x] = 0
            elif fm_map[y][x-1] != 0 and left == 0: # Left cell is ON but left side is not connected 
                    if top == 1 and bottom == 1: continue
                    modified_fm_map[y][x] = 0
            elif fm_map[y][x+1] != 0 and right == 0: # Right Cell is ON but right side is not connected
                    if top == 1 and bottom == 1: continue
                    modified_fm_map[y][x] = 0
    return modified_fm_map

def generate_fm_map(filepath, robot_pixels):

    # 1. generate grey image 
    img, grey_img, org_height, org_width = load_grey_img(filepath)

    if img is None: 
        print(filepath, ' :No files existed')
        return 
    
    # 2. Remove unknown area in SLAM
    unknown_area = 1 + np.bincount(grey_img.flat)[1:-1].argmax() # Retrieve the index that is the biggest except 0, 255
    grey_img[grey_img==unknown_area] = 0                            
    make_binary_img(grey_img, 255//2)
    
    # 3. Rotation
    rotated_img, angle = rotate_image(grey_img, robot_pixels, ROTATED_MAP_IMG)    
    (rot_height, rot_width) = rotated_img.shape[:2]

    # 4. generate a path map where robot can move  
    path_map_img = path_on_map(rotated_img, robot_pixels, PATH_ON_MAP_IMG)

    # 5. Remove small particles which are not connected to the main roads
    path_map_img = remove_particles(path_map_img, REMOVABLE_PARTICLE_SCALE*robot_pixels**2, REMOVED_MAP_IMG)

    # 6. Find optimal starting position of grid system
    start_h, start_w = find_optimal_starting_pos(path_map_img, robot_pixels, DEBUG_INFO)
 
    # 7. Draw grid on the path map for debugging
    draw_grid_on_map(GRID_MAP_IMG, path_map_img, robot_pixels, start_h, start_w)
     
    # 8. Create Fleet Management Map 
    fm_map_max_X = rot_width//robot_pixels
    fm_map_max_Y = rot_height//robot_pixels
    fm_map = np.zeros((fm_map_max_Y, fm_map_max_X), np.uint8)
    fm_to_real_coords = [[(-1,-1) for _ in range(fm_map_max_X)] for _ in range(fm_map_max_Y)] 
    fm_to_cell_info = [[(0, 0, 0, 0, 0.0) for _ in range(fm_map_max_X)] for _ in range(fm_map_max_Y)] 

    min_yx = (fm_map_max_X-1,fm_map_max_Y-1)
    max_yx = (0,0)
    for x in range(fm_map_max_X):
        for y in range(fm_map_max_Y):
            w = start_w + x*robot_pixels
            h = start_h + y*robot_pixels
            area = path_map_img[h:h+robot_pixels, w:w+robot_pixels]//255
            (area_h, area_w) = area.shape[:2]
            
            top = int(area[0,1:-1].sum() > 0)
            bottom = int(area[-1,1:-1].sum() > 0)
            left = int(area[1:-1,0].sum() > 0)
            right = int(area[1:-1,-1].sum() > 0)
            occupancy_rate = area.sum()/(area_h*area_w)
            
            if occupancy_rate < MIN_OCCUPANCY_RATE: continue    # to exclude noisy case

            (path_h, path_w) = find_nearest_valid_pixel_from_center(area)
            fm_to_cell_info[y][x] = (top, bottom, left, right, occupancy_rate)

            # if available pixels in the area is too close to the grid line, then make it an obstacle 
            if path_h <= PIXEL_FROM_GRID_LINE or abs(robot_pixels- path_h) <= PIXEL_FROM_GRID_LINE or \
                    path_w <= PIXEL_FROM_GRID_LINE or abs(robot_pixels - path_w) <= PIXEL_FROM_GRID_LINE:
                continue

            # Rotate the location back and transform it into the original map 
            (path_h, path_w) = rotate_pixel((path_h+h,path_w+w), angle, (rot_height//2,rot_width//2))
            fm_to_real_coords[y][x] = (path_w+org_width//2, path_h+org_height//2)

            fm_map[y][x] = 255 # Set the cell to be available

            # Find the maximum size of FM map
            min_yx = min(y, min_yx[0]),min(x, min_yx[1])
            max_yx = max(y, max_yx[0]),max(x, max_yx[1])
            
    # 9. Final check of Fleet Management Map 
    fm_map = check_fm_map_feasibility(fm_map, fm_to_cell_info)
    fm_map = remove_particles(fm_map, robot_pixels)             # 9-1. remove detached particles again
    
    # 10. resize fleet management map for better performance 
    fm_map = fm_map[min_yx[0]:max_yx[0]+1, min_yx[1]:max_yx[1]+1]
    save_image(fm_map, FM_MAP_IMG)

    return fm_map, min_yx, max_yx, fm_to_real_coords, start_h, start_w, angle, rotated_img, img
    
if __name__ == '__main__':

    filepath = './maps/map.pgm'
    Robot_Pixels =  20

    if False:
        filepath = './maps/sample_map.pgm'
        Robot_Pixels =  10
    
    print(filepath, ' map image processing starts')

    import time
    tic = time.time()
    generate_fm_map(filepath,Robot_Pixels)
    print('Elapsed Time to process:', round(time.time()-tic, 2), ' seconds')