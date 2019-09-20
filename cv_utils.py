from skimage.morphology import skeletonize
from skimage import data
from scipy import ndimage
import cv2
import numpy as np
from matplotlib import pyplot as plt 
import sknw
import math 

def save_image(img, filepath):
    if filepath is not None and img is not None: cv2.imwrite(filepath, img)

def load_grey_img(filepath):
    img = cv2.imread(filepath)
    if img is None: return None, None, 0, 0
    (height, width) = img.shape[:2]
    grey_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    return img, grey_img, height, width

def create_circular_kernel(radius, debug_msg=False):
    radius = radius - 1
    kernel = np.zeros((2*radius+1, 2*radius+1), np.uint8) 
    y,x = np.ogrid[-radius:radius+1, -radius:radius+1]
    mask = x**2 + y**2 <= radius**2
    kernel[mask] = 1
    (h,w) = kernel.shape[:2]
    kernel = kernel[1:h-1, 1:w-1]
    if debug_msg: print('Kernal with the size:', radius, '\n', kernel)
    return kernel

def is_divided(img):
    contours, _ = cv2.findContours(img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    return len(contours) > 1
    
def skeleton(img, save_filepath=None):
    (height, width) = img.shape[:2]
    img = img // 255
    ske = skeletonize(img).astype(np.uint16)

    result = np.zeros((height,width), np.uint8)

    # build graph from skeleton
    graph = sknw.build_sknw(ske, multi=False)
    
    sknw.draw_graph(result,graph)

    result[result>0] = 255
    save_image(result, save_filepath)
    return result

def morphologyEx(img,robot_pixels, save_filepath=None):
    ret = cv2.morphologyEx(img, cv2.MORPH_OPEN, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (robot_pixels-3, robot_pixels-3)))
    ret[ret>0] = 128
    ret = img + ret
    ret[ret>255] = 128
    save_image(ret, save_filepath)
    return ret

def rotate_image(img_gray, minLineLength, save_filepath=None):
    img_edges = cv2.Canny(img_gray, 100, 100, apertureSize=3)
    #save_image(img_edges, 'edges_img.jpg')
    lines = None
    while (lines is None) and (minLineLength < min(img_gray.shape[:2])):
        lines = cv2.HoughLinesP(img_edges, 1, math.pi / 180.0, 100, minLineLength=minLineLength, maxLineGap=5)
        minLineLength += 1
    if minLineLength == 0 : return img_gray, 0
    angles = []
    most_freq_ang = {}
    for line in lines:
        x1, y1, x2, y2 = line[0]
        #cv2.line(img_before, (x1, y1), (x2, y2), (255, 0, 0), 3)
        angle = math.degrees(math.atan2(y2 - y1, x2 - x1))
        angles.append(angle)
        deg = round(angle+0.5, 0)
        if str(deg) in most_freq_ang: most_freq_ang[str(deg)].append(len(angles)-1)
        else: most_freq_ang[str(deg)] = [len(angles)-1]

    key = max(most_freq_ang, key=lambda k: len(most_freq_ang[k]))
    angle = angles[most_freq_ang[key][0]]
    img_rotated = ndimage.rotate(img_gray, angle)
    threshold_after_rotation = np.bincount(img_rotated.flat).argmin()
    make_binary_img(img_rotated,threshold_after_rotation) # make binary agin 
    save_image(img_rotated, save_filepath)
    return img_rotated, angle

def remove_particles(img, min_size=400, save_filepath=None):
    nb_components, output, stats, _ = cv2.connectedComponentsWithStats(img, connectivity=4)
    sizes = stats[1:, -1]; nb_components = nb_components - 1

    removed_img = np.zeros((output.shape), np.uint8)
    for i in range(0, nb_components):
        if sizes[i] >= min_size:
            removed_img[output == i + 1] = 255
    
    save_image(removed_img, save_filepath)
    return removed_img

def make_binary_img(img, threshold):
    # filter colors, only white: 255, grey: THREDSHOLD_COLOR_V, and black: 0 are left
    img[img > threshold] = 255
    img[img <= threshold] = 0

def rotate_pixel(xy, degrees, center=(0,0)):
    """Use numpy to build a rotation matrix and take the dot product."""
    #x, y = xy
    x, y = xy[0] - center[0], xy[1] - center[1]
    radians = math.radians(degrees)
    c, s = np.cos(radians), np.sin(radians)
    j = np.matrix([[c, s], [-s, c]])
    m = np.dot(j, [x, y])
    return int(m.T[0]), int(m.T[1])

def draw_grid_on_map(save_filepath, img_org, line_interval, h_s=0, w_s=0):
    ''' Draw grid lines on map '''
    if save_filepath is None: return 
    grid_img = np.copy(img_org)
    (height, width) = grid_img.shape[:2]

    for x in range(0, width, line_interval):
        cv2.line(grid_img, (w_s+x, 0), (w_s+x, height-1), (0, 0, 0), 2)

    for y in range(0, height, line_interval):
        cv2.line(grid_img, (0, y+h_s), (width-1, y+h_s), (0, 0, 0), 2)
        cv2.line(grid_img, (0, y+h_s), (width-1, y+h_s), (255, 0, 0), 1)
    for x in range(0, width, line_interval):
        cv2.line(grid_img, (x+w_s, 0), (x+w_s, height-1), (255, 0, 0), 1)  

    save_image(grid_img, save_filepath)

def path_on_map(map, robot_pixels, save_filepath=None):
    ''' Draw available path on map'''
    kernel = create_circular_kernel(robot_pixels//2) 
    path_map_img = cv2.erode(map, kernel, iterations=1) 
    save_image(path_map_img, save_filepath)
    return path_map_img

# end of file