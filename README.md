Introduction  
============

This project is to test several OpenCV APIs to deal with map image in fleet management system


Prerequsities  
==============  

Ubuntu 16.04, Anaconda/Miniconda, and etc  


How to setup  
============    

```  
	$ conda create -n opencv python=3  
	$ conda activate opencv
	$ cd opencv_experiments  
	(opencv) opencv_experiments$ pip install -r requirements.txt  
```  

How to use  
==========  

```  
	(opencv) opencv_experiments$ python map_img_process.py  
	// Go to the debug folder to see results 
```  

Details of map_img_process.py  
=============================  
  
01. Change the original map, which comes from SLAM, into a grey image  
  
02. Turn unknown area on the map into black which means a robot cannot go there  
  
03. Make it binary, 0 or 255  
  
04. Rotate map into the best position automatically based on the most frequent angle of lines found by OpenCV  
  
05. Make it binary 0 or 255 again because rotation causes pixels to have a color between 1 and 254  
  
06. Generate path map or hit map based on robot size which is the vertical or horizontal length of a robot  
  
07. Remove small particles which are disconnected from the main area where bots are moving  
  
07. Find the optimal starting position to generate a grid/FM map in order to avoid placing available pixels on a grid line.   
  
08. Draw grid lines on the path map for debugging (Optional)  
  
09. Create a fleet management map using the path map and grid starting position  

    -if a grid cell is occupied by 255 with a small percentage or below like 4%, then the cell becomes an obstacle  
    -if all available pixels are placed near the grid lines, then the cell becomes an obstacle  
  
10. Store grid cell information such as border connection, occupancy rate and the location in the original map for the use in fleet management  
  
11. Check each cell of the FM map again if it's feasible. The aim of this is to remove those cells which are connected to each other in the grid system but are NOT connected in the original map.  
  
12. Remove disconnected particles again  
  
13. Get rid of outer and blocked cells in order to minimize the size of the FM map finally  
  
  
It takes a lot of steps shown above, but only 2 seconds is enough for the whole processing on average. For reference, 4 seconds is the longest time when it has a very difficult map to process.  
  
  
Reference  
==========  

- [How to rotate 2d point in python](https://gist.github.com/LyleScott/e36e08bfb23b1f87af68c9051f985302) 
- [sknw.py](https://github.com/snakers4/spacenet-three-topcoder/blob/master/sknw.py)  