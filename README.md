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

Before launching the service, config.py has to be updated in your environment.  

```  
	(opencv) opencv_experiments$ python map_img_process.py  
```  

Reference  
==========  

- [How to rotate 2d point in python](https://gist.github.com/LyleScott/e36e08bfb23b1f87af68c9051f985302) 
- [sknw.py](https://github.com/snakers4/spacenet-three-topcoder/blob/master/sknw.py)  