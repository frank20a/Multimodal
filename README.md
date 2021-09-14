# Multimodal
Semester project for "Computational Geometry &amp; 3D Modeling Applications" class.

## Project task
Use image processing and computational geometry technics to process data received by autonomus vehicles. These technics will be applied in Image-space and Lidar-space data respectively. 


## Image-Space

### Datasets
For image-space processing we are using data from the Carla and Kitti datasets,

### Tasks
1. Convert image(s) to grayscale
2. Use appropriate filters to elliminate noise
3. Detect lines in the image and then, using the Hough Line Transform, detect road lanes.

### Implementation
To achieve the tasks above, the OpenCV library is utilised. Images are converted to grayscaled and then processed by a combination of Gaussian and Median blur filters to reduce noise. Then, line detection is performed by means of the Canny method. Finally, after experimentation, we can provide appropriate attributes to the Probabilistic Hough Transform function implemented by OpenCV


## Lidar-Space

### Datasets
Datasets are provided by the University of Patras Computational Geometry Lab

### Tasks
1. Floor detection & seperation using z-axis threshold and RANSAC
2. Clustering for objects above ground. Classification of said clusters using protoypes for cars and pedestrians.
3. Triangulation of objects above ground

### Implementation
Floor detection is implemented with the requested methods. Clustering is performed with the DBSCAN algorithm. Classification is performed by comparing developed metrics and using a detection threshold. Metrics include:

- The average of the distance from its 3 nearest neighbours for each point of the comparing pointclouds
- Difference in volume of the boundin boxes of the comparing pointclouds
- Difference in point count of the comparing pointclouds
- Difference in height (z-distance from the lower and higher points) of the comparing pointclouds

Triangulation implementation is unsutisfactory but follows the following logic. Lidar pointclouds are height maps in spherical coordinates. As such, they can be triangulated by ignoring the <img src="https://render.githubusercontent.com/render/math?math=\phi">-axis as a 2D pointcloud using Delaunay triangulation. I implemented this by converting the points to spherical coordinates and subtracting <img src="https://render.githubusercontent.com/render/math?math=\phi_{avg}">. This "centers" the pointcloud in x-axis and I can use triangulation only in the yz-plane. The implemented triangulation algorithm is VERY slow and it doesn't work properly
