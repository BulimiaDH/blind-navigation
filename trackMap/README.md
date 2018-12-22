## Current configuration of trackMap
To use this spatial constraint on the IDOL dataset, two threshold must be set, one is the distance threshold, along x and y axis, the other is the rotation angle threshold.
The two threshold hold can be seen in the [`trackMap.h`](https://visionserver.lems.brown.edu/Blindfind/Blindfind3/blob/dev/trackMap/trackMap.h)
```
double _distThreshold;
double _angleThreshold;
void setThreshold(double distThreshold, double angleThreshold){this->_distThreshold=distThreshold; this->_angleThreshold=angleThreshold;};
```

Currently, there are two methods for the angle estimation:
* The first is intensity based estimation, which is set as an internal method of trackmap class.
* The other method is rotation estimation method. Which is an external class set as internal attribute of trackmap class.
In the `findConstraints` method of trackMap class, you can define different angle estimation method.