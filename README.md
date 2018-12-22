# Blindfind3 Repository
This is the source file of Blindfind3 software. It is still in development.

## Dependency
To test/run this software, one need to have the following packages 

pocketsphinx 

sphinxbase 

OpenCV3

Festival

MIT LCM

CUDA cudnn

caffe-GPU

DBoW3
 
iSAM (Incremental Smmothing and Mapping)

google proto buffer

## Documentation
Please check each subfolder to check the documentation for each class/module.

Currently, we use source code in **cmake_src** folder. Check the CMakeLists.txt for more information.

## Current Problems
Currently, track map module is not added to blindfind3 project, which means it has no connection with control center.

### Tutorial for track map module
To use track map API, read the code in trackGroundTest.cpp in the trackMap folder. It uses the IDOL dataset to generate four tracks using the current track map class.

## Different feature extract methods in CNNExtractor.cpp
We currently use **VGG16** model trained on **Place365** dataset. Then for blindfind3 project, we extract feature from **pool3**, which can be set as an attribute of extractor.
* To use the feature extractor for blindfind3 project.
```
cv::Mat prevConvFeature = cv::Mat::zeros(lastConvLayer->height()*lastConvLayer->width(), lastConvLayer->channels(), CV_32FC1);    
float* fpr = (float*)(prevConvFeature.data);    
int pointer=0;    
for(size_t i=0; i<lastConvLayer->height(); i++)        
    for(size_t j=0; j<lastConvLayer->width(); j++)            
        for(size_t k=0; k<lastConvLayer->channels(); k++)            
        {   
            memcpy(&(fpr[pointer]), &lastConvLayer->mutable_cpu_data()[lastConvLayer->offset(0,k,j,i)], sizeof(float)/sizeof(char));                
            pointer+=1;            
        
        }
```
* For current trackMap module, we use IDOL dataset, and the Bat of Words database uses feature from **pool5**, so for the feature extractor, this code should be used:
```
 cv::Mat prevConvFeature(lastConvLayer->channels(), lastConvLayer->height()*lastConvLayer->width(), CV_32FC1, lastConvLayer->mutable_cpu_data());
```
* If PCA is used, uncomment the following code in the cpp file:
```
cv::PCA pca(prevConvFeature, cv::Mat(), CV_PCA_DATA_AS_COL, 256);    
cv::Mat convFeature = pca.project(prevConvFeature);
```
> Remember that for different feature map, different database should be used. 

## Current database name for different feature extractor:
* For blindfind3, use `/home/blindfind/Documents/Blindfind3/data/CNNDatabase_BH_Pool3_.yml.gz`
* For current IDOL dataset that test the track map module use `/home/blindfind/Documents/Blindfind3/CNNdatabase.yml.gz`

## About current serialization of the intermediate status of tarckMap
Currently, we use proto buffer from google to serialize the intermediate status of the trackmap so that it can restore the previous state when continue annotating.
See [`~/data/trackInfo.proto`](https://visionserver.lems.brown.edu/Blindfind/Blindfind3/blob/dev/data/protobuf/trackInfo.proto) for more inforamtion. 
To successfully use the protocal buffer, you need to intall the compiler for the file.

* The protocal buffer is no longer able to be used, please write a database to store intermediate status.

