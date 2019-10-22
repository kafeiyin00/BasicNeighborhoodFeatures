# BasicNeighborhoodFeatures

##test pca
1. 格网大小5m (越亮代表平面性越显著)

![image](https://github.com/kafeiyin00/BasicNeighborhoodFeatures/blob/master/testimage/pcatest1_5m.png)
2. 格网大小2m

![image](https://github.com/kafeiyin00/BasicNeighborhoodFeatures/blob/master/testimage/pcatest2_2m.png)
##test featureDetection
1.  option.ratioMax = 0.2;
    option.radiusFeatureCalculation = 3;
    option.minPtNum = 50;
    option.radiusNonMax = 3;

    ![image](https://github.com/kafeiyin00/BasicNeighborhoodFeatures/blob/master/testimage/keypoint1.png)
2.  option.ratioMax = 0.4;
    optio   n.radiusFeatureCalculation = 3;
    option.minPtNum = 50;
    option.radiusNonMax = 3;
    
    ![image](https://github.com/kafeiyin00/BasicNeighborhoodFeatures/blob/master/testimage/keypoint2.png)
    
3. demPointcloud test
   option.ratioMax = 0.9;
   option.radiusFeatureCalculation = 10;
   option.minPtNum = 20;
   option.radiusNonMax = 10;
   
   ![image](https://github.com/kafeiyin00/BasicNeighborhoodFeatures/blob/master/testimage/keypoint3.png)