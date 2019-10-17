# BasicNeighborhoodFeatures

##test pca
1. 格网大小5m (越亮代表平面性越显著)

![image](../testimage/pcatest1_5m.png)
2. 格网大小2m

![image](../testimage/pcatest2_2m.png)
##test featureDetection
1.  option.ratioMax = 0.2;
    option.radiusFeatureCalculation = 3;
    option.minPtNum = 50;
    option.radiusNonMax = 3;

    ![image](../testimage/keypoint_1.png)
2.  option.ratioMax = 0.4;
    option.radiusFeatureCalculation = 3;
    option.minPtNum = 50;
    option.radiusNonMax = 3;
    
    ![image](../testimage/keypoint_2.png)