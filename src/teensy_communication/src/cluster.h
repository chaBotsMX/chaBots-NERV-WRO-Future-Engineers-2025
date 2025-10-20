#include "utils.h"
const int CLUSTER_JUMP__THRESHOLD = 15; 
void findClusters(){
    for (auto& point : lidarMSG) {
        float jump = getEuclideanDistance(previous_point.angle, previous_point.mag, point.angle, point.mag);

        if (jump > CLUSTER_JUMP__THRESHOLD) {
            int size = 1;
            
            while(jump < CLUSTER_JUMP__THRESHOLD && point  < lidarMSG.size()){
                size++;
                
                jump = getEuclideanDistance(previous_point.angle, previous_point.mag, lidarMSG[i].angle, lidarMSG[i].mag);
            }
        }
    }
}