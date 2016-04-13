#include <iostream>
#include <string>
#include <vector>
#include <ros/ros.h>
#include <toaster_msgs/Fact.h>
#include <toaster_msgs/FactList.h>


int main(int argc, char** argv) {
   
    ros::init(argc, argv, "move3d_facts");
    ros::NodeHandle node;
    ros::Rate loop_rate(30);

    // Publishing
    ros::Publisher facts_pub = node.advertise<toaster_msgs::FactList>("move3d_facts/factList", 1000);

    bool isReachableBy, isVisibleBy, isOn, isIn;
    toaster_msgs::FactList factList_msg;
    toaster_msgs::Fact fact_msg;
    
    /************************/
    /* Start of the Ros loop*/
    /************************/

    while (node.ok()) {
      //Get which facts we should compute from parameters   
      node.getParam("/computedFacts/isReachableBy", isReachableBy);
      node.getParam("/computedFacts/isVisibleBy", isVisibleBy);
      node.getParam("/computedFacts/isOn", isOn);
      node.getParam("/computedFacts/isIn", isIn);
        
    
        if(isReachableBy){

           //Test fact
           fact_msg.property = "isReachableBy";
           fact_msg.subjectId = "toto";
           fact_msg.targetId = "tata";
           fact_msg.confidence = 1.0;
           fact_msg.doubleValue = 0.95;
           fact_msg.valueType = 1;
           fact_msg.factObservability = 1.0;

           factList_msg.factList.push_back(fact_msg);
        }
        
        if(isVisibleBy){

           //Test fact
           fact_msg.property = "isVisibleBy";
           fact_msg.subjectId = "toto";
           fact_msg.targetId = "tata";
           fact_msg.confidence = 1.0;
           fact_msg.doubleValue = 0.95;
           fact_msg.valueType = 1;
           fact_msg.factObservability = 1.0;

           factList_msg.factList.push_back(fact_msg);
        }
        
        if(isOn){

           //Test fact
           fact_msg.property = "isOn";
           fact_msg.subjectId = "toto";
           fact_msg.targetId = "tata";
           fact_msg.confidence = 1.0;
           fact_msg.doubleValue = 0.95;
           fact_msg.valueType = 1;
           fact_msg.factObservability = 1.0;

           factList_msg.factList.push_back(fact_msg);
        }
        
        if(isIn){

           //Test fact
           fact_msg.property = "isIn";
           fact_msg.subjectId = "toto";
           fact_msg.targetId = "tata";
           fact_msg.confidence = 1.0;
           fact_msg.doubleValue = 0.95;
           fact_msg.valueType = 1;
           fact_msg.factObservability = 1.0;

           factList_msg.factList.push_back(fact_msg);
        }

        facts_pub.publish(factList_msg);
        ros::spinOnce();

        loop_rate.sleep();
    }
    return 0;
}
