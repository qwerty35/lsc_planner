#include <dynamic_debugger.hpp>

int main(int argc, char* argv[]) {
    ros::init (argc, argv, "multi_sync_debugger_node");
    ros::NodeHandle nh( "~" );

    DynamicDebugger dynamic_debugger(nh);
    dynamic_debugger.initialize();

    int planner_seq = 1;
    bool publish_lsc_contruction = true;
    ros::Rate rate(50);
    ros::Duration wait_term(0.5);
    while (ros::ok()) {
//        if(publish_lsc_contruction){
//            // LSC construction
//            dynamic_debugger.publishLSCConstruction(planner_seq);
//            if(dynamic_debugger.isFinished()){
//                dynamic_debugger.reset();
//                publish_lsc_contruction = not publish_lsc_contruction;
//                wait_term.sleep();
//            }
//        }
//        else{
            dynamic_debugger.publishTrajOptimization(planner_seq);
            if(dynamic_debugger.isFinished()){
                dynamic_debugger.reset();
                dynamic_debugger.doStep();
                planner_seq++;
                publish_lsc_contruction = not publish_lsc_contruction;
                wait_term.sleep();
            }
//        }
        rate.sleep();
    }

    return 0;
}
