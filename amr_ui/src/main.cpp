#include "amr_ui/handler.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "amr_ui_handler");
    ros::NodeHandle nh_("~");

    AMR_UI_Handler AmrUiHandler(nh_);

    std::thread thProcess(&AMR_UI_Handler::processHandler, &AmrUiHandler);

    ros::spin();

    signal(SIGINT, mySigintHandler);
    ros::waitForShutdown();

    if(!ros::ok())
    {
        std::cout << "[STATUS]shut down amr_ui_handler!" << "\n";
    }

    return 0;
}