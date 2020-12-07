#include <serow_utils/serow_utils.h>

int main( int argc, char** argv )
{
    ros::init(argc, argv, "serow_utils");
    ros::NodeHandle n;
    if(!ros::master::check())
    {
        cerr<<"Could not contact master!\n Quitting... "<<endl;
        return -1;
    }
    serow_utils* su = new serow_utils();
    su->connect(n);
    //ros::spin();
    su->run();
}