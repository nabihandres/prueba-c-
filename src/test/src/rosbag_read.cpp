 #include <rosbag/bag.h>
    #include <rosbag/view.h>
    #include <std_msgs/Int32.h>

    rosbag::Bag bag;
    bag.open("test.bag");  // BagMode is Read by default

    for(rosbag::MessageInstance const m: rosbag::View(bag))
    {
      std_msgs::Int32::ConstPtr i = m.instantiate<std_msgs::Int32>();
      if (i != nullptr)
        std::cout << i->data << std::endl;
    }

    bag.close();