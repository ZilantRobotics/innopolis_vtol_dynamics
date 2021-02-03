#include <iostream>
#include <cstdlib>
#include <unistd.h>
#include <uavcan/uavcan.hpp>


#include <uavcan/protocol/NodeStatus.hpp>
#include <uavcan/protocol/debug/KeyValue.hpp>


/**
 * Platform dependent function and memory pool
 */
extern uavcan::ICanDriver& getCanDriver();
extern uavcan::ISystemClock& getSystemClock();
constexpr unsigned NodeMemoryPoolSize = 16384;
typedef uavcan::Node<NodeMemoryPoolSize> Node;

static Node& getNode(){
    static Node node(getCanDriver(), getSystemClock());
    return node;
}

int main(int argc, const char** argv){
    if (argc < 2){
        std::cerr << "Usage: " << argv[0] << " <node-id>" << std::endl;
        return 1;
    }

    const int self_node_id = std::stoi(argv[1]);

    /*
     * Node initialization.
     * Node ID and name are required; otherwise, the node will refuse to start.
     * Version info is optional.
     */
    auto& node = getNode();
    node.setNodeID(self_node_id);
    node.setName("org.uavcan.tutorial.init");
    uavcan::protocol::SoftwareVersion sw_version;
    sw_version.major = 1;
    node.setSoftwareVersion(sw_version);
    uavcan::protocol::HardwareVersion hw_version;
    hw_version.major = 1;
    node.setHardwareVersion(hw_version);

    /*
     * Start the node.
     * All returnable error codes are listed in the header file uavcan/error.hpp.
     */
    const int node_start_res = node.start();
    if (node_start_res < 0){
        throw std::runtime_error("Failed to start the node; error: " + std::to_string(node_start_res));
    }

    uavcan::Subscriber<uavcan::protocol::NodeStatus> node_status_sub(node);
    const int node_status_start_res = node_status_sub.start(
        [&](const uavcan::ReceivedDataStructure<uavcan::protocol::NodeStatus>& msg)
        {
            std::cout << msg << std::endl;
        });

    uavcan::Publisher<uavcan::protocol::debug::KeyValue> kv_pub(node);
    const int kv_pub_init_res = kv_pub.init();
    if (kv_pub_init_res < 0){
        throw std::runtime_error("Failed to start the publisher; error: " + std::to_string(kv_pub_init_res));
    }

    node.setModeOperational();

    std::cout << "Hello world! " << node.getNodeID().get() + 0 << std::endl;

    while (true){
        const int res = node.spin(uavcan::MonotonicDuration::fromMSec(1000));
        if (res < 0){
            std::cerr << "Transient failure: " << res << std::endl;
        }

        const float random = std::rand() / float(RAND_MAX);
        if (random < 0.7){
            node.setHealthOk();
        }
        else if (random < 0.9){
            node.setHealthWarning();
        }
        else{
            node.setHealthError();
        }

        uavcan::protocol::debug::KeyValue kv_msg;  // Always zero initialized
        kv_msg.value = std::rand() / float(RAND_MAX);
        kv_msg.key = "a";   // "a"
        kv_msg.key += "b";  // "ab"
        kv_msg.key += "c";  // "abc"
        const int pub_res = kv_pub.broadcast(kv_msg);
        if (pub_res < 0){
            std::cerr << "KV publication failure: " << pub_res << std::endl;
        }else{
            std::cout << "msg has been published" << std::endl;
        }
    }
}