#include "sh_find_client/find_client.hpp"

#include <sh_common/ros_names.hpp>

#define NODE_NAME_PARAM_NAME sh::names::params::FIND_SERVER_NODE_NAME

using namespace std::chrono_literals;

namespace sh {

FindClient::FindClient() :
    FindBaseNode("find_client")
{
}
FindClient::~FindClient()
{
}

}

int main(int argc, char** argv) 
{
    // Create our node
    rclcpp::init(argc, argv);
    auto node = std::make_shared<sh::FindClient>();

    // Get the expected name of the node which is the peer to the FIND server
    node->declare_parameter<std::string>(NODE_NAME_PARAM_NAME);
    const std::string server_node_name = node->get_parameter(NODE_NAME_PARAM_NAME).as_string();

    // Create a client to get its parameters and populate the request
    auto get_parameters_scl = node->create_client<rcl_interfaces::srv::GetParameters>(
        std::string(node->get_namespace()) + "/" + server_node_name + "/get_parameters"
    );
    auto request = std::make_shared<rcl_interfaces::srv::GetParameters::Request>();
    request->names = {
        sh::names::params::FIND_HTTP_VERSION, 
        sh::names::params::FIND_SERVER_HOST, 
        sh::names::params::FIND_SERVER_PORT
    };

    // Wait for the service, though it should already exist
    while (!get_parameters_scl->wait_for_service(1s))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
            return 1;
        }
        RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Service not available, waiting again...");
    }

    // Place the request and capture the result
    auto result_future = get_parameters_scl->async_send_request(request);
    assert(rclcpp::spin_until_future_complete(node, result_future) == rclcpp::FutureReturnCode::SUCCESS);
    auto result = result_future.get();

    // Capture the values of the parameters
    assert(3 == result->values.size());
    assert(4 == result->values[0].type);
    const std::string http_version = result->values[0].string_value;
    assert(4 == result->values[1].type);
    const std::string server_host = result->values[1].string_value;
    assert(2 == result->values[2].type);
    const int server_port = result->values[2].integer_value;

    // Init communication with the FIND server and spin the node
    assert(node->init_communication(http_version, server_host, server_port));
    rclcpp::spin(node);

    return 0;
}
