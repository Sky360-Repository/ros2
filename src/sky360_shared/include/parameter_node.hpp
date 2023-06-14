#pragma once
#ifndef __PARAMETER_NODE_H__
#define __PARAMETER_NODE_H__

#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/srv/list_parameters.hpp"
#include "rcl_interfaces/srv/get_parameters.hpp"
#include "rcl_interfaces/srv/set_parameters.hpp"

class ParameterNode
    : public rclcpp::Node
{
public:
    ParameterNode(const std::string &node_name)
        : Node(node_name)
    {
        list_parameters_service_ = this->create_service<rcl_interfaces::srv::ListParameters>(
            "list_parameters", std::bind(&ParameterNode::handle_list_parameters, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

        get_parameters_service_ = this->create_service<rcl_interfaces::srv::GetParameters>(
            "get_parameters", std::bind(&ParameterNode::handle_get_parameters, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

        set_parameters_service_ = this->create_service<rcl_interfaces::srv::SetParameters>(
            "set_parameters", std::bind(&ParameterNode::handle_set_parameters, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    }

protected:
    virtual std::vector<rcl_interfaces::msg::SetParametersResult> set_parameters_callback(const std::vector<rclcpp::Parameter>& parameters_to_set) = 0;

private:
    rclcpp::Service<rcl_interfaces::srv::ListParameters>::SharedPtr list_parameters_service_;
    rclcpp::Service<rcl_interfaces::srv::GetParameters>::SharedPtr get_parameters_service_;
    rclcpp::Service<rcl_interfaces::srv::SetParameters>::SharedPtr set_parameters_service_;

    void handle_list_parameters(
        const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<rcl_interfaces::srv::ListParameters::Request> request,
        std::shared_ptr<rcl_interfaces::srv::ListParameters::Response> response)
    {
        (void)request_header;

        auto parameters_and_prefixes = this->list_parameters({}, request->depth);
        response->result.names = parameters_and_prefixes.names;
        response->result.prefixes = {};
    }

    void handle_get_parameters(
        const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<rcl_interfaces::srv::GetParameters::Request> request,
        std::shared_ptr<rcl_interfaces::srv::GetParameters::Response> response)
    {
        (void)request_header;

        for (const auto &name : request->names)
        {
            rclcpp::Parameter parameter = this->get_parameter(name);
            rcl_interfaces::msg::Parameter msg_parameter = parameter.to_parameter_msg();
            response->values.push_back(msg_parameter.value);
        }
    }

    void handle_set_parameters(
        const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<rcl_interfaces::srv::SetParameters::Request> request,
        std::shared_ptr<rcl_interfaces::srv::SetParameters::Response> response)
    {
        (void)request_header;

        std::vector<rclcpp::Parameter> parameters_to_set;
        for (const auto &parameter : request->parameters)
        {
            parameters_to_set.emplace_back(rclcpp::Parameter::from_parameter_msg(parameter));
        }
        response->results = set_parameters_callback(parameters_to_set);
    }
};

#endif // __PARAMETER_NODE_H__