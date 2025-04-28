// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from yolov12_msgs:msg/Yolov12Inference.idl
// generated code does not contain a copyright notice

#ifndef YOLOV12_MSGS__MSG__DETAIL__YOLOV12_INFERENCE__BUILDER_HPP_
#define YOLOV12_MSGS__MSG__DETAIL__YOLOV12_INFERENCE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "yolov12_msgs/msg/detail/yolov12_inference__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace yolov12_msgs
{

namespace msg
{

namespace builder
{

class Init_Yolov12Inference_camera_name
{
public:
  explicit Init_Yolov12Inference_camera_name(::yolov12_msgs::msg::Yolov12Inference & msg)
  : msg_(msg)
  {}
  ::yolov12_msgs::msg::Yolov12Inference camera_name(::yolov12_msgs::msg::Yolov12Inference::_camera_name_type arg)
  {
    msg_.camera_name = std::move(arg);
    return std::move(msg_);
  }

private:
  ::yolov12_msgs::msg::Yolov12Inference msg_;
};

class Init_Yolov12Inference_yolov12_inference
{
public:
  explicit Init_Yolov12Inference_yolov12_inference(::yolov12_msgs::msg::Yolov12Inference & msg)
  : msg_(msg)
  {}
  Init_Yolov12Inference_camera_name yolov12_inference(::yolov12_msgs::msg::Yolov12Inference::_yolov12_inference_type arg)
  {
    msg_.yolov12_inference = std::move(arg);
    return Init_Yolov12Inference_camera_name(msg_);
  }

private:
  ::yolov12_msgs::msg::Yolov12Inference msg_;
};

class Init_Yolov12Inference_header
{
public:
  Init_Yolov12Inference_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Yolov12Inference_yolov12_inference header(::yolov12_msgs::msg::Yolov12Inference::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_Yolov12Inference_yolov12_inference(msg_);
  }

private:
  ::yolov12_msgs::msg::Yolov12Inference msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::yolov12_msgs::msg::Yolov12Inference>()
{
  return yolov12_msgs::msg::builder::Init_Yolov12Inference_header();
}

}  // namespace yolov12_msgs

#endif  // YOLOV12_MSGS__MSG__DETAIL__YOLOV12_INFERENCE__BUILDER_HPP_
