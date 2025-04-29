// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from yolov12_msgs:msg/InferenceResult.idl
// generated code does not contain a copyright notice

#ifndef YOLOV12_MSGS__MSG__DETAIL__INFERENCE_RESULT__BUILDER_HPP_
#define YOLOV12_MSGS__MSG__DETAIL__INFERENCE_RESULT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "yolov12_msgs/msg/detail/inference_result__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace yolov12_msgs
{

namespace msg
{

namespace builder
{

class Init_InferenceResult_right
{
public:
  explicit Init_InferenceResult_right(::yolov12_msgs::msg::InferenceResult & msg)
  : msg_(msg)
  {}
  ::yolov12_msgs::msg::InferenceResult right(::yolov12_msgs::msg::InferenceResult::_right_type arg)
  {
    msg_.right = std::move(arg);
    return std::move(msg_);
  }

private:
  ::yolov12_msgs::msg::InferenceResult msg_;
};

class Init_InferenceResult_bottom
{
public:
  explicit Init_InferenceResult_bottom(::yolov12_msgs::msg::InferenceResult & msg)
  : msg_(msg)
  {}
  Init_InferenceResult_right bottom(::yolov12_msgs::msg::InferenceResult::_bottom_type arg)
  {
    msg_.bottom = std::move(arg);
    return Init_InferenceResult_right(msg_);
  }

private:
  ::yolov12_msgs::msg::InferenceResult msg_;
};

class Init_InferenceResult_left
{
public:
  explicit Init_InferenceResult_left(::yolov12_msgs::msg::InferenceResult & msg)
  : msg_(msg)
  {}
  Init_InferenceResult_bottom left(::yolov12_msgs::msg::InferenceResult::_left_type arg)
  {
    msg_.left = std::move(arg);
    return Init_InferenceResult_bottom(msg_);
  }

private:
  ::yolov12_msgs::msg::InferenceResult msg_;
};

class Init_InferenceResult_top
{
public:
  explicit Init_InferenceResult_top(::yolov12_msgs::msg::InferenceResult & msg)
  : msg_(msg)
  {}
  Init_InferenceResult_left top(::yolov12_msgs::msg::InferenceResult::_top_type arg)
  {
    msg_.top = std::move(arg);
    return Init_InferenceResult_left(msg_);
  }

private:
  ::yolov12_msgs::msg::InferenceResult msg_;
};

class Init_InferenceResult_confidence
{
public:
  explicit Init_InferenceResult_confidence(::yolov12_msgs::msg::InferenceResult & msg)
  : msg_(msg)
  {}
  Init_InferenceResult_top confidence(::yolov12_msgs::msg::InferenceResult::_confidence_type arg)
  {
    msg_.confidence = std::move(arg);
    return Init_InferenceResult_top(msg_);
  }

private:
  ::yolov12_msgs::msg::InferenceResult msg_;
};

class Init_InferenceResult_class_name
{
public:
  Init_InferenceResult_class_name()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_InferenceResult_confidence class_name(::yolov12_msgs::msg::InferenceResult::_class_name_type arg)
  {
    msg_.class_name = std::move(arg);
    return Init_InferenceResult_confidence(msg_);
  }

private:
  ::yolov12_msgs::msg::InferenceResult msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::yolov12_msgs::msg::InferenceResult>()
{
  return yolov12_msgs::msg::builder::Init_InferenceResult_class_name();
}

}  // namespace yolov12_msgs

#endif  // YOLOV12_MSGS__MSG__DETAIL__INFERENCE_RESULT__BUILDER_HPP_
