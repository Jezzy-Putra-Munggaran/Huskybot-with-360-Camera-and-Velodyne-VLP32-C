// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from yolov12_msgs:msg/Yolov12Inference.idl
// generated code does not contain a copyright notice

#ifndef YOLOV12_MSGS__MSG__DETAIL__YOLOV12_INFERENCE__TRAITS_HPP_
#define YOLOV12_MSGS__MSG__DETAIL__YOLOV12_INFERENCE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "yolov12_msgs/msg/detail/yolov12_inference__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'yolov12_inference'
#include "yolov12_msgs/msg/detail/inference_result__traits.hpp"

namespace yolov12_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const Yolov12Inference & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: yolov12_inference
  {
    if (msg.yolov12_inference.size() == 0) {
      out << "yolov12_inference: []";
    } else {
      out << "yolov12_inference: [";
      size_t pending_items = msg.yolov12_inference.size();
      for (auto item : msg.yolov12_inference) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: camera_name
  {
    out << "camera_name: ";
    rosidl_generator_traits::value_to_yaml(msg.camera_name, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Yolov12Inference & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
  }

  // member: yolov12_inference
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.yolov12_inference.size() == 0) {
      out << "yolov12_inference: []\n";
    } else {
      out << "yolov12_inference:\n";
      for (auto item : msg.yolov12_inference) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }

  // member: camera_name
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "camera_name: ";
    rosidl_generator_traits::value_to_yaml(msg.camera_name, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Yolov12Inference & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace yolov12_msgs

namespace rosidl_generator_traits
{

[[deprecated("use yolov12_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const yolov12_msgs::msg::Yolov12Inference & msg,
  std::ostream & out, size_t indentation = 0)
{
  yolov12_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use yolov12_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const yolov12_msgs::msg::Yolov12Inference & msg)
{
  return yolov12_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<yolov12_msgs::msg::Yolov12Inference>()
{
  return "yolov12_msgs::msg::Yolov12Inference";
}

template<>
inline const char * name<yolov12_msgs::msg::Yolov12Inference>()
{
  return "yolov12_msgs/msg/Yolov12Inference";
}

template<>
struct has_fixed_size<yolov12_msgs::msg::Yolov12Inference>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<yolov12_msgs::msg::Yolov12Inference>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<yolov12_msgs::msg::Yolov12Inference>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // YOLOV12_MSGS__MSG__DETAIL__YOLOV12_INFERENCE__TRAITS_HPP_
