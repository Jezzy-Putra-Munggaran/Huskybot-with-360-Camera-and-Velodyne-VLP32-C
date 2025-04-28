// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from yolov12_msgs:msg/Yolov12Inference.idl
// generated code does not contain a copyright notice

#ifndef YOLOV12_MSGS__MSG__DETAIL__YOLOV12_INFERENCE__STRUCT_HPP_
#define YOLOV12_MSGS__MSG__DETAIL__YOLOV12_INFERENCE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"
// Member 'yolov12_inference'
#include "yolov12_msgs/msg/detail/inference_result__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__yolov12_msgs__msg__Yolov12Inference __attribute__((deprecated))
#else
# define DEPRECATED__yolov12_msgs__msg__Yolov12Inference __declspec(deprecated)
#endif

namespace yolov12_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Yolov12Inference_
{
  using Type = Yolov12Inference_<ContainerAllocator>;

  explicit Yolov12Inference_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->camera_name = "";
    }
  }

  explicit Yolov12Inference_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    camera_name(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->camera_name = "";
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _yolov12_inference_type =
    std::vector<yolov12_msgs::msg::InferenceResult_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<yolov12_msgs::msg::InferenceResult_<ContainerAllocator>>>;
  _yolov12_inference_type yolov12_inference;
  using _camera_name_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _camera_name_type camera_name;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__yolov12_inference(
    const std::vector<yolov12_msgs::msg::InferenceResult_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<yolov12_msgs::msg::InferenceResult_<ContainerAllocator>>> & _arg)
  {
    this->yolov12_inference = _arg;
    return *this;
  }
  Type & set__camera_name(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->camera_name = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    yolov12_msgs::msg::Yolov12Inference_<ContainerAllocator> *;
  using ConstRawPtr =
    const yolov12_msgs::msg::Yolov12Inference_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<yolov12_msgs::msg::Yolov12Inference_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<yolov12_msgs::msg::Yolov12Inference_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      yolov12_msgs::msg::Yolov12Inference_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<yolov12_msgs::msg::Yolov12Inference_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      yolov12_msgs::msg::Yolov12Inference_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<yolov12_msgs::msg::Yolov12Inference_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<yolov12_msgs::msg::Yolov12Inference_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<yolov12_msgs::msg::Yolov12Inference_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__yolov12_msgs__msg__Yolov12Inference
    std::shared_ptr<yolov12_msgs::msg::Yolov12Inference_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__yolov12_msgs__msg__Yolov12Inference
    std::shared_ptr<yolov12_msgs::msg::Yolov12Inference_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Yolov12Inference_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->yolov12_inference != other.yolov12_inference) {
      return false;
    }
    if (this->camera_name != other.camera_name) {
      return false;
    }
    return true;
  }
  bool operator!=(const Yolov12Inference_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Yolov12Inference_

// alias to use template instance with default allocator
using Yolov12Inference =
  yolov12_msgs::msg::Yolov12Inference_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace yolov12_msgs

#endif  // YOLOV12_MSGS__MSG__DETAIL__YOLOV12_INFERENCE__STRUCT_HPP_
