// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from yolov12_msgs:msg/Yolov12Inference.idl
// generated code does not contain a copyright notice

#ifndef YOLOV12_MSGS__MSG__DETAIL__YOLOV12_INFERENCE__STRUCT_H_
#define YOLOV12_MSGS__MSG__DETAIL__YOLOV12_INFERENCE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'yolov12_inference'
#include "yolov12_msgs/msg/detail/inference_result__struct.h"
// Member 'camera_name'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/Yolov12Inference in the package yolov12_msgs.
typedef struct yolov12_msgs__msg__Yolov12Inference
{
  /// Header ROS2 (timestamp dan frame_id, untuk sinkronisasi waktu dan referensi frame)
  std_msgs__msg__Header header;
  /// Array hasil deteksi (list bounding box dan class, satu frame bisa banyak deteksi)
  yolov12_msgs__msg__InferenceResult__Sequence yolov12_inference;
  /// Nama kamera sumber deteksi (misal: "camera_front", "panorama", dsb)
  rosidl_runtime_c__String camera_name;
} yolov12_msgs__msg__Yolov12Inference;

// Struct for a sequence of yolov12_msgs__msg__Yolov12Inference.
typedef struct yolov12_msgs__msg__Yolov12Inference__Sequence
{
  yolov12_msgs__msg__Yolov12Inference * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} yolov12_msgs__msg__Yolov12Inference__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // YOLOV12_MSGS__MSG__DETAIL__YOLOV12_INFERENCE__STRUCT_H_
