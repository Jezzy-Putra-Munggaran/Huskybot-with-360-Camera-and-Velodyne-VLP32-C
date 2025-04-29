// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from yolov12_msgs:msg/InferenceResult.idl
// generated code does not contain a copyright notice

#ifndef YOLOV12_MSGS__MSG__DETAIL__INFERENCE_RESULT__STRUCT_H_
#define YOLOV12_MSGS__MSG__DETAIL__INFERENCE_RESULT__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'class_name'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/InferenceResult in the package yolov12_msgs.
typedef struct yolov12_msgs__msg__InferenceResult
{
  /// Nama kelas objek hasil deteksi (misal: "person", "car", "dog", dst)
  rosidl_runtime_c__String class_name;
  /// Skor confidence deteksi (0.0 - 1.0)
  float confidence;
  /// Koordinat atas bounding box (pixel, biasanya y1)
  int64_t top;
  /// Koordinat kiri bounding box (pixel, biasanya x1)
  int64_t left;
  /// Koordinat bawah bounding box (pixel, biasanya y2)
  int64_t bottom;
  /// Koordinat kanan bounding box (pixel, biasanya x2)
  int64_t right;
} yolov12_msgs__msg__InferenceResult;

// Struct for a sequence of yolov12_msgs__msg__InferenceResult.
typedef struct yolov12_msgs__msg__InferenceResult__Sequence
{
  yolov12_msgs__msg__InferenceResult * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} yolov12_msgs__msg__InferenceResult__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // YOLOV12_MSGS__MSG__DETAIL__INFERENCE_RESULT__STRUCT_H_
