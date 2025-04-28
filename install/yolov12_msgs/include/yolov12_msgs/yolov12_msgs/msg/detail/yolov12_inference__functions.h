// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from yolov12_msgs:msg/Yolov12Inference.idl
// generated code does not contain a copyright notice

#ifndef YOLOV12_MSGS__MSG__DETAIL__YOLOV12_INFERENCE__FUNCTIONS_H_
#define YOLOV12_MSGS__MSG__DETAIL__YOLOV12_INFERENCE__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "yolov12_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "yolov12_msgs/msg/detail/yolov12_inference__struct.h"

/// Initialize msg/Yolov12Inference message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * yolov12_msgs__msg__Yolov12Inference
 * )) before or use
 * yolov12_msgs__msg__Yolov12Inference__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_yolov12_msgs
bool
yolov12_msgs__msg__Yolov12Inference__init(yolov12_msgs__msg__Yolov12Inference * msg);

/// Finalize msg/Yolov12Inference message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_yolov12_msgs
void
yolov12_msgs__msg__Yolov12Inference__fini(yolov12_msgs__msg__Yolov12Inference * msg);

/// Create msg/Yolov12Inference message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * yolov12_msgs__msg__Yolov12Inference__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_yolov12_msgs
yolov12_msgs__msg__Yolov12Inference *
yolov12_msgs__msg__Yolov12Inference__create();

/// Destroy msg/Yolov12Inference message.
/**
 * It calls
 * yolov12_msgs__msg__Yolov12Inference__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_yolov12_msgs
void
yolov12_msgs__msg__Yolov12Inference__destroy(yolov12_msgs__msg__Yolov12Inference * msg);

/// Check for msg/Yolov12Inference message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_yolov12_msgs
bool
yolov12_msgs__msg__Yolov12Inference__are_equal(const yolov12_msgs__msg__Yolov12Inference * lhs, const yolov12_msgs__msg__Yolov12Inference * rhs);

/// Copy a msg/Yolov12Inference message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_yolov12_msgs
bool
yolov12_msgs__msg__Yolov12Inference__copy(
  const yolov12_msgs__msg__Yolov12Inference * input,
  yolov12_msgs__msg__Yolov12Inference * output);

/// Initialize array of msg/Yolov12Inference messages.
/**
 * It allocates the memory for the number of elements and calls
 * yolov12_msgs__msg__Yolov12Inference__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_yolov12_msgs
bool
yolov12_msgs__msg__Yolov12Inference__Sequence__init(yolov12_msgs__msg__Yolov12Inference__Sequence * array, size_t size);

/// Finalize array of msg/Yolov12Inference messages.
/**
 * It calls
 * yolov12_msgs__msg__Yolov12Inference__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_yolov12_msgs
void
yolov12_msgs__msg__Yolov12Inference__Sequence__fini(yolov12_msgs__msg__Yolov12Inference__Sequence * array);

/// Create array of msg/Yolov12Inference messages.
/**
 * It allocates the memory for the array and calls
 * yolov12_msgs__msg__Yolov12Inference__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_yolov12_msgs
yolov12_msgs__msg__Yolov12Inference__Sequence *
yolov12_msgs__msg__Yolov12Inference__Sequence__create(size_t size);

/// Destroy array of msg/Yolov12Inference messages.
/**
 * It calls
 * yolov12_msgs__msg__Yolov12Inference__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_yolov12_msgs
void
yolov12_msgs__msg__Yolov12Inference__Sequence__destroy(yolov12_msgs__msg__Yolov12Inference__Sequence * array);

/// Check for msg/Yolov12Inference message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_yolov12_msgs
bool
yolov12_msgs__msg__Yolov12Inference__Sequence__are_equal(const yolov12_msgs__msg__Yolov12Inference__Sequence * lhs, const yolov12_msgs__msg__Yolov12Inference__Sequence * rhs);

/// Copy an array of msg/Yolov12Inference messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_yolov12_msgs
bool
yolov12_msgs__msg__Yolov12Inference__Sequence__copy(
  const yolov12_msgs__msg__Yolov12Inference__Sequence * input,
  yolov12_msgs__msg__Yolov12Inference__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // YOLOV12_MSGS__MSG__DETAIL__YOLOV12_INFERENCE__FUNCTIONS_H_
