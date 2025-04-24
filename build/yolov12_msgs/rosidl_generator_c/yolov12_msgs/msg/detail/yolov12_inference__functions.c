// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from yolov12_msgs:msg/Yolov12Inference.idl
// generated code does not contain a copyright notice
#include "yolov12_msgs/msg/detail/yolov12_inference__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `yolov12_inference`
#include "yolov12_msgs/msg/detail/inference_result__functions.h"
// Member `camera_name`
#include "rosidl_runtime_c/string_functions.h"

bool
yolov12_msgs__msg__Yolov12Inference__init(yolov12_msgs__msg__Yolov12Inference * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    yolov12_msgs__msg__Yolov12Inference__fini(msg);
    return false;
  }
  // yolov12_inference
  if (!yolov12_msgs__msg__InferenceResult__Sequence__init(&msg->yolov12_inference, 0)) {
    yolov12_msgs__msg__Yolov12Inference__fini(msg);
    return false;
  }
  // camera_name
  if (!rosidl_runtime_c__String__init(&msg->camera_name)) {
    yolov12_msgs__msg__Yolov12Inference__fini(msg);
    return false;
  }
  return true;
}

void
yolov12_msgs__msg__Yolov12Inference__fini(yolov12_msgs__msg__Yolov12Inference * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // yolov12_inference
  yolov12_msgs__msg__InferenceResult__Sequence__fini(&msg->yolov12_inference);
  // camera_name
  rosidl_runtime_c__String__fini(&msg->camera_name);
}

bool
yolov12_msgs__msg__Yolov12Inference__are_equal(const yolov12_msgs__msg__Yolov12Inference * lhs, const yolov12_msgs__msg__Yolov12Inference * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  // yolov12_inference
  if (!yolov12_msgs__msg__InferenceResult__Sequence__are_equal(
      &(lhs->yolov12_inference), &(rhs->yolov12_inference)))
  {
    return false;
  }
  // camera_name
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->camera_name), &(rhs->camera_name)))
  {
    return false;
  }
  return true;
}

bool
yolov12_msgs__msg__Yolov12Inference__copy(
  const yolov12_msgs__msg__Yolov12Inference * input,
  yolov12_msgs__msg__Yolov12Inference * output)
{
  if (!input || !output) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  // yolov12_inference
  if (!yolov12_msgs__msg__InferenceResult__Sequence__copy(
      &(input->yolov12_inference), &(output->yolov12_inference)))
  {
    return false;
  }
  // camera_name
  if (!rosidl_runtime_c__String__copy(
      &(input->camera_name), &(output->camera_name)))
  {
    return false;
  }
  return true;
}

yolov12_msgs__msg__Yolov12Inference *
yolov12_msgs__msg__Yolov12Inference__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  yolov12_msgs__msg__Yolov12Inference * msg = (yolov12_msgs__msg__Yolov12Inference *)allocator.allocate(sizeof(yolov12_msgs__msg__Yolov12Inference), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(yolov12_msgs__msg__Yolov12Inference));
  bool success = yolov12_msgs__msg__Yolov12Inference__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
yolov12_msgs__msg__Yolov12Inference__destroy(yolov12_msgs__msg__Yolov12Inference * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    yolov12_msgs__msg__Yolov12Inference__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
yolov12_msgs__msg__Yolov12Inference__Sequence__init(yolov12_msgs__msg__Yolov12Inference__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  yolov12_msgs__msg__Yolov12Inference * data = NULL;

  if (size) {
    data = (yolov12_msgs__msg__Yolov12Inference *)allocator.zero_allocate(size, sizeof(yolov12_msgs__msg__Yolov12Inference), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = yolov12_msgs__msg__Yolov12Inference__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        yolov12_msgs__msg__Yolov12Inference__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
yolov12_msgs__msg__Yolov12Inference__Sequence__fini(yolov12_msgs__msg__Yolov12Inference__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      yolov12_msgs__msg__Yolov12Inference__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

yolov12_msgs__msg__Yolov12Inference__Sequence *
yolov12_msgs__msg__Yolov12Inference__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  yolov12_msgs__msg__Yolov12Inference__Sequence * array = (yolov12_msgs__msg__Yolov12Inference__Sequence *)allocator.allocate(sizeof(yolov12_msgs__msg__Yolov12Inference__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = yolov12_msgs__msg__Yolov12Inference__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
yolov12_msgs__msg__Yolov12Inference__Sequence__destroy(yolov12_msgs__msg__Yolov12Inference__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    yolov12_msgs__msg__Yolov12Inference__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
yolov12_msgs__msg__Yolov12Inference__Sequence__are_equal(const yolov12_msgs__msg__Yolov12Inference__Sequence * lhs, const yolov12_msgs__msg__Yolov12Inference__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!yolov12_msgs__msg__Yolov12Inference__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
yolov12_msgs__msg__Yolov12Inference__Sequence__copy(
  const yolov12_msgs__msg__Yolov12Inference__Sequence * input,
  yolov12_msgs__msg__Yolov12Inference__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(yolov12_msgs__msg__Yolov12Inference);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    yolov12_msgs__msg__Yolov12Inference * data =
      (yolov12_msgs__msg__Yolov12Inference *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!yolov12_msgs__msg__Yolov12Inference__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          yolov12_msgs__msg__Yolov12Inference__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!yolov12_msgs__msg__Yolov12Inference__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
