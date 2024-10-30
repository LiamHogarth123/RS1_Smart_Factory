// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from warehouse_robot_msgs:msg/RobotData.idl
// generated code does not contain a copyright notice
#include "warehouse_robot_msgs/msg/detail/robot_data__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `odom`
#include "nav_msgs/msg/detail/odometry__functions.h"

bool
warehouse_robot_msgs__msg__RobotData__init(warehouse_robot_msgs__msg__RobotData * msg)
{
  if (!msg) {
    return false;
  }
  // odom
  if (!nav_msgs__msg__Odometry__init(&msg->odom)) {
    warehouse_robot_msgs__msg__RobotData__fini(msg);
    return false;
  }
  // status
  // ar_tag_id
  return true;
}

void
warehouse_robot_msgs__msg__RobotData__fini(warehouse_robot_msgs__msg__RobotData * msg)
{
  if (!msg) {
    return;
  }
  // odom
  nav_msgs__msg__Odometry__fini(&msg->odom);
  // status
  // ar_tag_id
}

bool
warehouse_robot_msgs__msg__RobotData__are_equal(const warehouse_robot_msgs__msg__RobotData * lhs, const warehouse_robot_msgs__msg__RobotData * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // odom
  if (!nav_msgs__msg__Odometry__are_equal(
      &(lhs->odom), &(rhs->odom)))
  {
    return false;
  }
  // status
  if (lhs->status != rhs->status) {
    return false;
  }
  // ar_tag_id
  if (lhs->ar_tag_id != rhs->ar_tag_id) {
    return false;
  }
  return true;
}

bool
warehouse_robot_msgs__msg__RobotData__copy(
  const warehouse_robot_msgs__msg__RobotData * input,
  warehouse_robot_msgs__msg__RobotData * output)
{
  if (!input || !output) {
    return false;
  }
  // odom
  if (!nav_msgs__msg__Odometry__copy(
      &(input->odom), &(output->odom)))
  {
    return false;
  }
  // status
  output->status = input->status;
  // ar_tag_id
  output->ar_tag_id = input->ar_tag_id;
  return true;
}

warehouse_robot_msgs__msg__RobotData *
warehouse_robot_msgs__msg__RobotData__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  warehouse_robot_msgs__msg__RobotData * msg = (warehouse_robot_msgs__msg__RobotData *)allocator.allocate(sizeof(warehouse_robot_msgs__msg__RobotData), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(warehouse_robot_msgs__msg__RobotData));
  bool success = warehouse_robot_msgs__msg__RobotData__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
warehouse_robot_msgs__msg__RobotData__destroy(warehouse_robot_msgs__msg__RobotData * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    warehouse_robot_msgs__msg__RobotData__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
warehouse_robot_msgs__msg__RobotData__Sequence__init(warehouse_robot_msgs__msg__RobotData__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  warehouse_robot_msgs__msg__RobotData * data = NULL;

  if (size) {
    data = (warehouse_robot_msgs__msg__RobotData *)allocator.zero_allocate(size, sizeof(warehouse_robot_msgs__msg__RobotData), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = warehouse_robot_msgs__msg__RobotData__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        warehouse_robot_msgs__msg__RobotData__fini(&data[i - 1]);
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
warehouse_robot_msgs__msg__RobotData__Sequence__fini(warehouse_robot_msgs__msg__RobotData__Sequence * array)
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
      warehouse_robot_msgs__msg__RobotData__fini(&array->data[i]);
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

warehouse_robot_msgs__msg__RobotData__Sequence *
warehouse_robot_msgs__msg__RobotData__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  warehouse_robot_msgs__msg__RobotData__Sequence * array = (warehouse_robot_msgs__msg__RobotData__Sequence *)allocator.allocate(sizeof(warehouse_robot_msgs__msg__RobotData__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = warehouse_robot_msgs__msg__RobotData__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
warehouse_robot_msgs__msg__RobotData__Sequence__destroy(warehouse_robot_msgs__msg__RobotData__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    warehouse_robot_msgs__msg__RobotData__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
warehouse_robot_msgs__msg__RobotData__Sequence__are_equal(const warehouse_robot_msgs__msg__RobotData__Sequence * lhs, const warehouse_robot_msgs__msg__RobotData__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!warehouse_robot_msgs__msg__RobotData__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
warehouse_robot_msgs__msg__RobotData__Sequence__copy(
  const warehouse_robot_msgs__msg__RobotData__Sequence * input,
  warehouse_robot_msgs__msg__RobotData__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(warehouse_robot_msgs__msg__RobotData);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    warehouse_robot_msgs__msg__RobotData * data =
      (warehouse_robot_msgs__msg__RobotData *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!warehouse_robot_msgs__msg__RobotData__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          warehouse_robot_msgs__msg__RobotData__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!warehouse_robot_msgs__msg__RobotData__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
