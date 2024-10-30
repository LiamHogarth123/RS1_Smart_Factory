// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from warehouse_robot_msgs:msg/RobotData.idl
// generated code does not contain a copyright notice

#ifndef WAREHOUSE_ROBOT_MSGS__MSG__DETAIL__ROBOT_DATA__FUNCTIONS_H_
#define WAREHOUSE_ROBOT_MSGS__MSG__DETAIL__ROBOT_DATA__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "warehouse_robot_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "warehouse_robot_msgs/msg/detail/robot_data__struct.h"

/// Initialize msg/RobotData message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * warehouse_robot_msgs__msg__RobotData
 * )) before or use
 * warehouse_robot_msgs__msg__RobotData__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_warehouse_robot_msgs
bool
warehouse_robot_msgs__msg__RobotData__init(warehouse_robot_msgs__msg__RobotData * msg);

/// Finalize msg/RobotData message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_warehouse_robot_msgs
void
warehouse_robot_msgs__msg__RobotData__fini(warehouse_robot_msgs__msg__RobotData * msg);

/// Create msg/RobotData message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * warehouse_robot_msgs__msg__RobotData__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_warehouse_robot_msgs
warehouse_robot_msgs__msg__RobotData *
warehouse_robot_msgs__msg__RobotData__create();

/// Destroy msg/RobotData message.
/**
 * It calls
 * warehouse_robot_msgs__msg__RobotData__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_warehouse_robot_msgs
void
warehouse_robot_msgs__msg__RobotData__destroy(warehouse_robot_msgs__msg__RobotData * msg);

/// Check for msg/RobotData message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_warehouse_robot_msgs
bool
warehouse_robot_msgs__msg__RobotData__are_equal(const warehouse_robot_msgs__msg__RobotData * lhs, const warehouse_robot_msgs__msg__RobotData * rhs);

/// Copy a msg/RobotData message.
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
ROSIDL_GENERATOR_C_PUBLIC_warehouse_robot_msgs
bool
warehouse_robot_msgs__msg__RobotData__copy(
  const warehouse_robot_msgs__msg__RobotData * input,
  warehouse_robot_msgs__msg__RobotData * output);

/// Initialize array of msg/RobotData messages.
/**
 * It allocates the memory for the number of elements and calls
 * warehouse_robot_msgs__msg__RobotData__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_warehouse_robot_msgs
bool
warehouse_robot_msgs__msg__RobotData__Sequence__init(warehouse_robot_msgs__msg__RobotData__Sequence * array, size_t size);

/// Finalize array of msg/RobotData messages.
/**
 * It calls
 * warehouse_robot_msgs__msg__RobotData__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_warehouse_robot_msgs
void
warehouse_robot_msgs__msg__RobotData__Sequence__fini(warehouse_robot_msgs__msg__RobotData__Sequence * array);

/// Create array of msg/RobotData messages.
/**
 * It allocates the memory for the array and calls
 * warehouse_robot_msgs__msg__RobotData__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_warehouse_robot_msgs
warehouse_robot_msgs__msg__RobotData__Sequence *
warehouse_robot_msgs__msg__RobotData__Sequence__create(size_t size);

/// Destroy array of msg/RobotData messages.
/**
 * It calls
 * warehouse_robot_msgs__msg__RobotData__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_warehouse_robot_msgs
void
warehouse_robot_msgs__msg__RobotData__Sequence__destroy(warehouse_robot_msgs__msg__RobotData__Sequence * array);

/// Check for msg/RobotData message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_warehouse_robot_msgs
bool
warehouse_robot_msgs__msg__RobotData__Sequence__are_equal(const warehouse_robot_msgs__msg__RobotData__Sequence * lhs, const warehouse_robot_msgs__msg__RobotData__Sequence * rhs);

/// Copy an array of msg/RobotData messages.
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
ROSIDL_GENERATOR_C_PUBLIC_warehouse_robot_msgs
bool
warehouse_robot_msgs__msg__RobotData__Sequence__copy(
  const warehouse_robot_msgs__msg__RobotData__Sequence * input,
  warehouse_robot_msgs__msg__RobotData__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // WAREHOUSE_ROBOT_MSGS__MSG__DETAIL__ROBOT_DATA__FUNCTIONS_H_
