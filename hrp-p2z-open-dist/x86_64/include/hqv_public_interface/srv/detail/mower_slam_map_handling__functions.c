// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from hqv_public_interface:srv/MowerSlamMapHandling.idl
// generated code does not contain a copyright notice
#include "hqv_public_interface/srv/detail/mower_slam_map_handling__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

bool
hqv_public_interface__srv__MowerSlamMapHandling_Request__init(hqv_public_interface__srv__MowerSlamMapHandling_Request * msg)
{
  if (!msg) {
    return false;
  }
  // message_type
  // map_id
  return true;
}

void
hqv_public_interface__srv__MowerSlamMapHandling_Request__fini(hqv_public_interface__srv__MowerSlamMapHandling_Request * msg)
{
  if (!msg) {
    return;
  }
  // message_type
  // map_id
}

hqv_public_interface__srv__MowerSlamMapHandling_Request *
hqv_public_interface__srv__MowerSlamMapHandling_Request__create()
{
  hqv_public_interface__srv__MowerSlamMapHandling_Request * msg = (hqv_public_interface__srv__MowerSlamMapHandling_Request *)malloc(sizeof(hqv_public_interface__srv__MowerSlamMapHandling_Request));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(hqv_public_interface__srv__MowerSlamMapHandling_Request));
  bool success = hqv_public_interface__srv__MowerSlamMapHandling_Request__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
hqv_public_interface__srv__MowerSlamMapHandling_Request__destroy(hqv_public_interface__srv__MowerSlamMapHandling_Request * msg)
{
  if (msg) {
    hqv_public_interface__srv__MowerSlamMapHandling_Request__fini(msg);
  }
  free(msg);
}


bool
hqv_public_interface__srv__MowerSlamMapHandling_Request__Sequence__init(hqv_public_interface__srv__MowerSlamMapHandling_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  hqv_public_interface__srv__MowerSlamMapHandling_Request * data = NULL;
  if (size) {
    data = (hqv_public_interface__srv__MowerSlamMapHandling_Request *)calloc(size, sizeof(hqv_public_interface__srv__MowerSlamMapHandling_Request));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = hqv_public_interface__srv__MowerSlamMapHandling_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        hqv_public_interface__srv__MowerSlamMapHandling_Request__fini(&data[i - 1]);
      }
      free(data);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
hqv_public_interface__srv__MowerSlamMapHandling_Request__Sequence__fini(hqv_public_interface__srv__MowerSlamMapHandling_Request__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      hqv_public_interface__srv__MowerSlamMapHandling_Request__fini(&array->data[i]);
    }
    free(array->data);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

hqv_public_interface__srv__MowerSlamMapHandling_Request__Sequence *
hqv_public_interface__srv__MowerSlamMapHandling_Request__Sequence__create(size_t size)
{
  hqv_public_interface__srv__MowerSlamMapHandling_Request__Sequence * array = (hqv_public_interface__srv__MowerSlamMapHandling_Request__Sequence *)malloc(sizeof(hqv_public_interface__srv__MowerSlamMapHandling_Request__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = hqv_public_interface__srv__MowerSlamMapHandling_Request__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
hqv_public_interface__srv__MowerSlamMapHandling_Request__Sequence__destroy(hqv_public_interface__srv__MowerSlamMapHandling_Request__Sequence * array)
{
  if (array) {
    hqv_public_interface__srv__MowerSlamMapHandling_Request__Sequence__fini(array);
  }
  free(array);
}


bool
hqv_public_interface__srv__MowerSlamMapHandling_Response__init(hqv_public_interface__srv__MowerSlamMapHandling_Response * msg)
{
  if (!msg) {
    return false;
  }
  // map_id
  // map_ids
  return true;
}

void
hqv_public_interface__srv__MowerSlamMapHandling_Response__fini(hqv_public_interface__srv__MowerSlamMapHandling_Response * msg)
{
  if (!msg) {
    return;
  }
  // map_id
  // map_ids
}

hqv_public_interface__srv__MowerSlamMapHandling_Response *
hqv_public_interface__srv__MowerSlamMapHandling_Response__create()
{
  hqv_public_interface__srv__MowerSlamMapHandling_Response * msg = (hqv_public_interface__srv__MowerSlamMapHandling_Response *)malloc(sizeof(hqv_public_interface__srv__MowerSlamMapHandling_Response));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(hqv_public_interface__srv__MowerSlamMapHandling_Response));
  bool success = hqv_public_interface__srv__MowerSlamMapHandling_Response__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
hqv_public_interface__srv__MowerSlamMapHandling_Response__destroy(hqv_public_interface__srv__MowerSlamMapHandling_Response * msg)
{
  if (msg) {
    hqv_public_interface__srv__MowerSlamMapHandling_Response__fini(msg);
  }
  free(msg);
}


bool
hqv_public_interface__srv__MowerSlamMapHandling_Response__Sequence__init(hqv_public_interface__srv__MowerSlamMapHandling_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  hqv_public_interface__srv__MowerSlamMapHandling_Response * data = NULL;
  if (size) {
    data = (hqv_public_interface__srv__MowerSlamMapHandling_Response *)calloc(size, sizeof(hqv_public_interface__srv__MowerSlamMapHandling_Response));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = hqv_public_interface__srv__MowerSlamMapHandling_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        hqv_public_interface__srv__MowerSlamMapHandling_Response__fini(&data[i - 1]);
      }
      free(data);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
hqv_public_interface__srv__MowerSlamMapHandling_Response__Sequence__fini(hqv_public_interface__srv__MowerSlamMapHandling_Response__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      hqv_public_interface__srv__MowerSlamMapHandling_Response__fini(&array->data[i]);
    }
    free(array->data);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

hqv_public_interface__srv__MowerSlamMapHandling_Response__Sequence *
hqv_public_interface__srv__MowerSlamMapHandling_Response__Sequence__create(size_t size)
{
  hqv_public_interface__srv__MowerSlamMapHandling_Response__Sequence * array = (hqv_public_interface__srv__MowerSlamMapHandling_Response__Sequence *)malloc(sizeof(hqv_public_interface__srv__MowerSlamMapHandling_Response__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = hqv_public_interface__srv__MowerSlamMapHandling_Response__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
hqv_public_interface__srv__MowerSlamMapHandling_Response__Sequence__destroy(hqv_public_interface__srv__MowerSlamMapHandling_Response__Sequence * array)
{
  if (array) {
    hqv_public_interface__srv__MowerSlamMapHandling_Response__Sequence__fini(array);
  }
  free(array);
}
