/****************************************************************************
 * libs/libc/sched/sched_getprioritymin.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <assert.h>

#include <nuttx/arch.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sched_get_priority_min
 *
 * Description:
 *   This function returns the value of the lowest possible
 *   task priority for a specified scheduling policy.
 *
 * Input Parameters:
 *   policy - Scheduling policy requested.
 *
 * Returned Value:
 *   The minimum priority value or -1 (ERROR)
 *   (errno is not set)
 *
 * Assumptions:
 *
 ****************************************************************************/

int sched_get_priority_min(int policy)
{
  DEBUGASSERT(policy >= SCHED_OTHER && policy <= SCHED_SPORADIC);
  return SCHED_PRIORITY_MIN;
}
