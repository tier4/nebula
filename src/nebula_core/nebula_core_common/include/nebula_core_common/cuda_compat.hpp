// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

/// @brief Macros for annotating functions that can be compiled for both host (CPU) and device
/// (GPU). When compiled with nvcc (__CUDACC__), these expand to CUDA qualifiers. When compiled with
/// a regular C++ compiler, they expand to nothing.

#ifdef __CUDACC__
#define NEBULA_HOST_DEVICE __host__ __device__
#define NEBULA_DEVICE __device__
#else
#define NEBULA_HOST_DEVICE
#define NEBULA_DEVICE
#endif
