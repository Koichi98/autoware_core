// Copyright 2015-2021 Autoware Foundation
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

// VehicleInfoUtils is now header-only: its constructor is a member template (parameterized on the
// node type) and getVehicleInfo() is a trivial inline accessor, both defined in
// vehicle_info_utils.hpp. This translation unit is kept so the library target still has a source
// file to compile.
#include "autoware/vehicle_info_utils/vehicle_info_utils.hpp"
