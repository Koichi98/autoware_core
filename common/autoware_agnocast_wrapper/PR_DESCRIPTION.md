## Description

Make `agnocast_wrapper::Node` expose the **same public API in both builds** (`ENABLE_AGNOCAST=0` / `=1`).

The non-Agnocast build used `using Node = rclcpp::Node`, which leaked the entire `rclcpp::Node` API. Code could then rely on members the Agnocast-build `Node` does not provide, compiling under `=0` but breaking under `=1`. Now the `=0` build is a standalone class that **owns** an internal `rclcpp::Node` and forwards the same curated members the Agnocast build exposes (basic info, time, node interfaces, callback groups, parameters, pub/sub, polling subscriber, client/service, `create_wall_timer`, `get_rclcpp_node()`). Cross-distro differences are handled too (`OnSetParametersCallbackType` alias; `create_client`/`create_service` accept `rclcpp::QoS`, converted to the rmw profile on Humble).

Because `Node` is no longer an `rclcpp::Node`, helpers that took `rclcpp::Node*` now go through `get_rclcpp_node()`: `diagnostic_updater.hpp`, `tf2.hpp`, the free `create_timer()`, and `message_filters.hpp` (`Subscriber` becomes a thin wrapper taking the wrapper `Node*`). See the README "Supported API surface" table for the full list.

## Related links

**Parent Issue:**

- Link

## How was this PR tested?

- Builds with `ENABLE_AGNOCAST=0` and `=1`
- Existing nodes using `agnocast_wrapper::Node` compile unchanged under both

## Notes for reviewers

None.

## Interface changes

None. (C++ wrapper API only; no topics/parameters. No runtime behavior change — members forward to the internal `rclcpp::Node`.)

## Effects on system behavior

None.
