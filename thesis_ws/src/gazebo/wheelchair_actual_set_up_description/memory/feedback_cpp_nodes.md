---
name: feedback-cpp-nodes
description: All ROS nodes in this project must be written in C++, not Python
metadata:
  type: feedback
---

All ROS nodes must be implemented in C++.

**Why:** User explicitly stated this as a project rule.

**How to apply:** When adding any new ROS node (publisher, subscriber, broadcaster, etc.) to any package in this workspace, always write it as a C++ source file and compile it via CMakeLists.txt. Never create Python ROS node scripts (.py) for this project.
