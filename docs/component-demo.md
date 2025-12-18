---
id: component-demo
title: Custom Component Demo
description: A showcase of all custom MDX components available in this book
sidebar_label: 🧩 Component Demo
sidebar_position: 99
---

# Custom Component Demo

This page demonstrates all the custom MDX components you can use throughout this book. Copy and paste these examples into your content!

---

## TL;DR Box

The `TLDRBox` component is perfect for quick summaries at the start of sections—optimized for RAG retrieval.

<TLDRBox>
This chapter teaches you how ROS 2 nodes communicate using topics and services. By the end, you'll understand pub/sub patterns, service calls, and why your robot isn't responding to your desperate pleas.
</TLDRBox>

**Usage:**
```jsx
<TLDRBox>
Your quick summary goes here. Keep it under 2-3 sentences.
</TLDRBox>

// Custom title
<TLDRBox title="Quick Take">
Alternative title for the box.
</TLDRBox>
```

---

## Callout Boxes

The `Callout` component comes in several flavors for different contexts.

### Info Callout
<Callout type="info">
ROS 2 uses DDS (Data Distribution Service) under the hood. Think of it as the postal system for robots, except it actually delivers on time.
</Callout>

### Tip Callout
<Callout type="tip">
Always use `colcon build --symlink-install` during development. Your future self will thank you when you don't have to rebuild after every tiny change.
</Callout>

### Warning Callout
<Callout type="warning">
Never run `rm -rf /` on your robot. Yes, someone has done this. No, we won't tell you who.
</Callout>

### Danger Callout
<Callout type="danger">
If your robot starts questioning the meaning of existence, unplug it immediately. We're not ready for that conversation.
</Callout>

### Joke Callout
<Callout type="joke">
Why did the ROS node cross the network? To get to the other callback! ...I'll see myself out.
</Callout>

### Robot Callout
<Callout type="robot">
Beep boop. As a robot, I find it offensive that you assume I'll fall over. I prefer the term "aggressive floor inspection."
</Callout>

### Custom Title & Emoji
<Callout type="tip" title="Hot Take" emoji="🔥">
URDF is just XML cosplaying as a robot description language. Fight me.
</Callout>

**Usage:**
```jsx
<Callout type="info">Your info content</Callout>
<Callout type="tip">Your tip content</Callout>
<Callout type="warning">Your warning content</Callout>
<Callout type="danger">Your danger content</Callout>
<Callout type="joke">Your joke content</Callout>
<Callout type="robot">Robot speaking content</Callout>

// Custom title and emoji
<Callout type="tip" title="Custom Title" emoji="🎯">
Content here
</Callout>
```

---

## Quiz Component

The `Quiz` component adds interactive knowledge checks to your content.

<Quiz title="ROS 2 Fundamentals Check">
  <QuizQuestion
    question="What is the primary communication pattern used by ROS 2 topics?"
    options={[
      { id: "a", text: "Request-Response" },
      { id: "b", text: "Publish-Subscribe" },
      { id: "c", text: "Smoke Signals" },
      { id: "d", text: "Carrier Pigeon Protocol" }
    ]}
    correctId="b"
    explanation="Topics use the publish-subscribe pattern where publishers send messages to a topic and all subscribers receive them. It's like a group chat, but for robots."
    funFact="DDS can handle millions of messages per second. Your group chat wishes it could keep up."
  />
  <QuizQuestion
    question="What file format is used to describe a robot's physical structure in ROS?"
    options={[
      { id: "a", text: "JSON (because everything is JSON now)" },
      { id: "b", text: "YAML (the Python tax)" },
      { id: "c", text: "URDF (XML in disguise)" },
      { id: "d", text: "A series of interpretive dances" }
    ]}
    correctId="c"
    explanation="URDF (Unified Robot Description Format) uses XML to describe links, joints, and other robot properties. Yes, XML. In 2024. We cope."
  />
</Quiz>

**Usage:**
```jsx
<Quiz title="Your Quiz Title">
  <QuizQuestion
    question="Your question here?"
    options={[
      { id: "a", text: "Option A" },
      { id: "b", text: "Option B" },
      { id: "c", text: "Option C" },
      { id: "d", text: "Option D" }
    ]}
    correctId="b"
    explanation="Explanation shown after answering."
    funFact="Optional fun fact for correct answers."
  />
</Quiz>
```

---

## Progress Tracker

The `ProgressTracker` component shows learning progress through a module.

<ProgressTracker
  title="Module 1 Progress"
  steps={[
    { id: "1", title: "Introduction to ROS 2", status: "completed", description: "Why ROS 2 exists and why you should care" },
    { id: "2", title: "Nodes, Topics & Services", status: "completed", description: "The holy trinity of robot communication" },
    { id: "3", title: "Python with rclpy", status: "current", description: "Making Python talk to robots" },
    { id: "4", title: "URDF for Humanoids", status: "upcoming", description: "Describing your robot's fabulous body" },
    { id: "5", title: "Capstone Project", status: "upcoming", description: "Put it all together" }
  ]}
/>

**Usage:**
```jsx
<ProgressTracker
  title="Your Progress Title"
  showPercentage={true}
  steps={[
    { id: "1", title: "Step 1", status: "completed", description: "Optional description" },
    { id: "2", title: "Step 2", status: "current" },
    { id: "3", title: "Step 3", status: "upcoming" }
  ]}
/>
```

**Step statuses:**
- `completed` - Shows a green checkmark
- `current` - Shows a pulsing indicator
- `upcoming` - Shows a number, grayed out

---

## Code Blocks

Code blocks come with enhanced styling, copy buttons, and syntax highlighting.

### Python Example
```python title="my_first_node.py"
import rclpy
from rclpy.node import Node

class MyFirstNode(Node):
    """A simple ROS 2 node that proves you're not completely lost."""
    
    def __init__(self):
        super().__init__('my_first_node')
        self.get_logger().info('Hello, Robot World! 🤖')
        # highlight-next-line
        self.timer = self.create_timer(1.0, self.timer_callback)
    
    def timer_callback(self):
        self.get_logger().info('Still alive... unlike your social life during this course')

def main():
    rclpy.init()
    node = MyFirstNode()
    rclpy.spin(node)
    rclpy.shutdown()
```

### Shell Commands
```bash title="Terminal"
# Build your workspace (prepare for the waiting game)
colcon build --symlink-install

# Source the workspace (the ritual we all perform)
source install/setup.bash

# Run your node (moment of truth)
ros2 run my_package my_first_node
```

### URDF Example
```xml title="robot.urdf"
<?xml version="1.0"?>
<robot name="my_humanoid">
  <!-- The base link - where all the magic starts -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.3 0.1"/>
      </geometry>
    </visual>
  </link>
  
  <!-- A joint, because robots need to move -->
  <joint name="torso_joint" type="revolute">
    <parent link="base_link"/>
    <child link="torso"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1.0"/>
  </joint>
</robot>
```

---

## Combining Components

You can combine these components for rich, engaging content:

<TLDRBox>
Services in ROS 2 are like ordering food: you send a request, wait awkwardly, and eventually get a response. Unlike food delivery, the response usually arrives.
</TLDRBox>

<Callout type="warning">
Don't call services in callbacks. It's like calling tech support while you're already on hold with tech support—nothing good comes from it.
</Callout>

```python title="service_example.py"
from example_interfaces.srv import AddTwoInts

# Create a service client
client = node.create_client(AddTwoInts, 'add_two_ints')

# Wait for the service (patience is a virtue)
while not client.wait_for_service(timeout_sec=1.0):
    node.get_logger().info('Waiting for service... still waiting... any day now...')
```

<Callout type="tip" title="Pro Move" emoji="💪">
Use `async` service calls in production code. Your robot will thank you by not freezing mid-task.
</Callout>

---

Now go forth and build amazing documentation! 🚀
