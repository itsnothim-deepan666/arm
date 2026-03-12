# Robotic Arm — MuJoCo Simulation

A robotic arm exported from OnShape and loaded into a MuJoCo physics simulation.

---

## Project Structure

```
arm/
├── robot.urdf          # Robot description (links, joints, meshes)
├── arm_test.py         # Main simulation script
├── mujoco_test.py      # Basic MuJoCo test script
├── assets/             # STL mesh files for each part
│   ├── base_link.stl
│   ├── left.stl
│   ├── right.stl
│   ├── jaw1.stl
│   ├── jaw2.stl
│   ├── jaw3.stl
│   ├── jaw4.stl
│   ├── jaw5.stl
│   └── jaw6.stl
└── .gitignore
```

---

## Setup

### Requirements

- Python 3.x
- MuJoCo Python bindings

### Install

```bash
pip install mujoco
```

### Run

```bash
cd /home/deepan/build/arm
python arm_test.py
```

> Make sure to run from the project root directory so MuJoCo can resolve the mesh paths relative to `robot.urdf`.

---

## Robot Description (`robot.urdf`)

### Links

| Link | Mesh | Mass |
|------|------|------|
| `base_link` | base_link.stl | 1.0 kg |
| `left` | left.stl | 0.3 kg |
| `right` | right.stl | 0.3 kg |
| `jaw1` | jaw1.stl | 0.1 kg |
| `jaw2` | jaw2.stl | 0.1 kg |
| `jaw3` | jaw3.stl | 0.1 kg |
| `jaw4` | jaw4.stl | 0.1 kg |
| `jaw5` | jaw5.stl | 0.1 kg |
| `jaw6` | jaw6.stl | 0.1 kg |

### Joints

| Joint | Type | Parent → Child | Axis | Status |
|-------|------|----------------|------|--------|
| `base_to_left` | fixed | base_link → left | — | ✅ Done |
| `base_to_right` | fixed | base_link → right | — | ✅ Done |
| `base_to_jaw1` | fixed | base_link → jaw1 | — | ✅ Done |
| `jaw1_to_jaw2` | **revolute** | jaw1 → jaw2 | Y-axis | ✅ Done |
| `jaw2_to_jaw3` | fixed | jaw2 → jaw3 | — | ⚠️ To be set as revolute |
| `jaw3_to_jaw4` | fixed | jaw3 → jaw4 | — | ⚠️ To be set as revolute |
| `jaw4_to_jaw5` | fixed | jaw4 → jaw5 | — | ⚠️ To be set as revolute |
| `jaw5_to_jaw6` | fixed | jaw5 → jaw6 | — | ⚠️ To be set as revolute |

### MuJoCo Compiler Tag

The URDF includes a `<mujoco>` block to tell MuJoCo where to find the mesh files:

```xml
<mujoco>
  <compiler meshdir="assets"/>
</mujoco>
```

This is needed because MuJoCo doesn't understand ROS `package://` paths.

---

## URDF Fixes Applied

The original URDF was exported from OnShape on Windows and had several issues fixed:

1. **Mesh paths used `package://` prefix** — not understood by MuJoCo. Fixed to plain filenames with `meshdir` compiler tag.
2. **Backslash path separators (`\`)** — Windows style, broken on Linux. Fixed to forward slashes.
3. **Mesh origins were in world-space CAD coordinates** — caused parts to appear far off-screen. Reset to `xyz="0 0 0"`.
4. **All parts black (`rgba="0 0 0 1"`)** — invisible against the dark MuJoCo background. Changed to light grey `rgba="0.8 0.8 0.8 1"`.
5. **Only `right` link existed** — all other links (`base_link`, `left`, `jaw1`–`jaw6`) were missing. Added manually.

---

## Defining a Revolute Joint

To convert a `fixed` joint to a `revolute`:

```xml
<joint name="jaw1_to_jaw2" type="revolute">
  <parent link="jaw1"/>
  <child link="jaw2"/>
  <origin xyz="0 0 0" rpy="0 0 0"/>   <!-- pivot point in parent frame -->
  <axis xyz="0 1 0"/>                  <!-- rotation axis (Y here) -->
  <limit lower="-1.57" upper="1.57" effort="10" velocity="1.0"/>
  <dynamics damping="1.0" friction="0.1"/>
</joint>
```

### Key attributes

| Attribute | Description |
|-----------|-------------|
| `axis` | The 3D vector the joint rotates around (`0 1 0` = Y-axis) |
| `limit lower/upper` | Joint angle limits in radians (`±1.57` = ±90°) |
| `effort` | Max torque (Nm) |
| `velocity` | Max angular velocity (rad/s) |
| `damping` | Resistance to motion — prevents simulation instability |
| `friction` | Friction at the joint |

> ⚠️ **Without `damping`**, a constant torque will accelerate the joint infinitely → NaN/Inf instability.

### Setting the correct pivot point (`<origin>`)

The `<origin>` inside a joint defines **where the pivot is in the parent link's coordinate frame**.

Since all STL meshes were exported from CAD with real-world vertex positions baked in, all joint origins are currently `0 0 0`. To properly assemble the arm, measure the actual hinge positions from your CAD model (OnShape) and set them here.

To find body positions in MuJoCo:

```python
import mujoco
model = mujoco.MjModel.from_xml_path("robot.urdf")
for i in range(model.nbody):
    print(model.body(i).name, model.body(i).pos)
```

---

## Testing Joints in the Viewer

Run the simulation and use these keyboard shortcuts in the MuJoCo viewer:

| Shortcut | Action |
|----------|--------|
| `Ctrl+R` | Open joint slider panel — drag to test each joint manually |
| `Ctrl+D` | Show body coordinate frames as axes |
| `Ctrl+A` | Auto-fit camera to scene |
| Scroll wheel | Zoom in/out |

---

## TODO

- [ ] Measure actual joint pivot positions from OnShape CAD
- [ ] Set correct `<origin xyz="...">` for each joint
- [ ] Convert remaining joints (`jaw2_to_jaw3` through `jaw5_to_jaw6`) to `revolute`
- [ ] Add actuators for all revolute joints
- [ ] Add a control loop to command joint positions

---

## Simulation Script (`arm_test.py`)

```python
import mujoco
import mujoco.viewer

model = mujoco.MjModel.from_xml_path("robot.urdf")
data = mujoco.MjData(model)

with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        mujoco.mj_step(model, data)
        viewer.sync()  # Required to update the viewer each frame
```

> `viewer.sync()` must be called every loop iteration — without it the viewer freezes on the first frame.
