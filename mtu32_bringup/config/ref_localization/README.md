Per-robot overrides of `../ref_localization.yaml`, loaded after it by `bringup_main.launch.py`:

- `assignments.yaml`: which Motive rigid body each robot follows, written by multirobot_sim's web UI
  (Motion capture card), keyed by node (`/<ns>/natnet_ref_pose`).
- `<namespace>.yaml` (e.g. `a200_0284.yaml`), hand-written, loaded last. With `colcon build --symlink-install` a
  *new* file here needs `colcon build --packages-select mtu32_bringup` before a launch sees it. Typical content:

```yaml
/**/natnet_ref_pose:
  ros__parameters:
    base_link_offset: [0.0, 0.0, -0.28, 0.0, 0.0, 0.0, 1.0] # base_link in the rigid body's frame
```

  Keep `rigid_body` out of it: the robot-to-rigid-body mapping is `assignments.yaml` alone (the web UI shows and
  edits only that file, and a `rigid_body` here, loaded last, would silently override it).

Measure `base_link_offset` with `mocap_fake_localizer`'s `calibrate_ref_offset.py --yes --write <this folder>/<namespace>.yaml`
(drives the robot: forward/back and one turn in place).
