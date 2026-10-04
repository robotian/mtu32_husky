Per-robot overrides of `../ref_localization.yaml`, one file per robot namespace (`<namespace>.yaml`, e.g.
`a200_0284.yaml`), loaded after the shared file by `bringup_main.launch.py`. Typical content:

```yaml
/**/natnet_ref_pose:
  ros__parameters:
    rigid_body: A200_1                                      # Motive's name, if it isn't the namespace
    base_link_offset: [0.0, 0.0, -0.28, 0.0, 0.0, 0.0, 1.0] # base_link in the rigid body's frame
```
